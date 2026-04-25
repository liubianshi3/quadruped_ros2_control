#include "dog2_mpc/mpc_controller.hpp"
#include <chrono>
#include <iostream>

namespace dog2_mpc {

MPCController::MPCController(double mass,
                             const Eigen::Matrix3d& inertia,
                             const Parameters& params)
    : params_(params),
      base_foot_positions_(Eigen::MatrixXd::Zero(4, 3)),
      sliding_velocity_(Eigen::Vector4d::Zero()),
      crossing_enabled_(false),
      freeze_crossing_rail_targets_(false),
      last_solve_time_(0.0),
      last_solve_status_(-1),
      initialized_(false) {
    
    // 创建扩展SRBD模型（包含滑动副）
    extended_srbd_model_ = std::make_unique<ExtendedSRBDModel>(mass, inertia);
    
    // 创建OSQP求解器
    osqp_solver_ = std::make_unique<OSQPInterface>();
    
    // 创建越障功能组件
    crossing_state_machine_ = std::make_unique<CrossingStateMachine>();
    hybrid_gait_generator_ = std::make_unique<HybridGaitGenerator>();
    
    // 初始化参考轨迹（16维扩展状态）
    x_ref_.resize(params_.horizon, Eigen::VectorXd::Zero(16));
    
    // 初始化线性化矩阵（16维扩展）
    A_matrices_.resize(params_.horizon);
    B_matrices_.resize(params_.horizon);
    C_matrices_.resize(params_.horizon);
    
    // 初始化预测结果（16维扩展状态）
    x_predicted_.resize(params_.horizon, Eigen::VectorXd::Zero(16));
    u_predicted_.resize(params_.horizon, Eigen::VectorXd::Zero(12));
    
    initialized_ = true;

    current_slack_weight_ = 1e5;  // 默认 exact penalty 权重（可由 Node 运行时调整）
    
    std::cout << "[MPCController] 16维扩展MPC初始化完成，包含滑动副约束" << std::endl;
}

void MPCController::setReference(const std::vector<Eigen::VectorXd>& x_ref) {
    if (x_ref.size() != static_cast<size_t>(params_.horizon)) {
        std::cerr << "Warning: Reference trajectory size mismatch. Expected "
                  << params_.horizon << ", got " << x_ref.size() << std::endl;
        return;
    }
    
    // 检查参考轨迹维度
    for (const auto& x : x_ref) {
        if (x.size() != 16) {
            std::cerr << "Warning: Reference state must be 16-dimensional (extended state)" << std::endl;
            return;
        }
    }
    
    x_ref_ = x_ref;
}

void MPCController::setBaseFootPositions(const Eigen::MatrixXd& foot_positions) {
    if (foot_positions.rows() != 4 || foot_positions.cols() != 3) {
        std::cerr << "Warning: Base foot positions must be 4×3 matrix" << std::endl;
        return;
    }
    base_foot_positions_ = foot_positions;
}

void MPCController::setSlidingVelocity(const Eigen::Vector4d& sliding_velocity) {
    sliding_velocity_ = sliding_velocity;
    extended_srbd_model_->setSlidingVelocity(sliding_velocity);
}

void MPCController::setCrossingGuardParams(double rail_tracking_error_threshold,
                                          double support_polygon_margin_threshold,
                                          double transition_stable_time) {
    if (crossing_state_machine_) {
        crossing_state_machine_->setRailTrackingErrorThreshold(rail_tracking_error_threshold);
        crossing_state_machine_->setSupportPolygonMarginThreshold(support_polygon_margin_threshold);
        crossing_state_machine_->setTransitionStableTime(transition_stable_time);
    }
}

void MPCController::setSlackLinearWeight(double slack_linear_weight) {
    current_slack_weight_ = slack_linear_weight;
}

bool MPCController::solve(const Eigen::VectorXd& x0, Eigen::VectorXd& u_optimal) {
    std::cout << "[MPC] 16维扩展solve方法开始，x0.size()=" << x0.size() << std::endl;
    
    if (!initialized_) {
        std::cerr << "Error: MPC controller not initialized" << std::endl;
        return false;
    }
    
    if (x0.size() != 16) {
        std::cerr << "Error: Initial state must be 16-dimensional (extended state)" << std::endl;
        return false;
    }
    
    std::cout << "[MPC] 基本检查通过" << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. 更新越障状态（如果启用）
    std::cout << "[MPC] crossing_enabled_=" << crossing_enabled_ << std::endl;
    if (crossing_enabled_ && crossing_state_machine_) {
        std::cout << "[MPC] 更新越障状态..." << std::endl;
        updateCrossingState(x0);
        std::cout << "[MPC] 越障状态更新完成" << std::endl;
    }
    
    // 2. 线性化整个时域（16维扩展）
    std::cout << "[MPC] 开始线性化时域..." << std::endl;
    linearizeHorizon(x0);
    std::cout << "[MPC] 线性化完成" << std::endl;
    
    // 3. 构建QP问题（包含滑动副约束）
    std::cout << "[MPC] 开始构建16维QP问题..." << std::endl;
    Eigen::SparseMatrix<double> P, A;
    Eigen::VectorXd q, l, u;
    buildQP(x0, P, q, A, l, u);
    std::cout << "[MPC] 16维QP问题构建完成" << std::endl;
    
    // 4. 设置OSQP求解器
    std::cout << "[MPC] 设置OSQP求解器..." << std::endl;
    bool setup_success = osqp_solver_->setup(P, q, A, l, u);
    if (!setup_success) {
        std::cerr << "Error: OSQP setup failed" << std::endl;
        return false;
    }
    std::cout << "[MPC] OSQP设置成功" << std::endl;
    
    // 5. 求解
    Eigen::VectorXd solution;
    bool success = osqp_solver_->solve(solution);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    last_solve_time_ = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    last_solve_status_ = osqp_solver_->getStatus();
    
    if (!success) {
        std::cerr << "Error: OSQP solver failed with status " 
                  << last_solve_status_ << std::endl;
        return false;
    }
    
    // 6. 提取解
    extractSolution(solution);
    
    // 7. 返回第一步控制
    u_optimal = u_predicted_[0];
    
    return true;
}

bool MPCController::solve(const Eigen::VectorXd& srbd_state, 
                         const Eigen::Vector4d& sliding_positions,
                         Eigen::VectorXd& u_optimal) {
    // 构建16维扩展状态
    Eigen::VectorXd extended_state = ExtendedSRBDModel::buildExtendedState(srbd_state, sliding_positions);
    
    // 调用主solve方法
    return solve(extended_state, u_optimal);
}

void MPCController::linearizeHorizon(const Eigen::VectorXd& x0) {
    Eigen::VectorXd x_current = x0;  // 16维扩展状态
    
    for (int k = 0; k < params_.horizon; ++k) {
        // 使用参考控制进行线性化（如果没有则用零）
        Eigen::VectorXd u_ref = Eigen::VectorXd::Zero(12);
        
        // 扩展动力学线性化：A(16×16), B(16×12), C(16×4)
        Eigen::MatrixXd A_ext, B_ext, C_ext;
        extended_srbd_model_->linearize(x_current, u_ref, base_foot_positions_, 
                                       sliding_velocity_, params_.dt, A_ext, B_ext, C_ext);
        
        A_matrices_[k] = A_ext;
        B_matrices_[k] = B_ext;
        C_matrices_[k] = C_ext;
        
        // 前向传播用于下一步线性化
        Eigen::VectorXd x_next;
        extended_srbd_model_->discreteDynamics(x_current, u_ref, base_foot_positions_, 
                                              sliding_velocity_, params_.dt, x_next);
        
        x_current = x_next;
    }
}

void MPCController::buildQP(const Eigen::VectorXd& x0,
                            Eigen::SparseMatrix<double>& P,
                            Eigen::VectorXd& q,
                            Eigen::SparseMatrix<double>& A,
                            Eigen::VectorXd& l,
                            Eigen::VectorXd& u) {
    const int N = params_.horizon;
    const int nx = 16;  // 扩展状态维度：SRBD(12) + 滑动副(4)
    const int nu = 12;  // 控制维度不变

    // 仅在 crossing 模式下，把滑动副 rail 边界做真正的 soft bound：
    //   引入上下界松弛变量：
    //     d_i + s_lower(k,i) >= d_min(i)
    //     d_i - s_upper(k,i) <= d_max(i)
    //   并在目标函数中对 slack 做 exact penalty（q 一次项 + P 二次项）
    const bool crossing_approach =
        crossing_enabled_ && crossing_state_machine_ &&
        crossing_state_machine_->getCurrentState() == CrossingStateMachine::CrossingState::APPROACH;
    const bool use_soft_rail_bounds = crossing_enabled_ && crossing_state_machine_ &&
                                       params_.enable_sliding_constraints &&
                                       !crossing_approach &&
                                       !freeze_crossing_rail_targets_;
    const int num_rail_slack = use_soft_rail_bounds ? 8 : 0;  // 4 lower + 4 upper
    
    // 优化变量: z = [x_1, u_0, x_2, u_1, ..., x_N, u_{N-1}]
    const int nv_base = N * (nx + nu);
    const int nv = nv_base + (use_soft_rail_bounds ? N * num_rail_slack : 0);
    const int rail_slack_base = nv_base;
    
    std::cout << "[MPC] 构建16维QP问题: N=" << N << ", nx=" << nx << ", nu=" << nu << ", nv=" << nv << std::endl;
    
    // 1. 构建目标函数 Hessian 矩阵 P
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.reserve(nv);
    
    for (int k = 0; k < N; ++k) {
        // 变量索引：x_{k+1} 在位置 k*(nx+nu), u_k 在位置 k*(nx+nu)+nx
        int x_offset = k * (nx + nu);
        int u_offset = x_offset + nx;
        
        // 状态权重 Q (16×16)
        for (int i = 0; i < nx; ++i) {
            P_triplets.push_back(Eigen::Triplet<double>(
                x_offset + i, x_offset + i, params_.Q(i, i)));
        }
        
        // 控制权重 R (12×12)
        for (int i = 0; i < nu; ++i) {
            P_triplets.push_back(Eigen::Triplet<double>(
                u_offset + i, u_offset + i, params_.R(i, i)));
        }
    }

    // slack cost for soft rail bounds
    if (use_soft_rail_bounds) {
        const double slack_quadratic_weight = 1e3;  // 二次项（保持数值稳定）
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < 4; ++i) {
                const int s_lower_idx = rail_slack_base + k * 4 + i;
                const int s_upper_idx = rail_slack_base + N * 4 + k * 4 + i;

                P_triplets.push_back(Eigen::Triplet<double>(
                    s_lower_idx, s_lower_idx, slack_quadratic_weight));
                P_triplets.push_back(Eigen::Triplet<double>(
                    s_upper_idx, s_upper_idx, slack_quadratic_weight));
            }
        }
    }
    
    P.resize(nv, nv);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());
    
    // 2. 构建线性项 q
    q.resize(nv);
    q.setZero();

    // exact penalty：在线性项 q 中加入强惩罚，保证 slack s->0 时也有非零导数
    if (use_soft_rail_bounds) {
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < 4; ++i) {
                const int s_lower_idx = rail_slack_base + k * 4 + i;
                const int s_upper_idx = rail_slack_base + N * 4 + k * 4 + i;
                q(s_lower_idx) = current_slack_weight_;
                q(s_upper_idx) = current_slack_weight_;
            }
        }
    }
    
    for (int k = 0; k < N; ++k) {
        int x_offset = k * (nx + nu);  // x_{k+1}的位置
        int u_offset = x_offset + nx;  // u_k的位置
        
        // 状态跟踪项（16维）
        q.segment(x_offset, nx) = -params_.Q * x_ref_[k];
        
        // 重力补偿项：在 stage-3 简化 SRBD MPC 中，若没有足够的垂向力先验，
        // 短时域优化会倾向于“少用力、允许机身下坠”。这里给每条腿加入
        // 接近静态支撑力的线性偏置，帮助求解器在站立/低速行走时维持 body height。
        double weight_per_leg = extended_srbd_model_->getMass() * 9.81 / 4.0;
        for (int i = 0; i < 4; ++i) {
            q(u_offset + i * 3 + 2) = -1.0 * weight_per_leg;  // z方向，负号表示鼓励正值
        }
    }
    
    // 3. 构建约束
    std::vector<Eigen::Triplet<double>> A_triplets;
    std::vector<double> l_vec, u_vec;
    
    // 3.1 动力学约束（等式约束）
    addDynamicsConstraints(x0, A_triplets, l_vec, u_vec);
    
    // 3.2 滑动副约束（不等式约束）
    if (params_.enable_sliding_constraints) {
        addSlidingConstraints(A_triplets, l_vec, u_vec);
    }

    // 3.2.1 rail+window 碰撞约束（不等式约束）
    if (params_.enable_boundary_constraints) {
        addRailWindowConstraints(A_triplets, l_vec, u_vec);
    }
    
    // 3.3 控制约束（不等式约束）
    addControlConstraints(A_triplets, l_vec, u_vec);
    
    // 构建约束矩阵
    const int n_constraints = l_vec.size();
    A.resize(n_constraints, nv);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    
    l.resize(n_constraints);
    u.resize(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        l(i) = l_vec[i];
        u(i) = u_vec[i];
    }
    
    std::cout << "[MPC] 16维QP问题构建完成: " << nv << "变量, " << n_constraints << "约束, " 
              << A_triplets.size() << "非零元素" << std::endl;
    
    // 调试信息：检查约束矩阵的稀疏度
    if (A_triplets.size() > 0) {
        std::cout << "[MPC] 约束矩阵稀疏度: " << (double)A_triplets.size() / (n_constraints * nv) * 100.0 << "%" << std::endl;
    }
}

void MPCController::extractSolution(const Eigen::VectorXd& solution) {
    const int nx = 16;  // 扩展状态维度
    const int nu = 12;  // 控制维度
    
    // 变量排列: [x_1, u_0, x_2, u_1, ..., x_N, u_{N-1}]
    for (int k = 0; k < params_.horizon; ++k) {
        int x_offset = k * (nx + nu);      // x_{k+1}的位置
        int u_offset = x_offset + nx;      // u_k的位置
        
        x_predicted_[k] = solution.segment(x_offset, nx);  // 16维扩展状态
        u_predicted_[k] = solution.segment(u_offset, nu);  // 12维控制
    }
}

std::vector<Eigen::VectorXd> MPCController::getPredictedTrajectory() const {
    return x_predicted_;
}

std::vector<Eigen::VectorXd> MPCController::getPredictedControl() const {
    return u_predicted_;
}

double MPCController::getSolveTime() const {
    return last_solve_time_;
}

int MPCController::getSolveStatus() const {
    return last_solve_status_;
}

void MPCController::updateParameters(const Parameters& params) {
    params_ = params;
    
    // 重新调整大小（16维扩展状态）
    x_ref_.resize(params_.horizon, Eigen::VectorXd::Zero(16));
    A_matrices_.resize(params_.horizon);
    B_matrices_.resize(params_.horizon);
    C_matrices_.resize(params_.horizon);
    x_predicted_.resize(params_.horizon, Eigen::VectorXd::Zero(16));
    u_predicted_.resize(params_.horizon, Eigen::VectorXd::Zero(12));
}

// ========== 越障功能实现 ==========

void MPCController::initializeCrossing(const CrossingStateMachine::RobotState& initial_state,
                                      const CrossingStateMachine::WindowObstacle& window) {
    crossing_state_machine_->initialize(initial_state, window);
    hybrid_gait_generator_->initialize(initial_state);
    crossing_enabled_ = true;
    
    std::cout << "[MPCController] 越障功能已启用" << std::endl;
    std::cout << "  初始位置: x=" << initial_state.position.x() << "m" << std::endl;
    std::cout << "  窗框位置: x=" << window.x_position << "m" << std::endl;
}

bool MPCController::updateCrossing(const CrossingStateMachine::RobotState& current_state, double dt) {
    if (!crossing_enabled_) {
        return false;
    }
    
    return crossing_state_machine_->update(current_state, dt);
}

CrossingStateMachine::CrossingState MPCController::getCurrentCrossingState() const {
    if (!crossing_enabled_) {
        return CrossingStateMachine::CrossingState::APPROACH;
    }
    
    return crossing_state_machine_->getCurrentState();
}

double MPCController::getCrossingProgress() const {
    if (!crossing_enabled_) {
        return 0.0;
    }
    
    return crossing_state_machine_->getProgress();
}

std::vector<Eigen::VectorXd> MPCController::generateCrossingReference(
    const CrossingStateMachine::RobotState& current_state, double dt) {
    
    if (!crossing_enabled_) {
        return x_ref_;  // 返回默认参考轨迹
    }
    
    std::vector<Eigen::VectorXd> crossing_ref(params_.horizon, Eigen::VectorXd::Zero(12));
    
    // 获取当前越障状态
    auto crossing_state = crossing_state_machine_->getCurrentState();
    auto target_state = crossing_state_machine_->getTargetState();
    
    // 根据越障状态生成不同的参考轨迹
    switch (crossing_state) {
        case CrossingStateMachine::CrossingState::APPROACH: {
            // 接近阶段：正常Trot步态
            Eigen::Vector3d desired_velocity(0.2, 0.0, 0.0);  // 0.2 m/s前进
            auto gait_state = hybrid_gait_generator_->generateNormalTrotGait(current_state, desired_velocity, dt);
            
            for (int k = 0; k < params_.horizon; ++k) {
                crossing_ref[k].segment<3>(0) = gait_state.com_target;  // 位置
                crossing_ref[k].segment<3>(3) = target_state.orientation;  // 姿态
                crossing_ref[k].segment<3>(6) = gait_state.com_velocity_target;  // 线速度
                crossing_ref[k].segment<3>(9).setZero();  // 角速度
            }
            break;
        }
        
        case CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT: {
            // 机身前探：静态控制，滑动副伸展
            for (int k = 0; k < params_.horizon; ++k) {
                crossing_ref[k].segment<3>(0) = target_state.position;
                crossing_ref[k].segment<3>(3) = target_state.orientation;
                crossing_ref[k].segment<3>(6).setZero();  // 速度为0
                crossing_ref[k].segment<3>(9).setZero();
            }
            break;
        }
        
        case CrossingStateMachine::CrossingState::HYBRID_GAIT_WALKING: {
            // 混合构型行走：核心创新！
            Eigen::Vector3d desired_velocity(0.1, 0.0, 0.0);  // 慢速前进
            auto gait_state = hybrid_gait_generator_->generateHybridTrotGait(current_state, desired_velocity, dt);
            
            for (int k = 0; k < params_.horizon; ++k) {
                crossing_ref[k].segment<3>(0) = gait_state.com_target;
                crossing_ref[k].segment<3>(3) = target_state.orientation;
                crossing_ref[k].segment<3>(6) = gait_state.com_velocity_target;
                crossing_ref[k].segment<3>(9).setZero();
            }
            
            std::cout << "[MPCController] 混合构型行走模式激活！" << std::endl;
            break;
        }
        
        case CrossingStateMachine::CrossingState::RAIL_ALIGNMENT: {
            // 精确停车定位
            for (int k = 0; k < params_.horizon; ++k) {
                crossing_ref[k].segment<3>(0) = target_state.position;
                crossing_ref[k].segment<3>(3) = target_state.orientation;
                crossing_ref[k].segment<3>(6).setZero();  // 停止
                crossing_ref[k].segment<3>(9).setZero();
            }
            break;
        }
        
        default: {
            // 其他状态：使用目标状态
            for (int k = 0; k < params_.horizon; ++k) {
                crossing_ref[k].segment<3>(0) = target_state.position;
                crossing_ref[k].segment<3>(3) = target_state.orientation;
                crossing_ref[k].segment<3>(6) = target_state.velocity;
                crossing_ref[k].segment<3>(9) = target_state.angular_velocity;
            }
            break;
        }
    }
    
    return crossing_ref;
}

void MPCController::updateCrossingState(const Eigen::VectorXd& x0) {
    if (!crossing_enabled_ || !crossing_state_machine_) {
        return;
    }
    
    // 将MPC状态转换为CrossingStateMachine::RobotState
    CrossingStateMachine::RobotState robot_state;
    robot_state.position = x0.segment<3>(0);
    robot_state.velocity = x0.segment<3>(6);
    robot_state.orientation = x0.segment<3>(3);
    robot_state.angular_velocity = x0.segment<3>(9);
    
    // 滑动副状态：16D 扩展状态中滑动副位置在 [12..15]
    robot_state.sliding_positions = ExtendedSRBDModel::extractSlidingPositions(x0);
    robot_state.sliding_velocities = sliding_velocity_;

    // 腿部构型：为了闭环状态机 completion guards，使用当前 stage 的“期望构型”填充 leg_configs
    // （这样 guard 不会因为 leg_configs 未同步而永远无法触发）
    const auto stage = crossing_state_machine_->getCurrentState();
    for (int i = 0; i < 4; ++i) {
        bool is_front_leg = (i == 0 || i == 3);

        switch (stage) {
            case CrossingStateMachine::CrossingState::FRONT_LEGS_TRANSIT:
            case CrossingStateMachine::CrossingState::HYBRID_GAIT_WALKING:
            case CrossingStateMachine::CrossingState::RAIL_ALIGNMENT:
                robot_state.leg_configs[i] = is_front_leg
                    ? CrossingStateMachine::LegConfiguration::KNEE
                    : CrossingStateMachine::LegConfiguration::ELBOW;
                break;
            case CrossingStateMachine::CrossingState::REAR_LEGS_TRANSIT:
            case CrossingStateMachine::CrossingState::ALL_KNEE_STATE:
                // 前后腿都切到膝式（与 computeRearLegsTransitTarget/computeAllKneeStateTarget 保持一致）
                robot_state.leg_configs[i] = CrossingStateMachine::LegConfiguration::KNEE;
                break;
            case CrossingStateMachine::CrossingState::RECOVERY:
            case CrossingStateMachine::CrossingState::CONTINUE_FORWARD:
            case CrossingStateMachine::CrossingState::COMPLETED:
                robot_state.leg_configs[i] = CrossingStateMachine::LegConfiguration::ELBOW;
                break;
            default:
                robot_state.leg_configs[i] = CrossingStateMachine::LegConfiguration::ELBOW;
                break;
        }
        robot_state.foot_contacts[i] = true;
    }

    // 足端位置：CrossingStateMachine 的 foot_positions 被当作世界系量使用（用于与 window.x_position 比较）
    // SRBD/ExtendedSRBDModel 的 base_foot_positions_ 在这里视为“相对 CoM”的向量，因此需要加上 CoM 世界位姿。
    Eigen::MatrixXd rel_foot_positions = extended_srbd_model_->computeFootPositions(
        robot_state.sliding_positions, base_foot_positions_);
    for (int i = 0; i < 4; ++i) {
        if (rel_foot_positions.rows() >= 4 && rel_foot_positions.cols() >= 3) {
            robot_state.foot_positions[i] = robot_state.position + rel_foot_positions.row(i).transpose();
        } else {
            robot_state.foot_positions[i] = robot_state.position;
        }
    }
    
    // 更新状态机
    crossing_state_machine_->update(robot_state, params_.dt);
    
    // 更新参考轨迹（12维SRBD）
    auto crossing_ref_12d = generateCrossingReference(robot_state, params_.dt);
    
    // 扩展为16维（添加滑动副目标位置）
    x_ref_.resize(params_.horizon);
    auto target_state = crossing_state_machine_->getTargetState();
    for (int k = 0; k < params_.horizon; ++k) {
        x_ref_[k] = Eigen::VectorXd::Zero(16);
        x_ref_[k].segment<12>(0) = crossing_ref_12d[k];  // SRBD状态
        x_ref_[k].segment<4>(12) = freeze_crossing_rail_targets_
            ? robot_state.sliding_positions
            : target_state.sliding_positions;
    }
}

} // namespace dog2_mpc

namespace dog2_mpc {

void MPCController::addDynamicsConstraints(const Eigen::VectorXd& x0,
                                          std::vector<Eigen::Triplet<double>>& A_triplets,
                                          std::vector<double>& l_vec,
                                          std::vector<double>& u_vec) {
    const int N = params_.horizon;
    const int nx = 16;  // 扩展状态维度
    const int nu = 12;  // 控制维度
    
    // 动力学约束：x_{k+1} - A_k * x_k - B_k * u_k - C_k * v_k = 0
    // 其中 v_k 是滑动副速度（已知）
    
    for (int k = 0; k < N; ++k) {
        int constraint_offset = l_vec.size();
        int x_kp1_offset = k * (nx + nu);      // x_{k+1}的位置
        int u_k_offset = x_kp1_offset + nx;    // u_k的位置
        
        for (int i = 0; i < nx; ++i) {
            int row = constraint_offset + i;
            
            // x_{k+1} 项：系数为 1
            A_triplets.push_back(Eigen::Triplet<double>(row, x_kp1_offset + i, 1.0));
            
            // -B_k * u_k 项
            for (int j = 0; j < nu; ++j) {
                if (std::abs(B_matrices_[k](i, j)) > 1e-12) {
                    A_triplets.push_back(Eigen::Triplet<double>(
                        row, u_k_offset + j, -B_matrices_[k](i, j)));
                }
            }
            
            if (k == 0) {
                // 第一个约束: x_1 - B_0 * u_0 = A_0 * x_0 + C_0 * v_0
                Eigen::VectorXd rhs = A_matrices_[0] * x0 + C_matrices_[0] * sliding_velocity_;
                l_vec.push_back(rhs(i));
                u_vec.push_back(rhs(i));
            } else {
                // 后续约束: x_{k+1} - A_k * x_k - B_k * u_k = C_k * v_k
                // 添加 -A_k * x_k 项
                int x_k_offset = (k - 1) * (nx + nu);  // x_k的位置
                for (int j = 0; j < nx; ++j) {
                    if (std::abs(A_matrices_[k](i, j)) > 1e-12) {
                        A_triplets.push_back(Eigen::Triplet<double>(
                            row, x_k_offset + j, -A_matrices_[k](i, j)));
                    }
                }
                
                // 右侧：C_k * v_k
                double rhs = (C_matrices_[k] * sliding_velocity_)(i);
                l_vec.push_back(rhs);
                u_vec.push_back(rhs);
            }
        }
    }
}

void MPCController::addSlidingConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                                         std::vector<double>& l_vec,
                                         std::vector<double>& u_vec) {
    const int N = params_.horizon;
    const int nx = 16;
    const int nu = 12;
    const int sliding_start_idx = 12;  // 滑动副在状态向量中的起始索引

    if (crossing_enabled_ &&
        crossing_state_machine_ &&
        crossing_state_machine_->getCurrentState() == CrossingStateMachine::CrossingState::APPROACH) {
        // APPROACH 仍是低速靠近窗口阶段。此时 measured rail 可能已经被
        // WBC/物理限位带到任意合法边界，过早引入 crossing rail soft-bound、
        // symmetry 和 coordination guard 会把本来可行的行走 QP 推成 infeasible。
        // Rail 的真实越障重排从 BODY_FORWARD_SHIFT 开始再施加阶段约束。
        std::cout << "[MPC] APPROACH阶段跳过crossing rail约束，保持QP可行性" << std::endl;
        return;
    }
    
    // 获取滑动副限位（如果越障开启则使用 CrossingStateMachine 的 stage-specific constraints）
    Eigen::Vector4d d_min, d_max;
    Eigen::Vector4d v_slide_max_vec;
    if (crossing_enabled_ && crossing_state_machine_ &&
        !freeze_crossing_rail_targets_) {
        const auto stage_constraints = crossing_state_machine_->getCurrentConstraints();
        d_min = stage_constraints.sliding_min;
        d_max = stage_constraints.sliding_max;
        v_slide_max_vec = stage_constraints.sliding_vel_max;
    } else {
        // Canonical rail semantics from dog2.urdf.xacro:
        // lf/rh in [0.0, 0.111], lh/rf in [-0.111, 0.0].
        // Walking stage keeps rails near their current locked stance, so the
        // primary feasibility constraints should only enforce these per-leg
        // physical limits and rate bounds.
        d_min << 0.0, -0.111, 0.0, -0.111;
        d_max << 0.111, 0.0, 0.111, 0.0;
        v_slide_max_vec.setConstant(1.0);  // 最大速度 1 m/s
    }
    // These are guardrail constraints, not the primary rail target. Keep them
    // loose enough that entering crossing with measured rails near arbitrary
    // physical-limit positions does not make the first QP infeasible.
    double epsilon_sym = 0.15;
    double coord_tolerance = 0.25;
    
    int n_constraints_added = 0;

    // 1. rail 位置约束（真正 soft bound：slack variable + cost）
    //   - crossing 模式下：引入 s(k,i) >= 0
    //     d_i + s_i >= d_min
    //     d_i - s_i <= d_max
    //   - 非 crossing：直接硬约束
    const bool use_soft_rail_bounds =
        crossing_enabled_ && crossing_state_machine_ && !freeze_crossing_rail_targets_;
    const int nv_base = N * (nx + nu);
    const int slack_lower_base = nv_base;       // s_lower 放在末尾
    const int slack_upper_base = nv_base + N * 4;  // s_upper 在 s_lower 后面
    const double BIG = 10.0;
    const double osqp_infty = 1.0;  // rail bound slack 只需覆盖厘米级 rail 误差

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < 4; ++i) {
            const int d_var_idx = k * (nx + nu) + sliding_start_idx + i;

            if (!use_soft_rail_bounds) {
                // 硬约束：d_min <= d_i <= d_max
                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), d_var_idx, 1.0));
                l_vec.push_back(d_min(i));
                u_vec.push_back(d_max(i));
                n_constraints_added++;
                continue;
            }

            // 两个解耦 slack：下界松弛 / 上界松弛
            const int s_lower_var_idx = slack_lower_base + k * 4 + i;
            const int s_upper_var_idx = slack_upper_base + k * 4 + i;

            // slack lower: s_lower in [0, OSQP_INFTY]
            {
                const int row = l_vec.size();
                A_triplets.push_back(Eigen::Triplet<double>(row, s_lower_var_idx, 1.0));
                l_vec.push_back(0.0);
                u_vec.push_back(osqp_infty);
                n_constraints_added++;
            }

            // slack upper: s_upper in [0, OSQP_INFTY]
            {
                const int row = l_vec.size();
                A_triplets.push_back(Eigen::Triplet<double>(row, s_upper_var_idx, 1.0));
                l_vec.push_back(0.0);
                u_vec.push_back(osqp_infty);
                n_constraints_added++;
            }

            // lower: d_i + s_lower >= d_min
            {
                const int row = l_vec.size();
                A_triplets.push_back(Eigen::Triplet<double>(row, d_var_idx, 1.0));
                A_triplets.push_back(Eigen::Triplet<double>(row, s_lower_var_idx, 1.0));
                l_vec.push_back(d_min(i));
                u_vec.push_back(BIG);
                n_constraints_added++;
            }

            // upper: d_i - s_upper <= d_max
            {
                const int row = l_vec.size();
                A_triplets.push_back(Eigen::Triplet<double>(row, d_var_idx, 1.0));
                A_triplets.push_back(Eigen::Triplet<double>(row, s_upper_var_idx, -1.0));
                l_vec.push_back(-BIG);
                u_vec.push_back(d_max(i));
                n_constraints_added++;
            }
        }
    }
    
    // 2. 速度限制约束：-v_max*dt <= d[k+1] - d[k] <= v_max*dt
    for (int k = 0; k < N - 1; ++k) {
        for (int i = 0; i < 4; ++i) {
            int var_idx_k = k * (nx + nu) + sliding_start_idx + i;
            int var_idx_k1 = (k + 1) * (nx + nu) + sliding_start_idx + i;
            
            // d[k+1] - d[k]
            A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), var_idx_k1, 1.0));
            A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), var_idx_k, -1.0));
            const double v_slide_max = v_slide_max_vec(i);
            l_vec.push_back(-v_slide_max * params_.dt);
            u_vec.push_back(v_slide_max * params_.dt);
            n_constraints_added++;
        }
    }
    
    // Symmetry / coordination are useful as secondary objectives in later WBC
    // stages, but they are too restrictive for the current stage-3 simplified
    // walking pipeline, where rail joints should stay close to the measured
    // locked stance rather than dominate feasibility.
    if (crossing_enabled_ && !freeze_crossing_rail_targets_) {
        // 3. 对称约束：-epsilon <= d1 - d3 <= epsilon, -epsilon <= d2 - d4 <= epsilon
        for (int k = 0; k < N; ++k) {
            // d1 - d3
            {
                int d1_idx = k * (nx + nu) + sliding_start_idx + 0;
                int d3_idx = k * (nx + nu) + sliding_start_idx + 2;

                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), d1_idx, 1.0));
                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), d3_idx, -1.0));
                l_vec.push_back(-epsilon_sym);
                u_vec.push_back(epsilon_sym);
                n_constraints_added++;
            }

            // d2 - d4
            {
                int d2_idx = k * (nx + nu) + sliding_start_idx + 1;
                int d4_idx = k * (nx + nu) + sliding_start_idx + 3;

                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), d2_idx, 1.0));
                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), d4_idx, -1.0));
                l_vec.push_back(-epsilon_sym);
                u_vec.push_back(epsilon_sym);
                n_constraints_added++;
            }
        }

        // 4. 协调约束：-tolerance <= d1 + d2 + d3 + d4 <= tolerance
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < 4; ++i) {
                int var_idx = k * (nx + nu) + sliding_start_idx + i;
                A_triplets.push_back(Eigen::Triplet<double>(l_vec.size(), var_idx, 1.0));
            }
            l_vec.push_back(-coord_tolerance);
            u_vec.push_back(coord_tolerance);
            n_constraints_added++;
        }
    }
    
    std::cout << "[MPC] 滑动副约束已添加: " << n_constraints_added << "个约束" << std::endl;
}

void MPCController::addRailWindowConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                                            std::vector<double>& l_vec,
                                            std::vector<double>& u_vec) {
    if (!crossing_enabled_ || !crossing_state_machine_ ||
        freeze_crossing_rail_targets_) {
        return;
    }

    const int N = params_.horizon;
    const int nx = 16;  // 扩展状态维度
    const int nu = 12;  // 控制维度
    const int sliding_start_idx = 12;  // d1..d4 在状态向量中的起始索引

    const auto stage = crossing_state_machine_->getCurrentState();
    const auto window = crossing_state_machine_->getWindowObstacle();

    // 足端/rail 近似：world_x_foot ~= x_CoM + base_foot_x + d_i
    // 在越障跨越阶段，对部分腿施加“窗框平面”安全侧约束：
    //   transiting legs:  x_foot >= x_window + d_safe
    //   non-transiting legs: x_foot <= x_window - d_safe
    const double x_window = window.x_position;
    const double d_safe = window.safety_margin;
    const double BIG = 10.0;  // 用大上下界避免 OSQP 处理无穷

    // leg_mode:
    // 0: none
    // 1: GE (x >= x_window + d_safe)
    // 2: GE (x >= x_window - d_safe)
    // 3: LE (x <= x_window - d_safe)
    std::array<int, 4> leg_mode{0, 0, 0, 0};
    switch (stage) {
        case CrossingStateMachine::CrossingState::FRONT_LEGS_TRANSIT:
            leg_mode[0] = 1;
            leg_mode[3] = 1;
            leg_mode[1] = 3;
            leg_mode[2] = 3;
            break;
        case CrossingStateMachine::CrossingState::HYBRID_GAIT_WALKING:
            // 前腿已穿过窗口平面，要求保持安全侧
            leg_mode[0] = 1;
            leg_mode[3] = 1;
            break;
        case CrossingStateMachine::CrossingState::REAR_LEGS_TRANSIT:
            // 后腿穿越，front legs 保持不回落到左侧
            leg_mode[1] = 1;
            leg_mode[2] = 1;
            leg_mode[0] = 2;
            leg_mode[3] = 2;
            break;
        default:
            return;  // 其他阶段不启用 rail-window 安全侧约束（避免引入不可行性）
    }

    int n_constraints_added = 0;
    for (int k = 0; k < N; ++k) {
        const int x_idx = k * (nx + nu) + 0;  // x_CoM in interleaved [x_k, u_k] layout

        for (int leg_i = 0; leg_i < 4; ++leg_i) {
            if (leg_mode[leg_i] == 0) {
                continue;
            }

            const int d_idx = k * (nx + nu) + sliding_start_idx + leg_i;  // d_i

            // row 对应到 A*x 的一行不等式约束 [l, u]
            const int row = static_cast<int>(l_vec.size());

            A_triplets.push_back(Eigen::Triplet<double>(row, x_idx, 1.0));
            A_triplets.push_back(Eigen::Triplet<double>(row, d_idx, 1.0));

            // base_foot_positions_ 被当作“相对 CoM 的相对足端向量”
            double base_x = 0.0;
            if (base_foot_positions_.rows() >= 4 && base_foot_positions_.cols() >= 1) {
                base_x = base_foot_positions_(leg_i, 0);
            }

            double l_bound = -BIG;
            double u_bound = BIG;

            if (leg_mode[leg_i] == 1) {
                l_bound = (x_window + d_safe) - base_x;
            } else if (leg_mode[leg_i] == 2) {
                l_bound = (x_window - d_safe) - base_x;
            } else if (leg_mode[leg_i] == 3) {
                u_bound = (x_window - d_safe) - base_x;
            }

            l_vec.push_back(l_bound);
            u_vec.push_back(u_bound);
            n_constraints_added++;
        }
    }

    std::cout << "[MPC] rail+window 约束已添加: " << n_constraints_added << " 个约束" << std::endl;
}

void MPCController::addControlConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                                         std::vector<double>& l_vec,
                                         std::vector<double>& u_vec) {
    const int N = params_.horizon;
    const int nx = 16;
    const int nu = 12;
    
    // 控制约束：u_min <= u_k <= u_max
    for (int k = 0; k < N; ++k) {
        int u_k_offset = k * (nx + nu) + nx;  // u_k的位置
        
        for (int i = 0; i < nu; ++i) {
            int row = l_vec.size();
            
            // u_k[i] 项：系数为 1
            A_triplets.push_back(Eigen::Triplet<double>(row, u_k_offset + i, 1.0));
            
            l_vec.push_back(params_.u_min(i));
            u_vec.push_back(params_.u_max(i));
        }
    }
}
} // namespace dog2_mpc
