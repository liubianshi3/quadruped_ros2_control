#ifndef DOG2_MPC_MPC_CONTROLLER_HPP
#define DOG2_MPC_MPC_CONTROLLER_HPP

#include "dog2_mpc/extended_srbd_model.hpp"
#include "dog2_mpc/sliding_constraints.hpp"
#include "dog2_mpc/boundary_constraints.hpp"
#include "dog2_mpc/osqp_interface.hpp"
#include "dog2_mpc/crossing_state_machine.hpp"
#include "dog2_mpc/hybrid_gait_generator.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>

namespace dog2_mpc {

/**
 * @brief MPC控制器主类
 * 
 * 整合SRBD模型、约束和OSQP求解器，实现滚动时域优化
 * 
 * 优化问题:
 *   min  Σ ||x[k] - x_ref[k]||²_Q + ||u[k]||²_R
 *   s.t. x[k+1] = A*x[k] + B*u[k]
 *        滑动副约束
 *        窗框边界约束
 *        控制输入约束
 */
class MPCController {
public:
    /**
     * @brief MPC参数结构
     */
    struct Parameters {
        int horizon;                    ///< 预测时域
        double dt;                      ///< 时间步长 (s)
        Eigen::MatrixXd Q;              ///< 状态权重矩阵 (16×16) - 扩展到包含滑动副
        Eigen::MatrixXd R;              ///< 控制权重矩阵 (12×12)
        Eigen::VectorXd u_min;          ///< 控制下界 (12维)
        Eigen::VectorXd u_max;          ///< 控制上界 (12维)
        bool enable_sliding_constraints;     ///< 启用滑动副约束
        bool enable_boundary_constraints;    ///< 启用窗框约束
        
        Parameters()
            : horizon(20),
              dt(0.05),
              Q(Eigen::MatrixXd::Identity(16, 16)),  // 扩展到16维
              R(Eigen::MatrixXd::Identity(12, 12) * 0.01),
              u_min(Eigen::VectorXd::Constant(12, -100.0)),
              u_max(Eigen::VectorXd::Constant(12, 100.0)),
              enable_sliding_constraints(true),
              enable_boundary_constraints(true) {
            // 默认权重 - 16维状态
            Q.diagonal() << 100, 100, 100,  // 位置 [x, y, z]
                           10, 10, 10,       // 姿态 [roll, pitch, yaw]
                           10, 10, 10,       // 线速度 [vx, vy, vz]
                           1, 1, 1,          // 角速度 [wx, wy, wz]
                           50, 50, 50, 50;   // 滑动副位置 [j1, j2, j3, j4]
        }
    };
    
    /**
     * @brief 构造函数
     * @param mass 机器人质量 (kg)
     * @param inertia 惯性张量 (3×3)
     * @param params MPC参数
     */
    MPCController(double mass,
                  const Eigen::Matrix3d& inertia,
                  const Parameters& params = Parameters());
    
    /**
     * @brief 设置参考轨迹
     * @param x_ref 参考状态序列 (horizon × 16) - 扩展状态
     */
    void setReference(const std::vector<Eigen::VectorXd>& x_ref);
    
    /**
     * @brief 设置基础足端位置（滑动副为0时的位置）
     * @param foot_positions 基础足端位置 (4×3矩阵)
     */
    void setBaseFootPositions(const Eigen::MatrixXd& foot_positions);
    
    /**
     * @brief 设置滑动副速度
     * @param sliding_velocity 滑动副速度 [v1, v2, v3, v4] (m/s)
     */
    void setSlidingVelocity(const Eigen::Vector4d& sliding_velocity);

    /**
     * @brief 设置越障状态机 guard 参数（用于 stable transition / rail tracking / support polygon）
     * @param rail_tracking_error_threshold rail tracking guard 阈值（单位：m）
     * @param support_polygon_margin_threshold 支撑多边形 margin 阈值（单位：m）
     * @param transition_stable_time stage transition 稳定保持时长（单位：s）
     */
    void setCrossingGuardParams(double rail_tracking_error_threshold,
                                 double support_polygon_margin_threshold,
                                 double transition_stable_time);

    /**
     * @brief 设置 rail soft bound 的 exact penalty 一次项权重
     * @param slack_linear_weight 松弛变量惩罚权重（用于 q 向量）
     */
    void setSlackLinearWeight(double slack_linear_weight);

    /**
     * @brief 冻结 crossing 期间的 rail 参考和 rail-stage 约束（诊断用）
     */
    void setFreezeCrossingRailTargets(bool freeze) {
        freeze_crossing_rail_targets_ = freeze;
    }
    
    /**
     * @brief 求解MPC优化问题（16维扩展版本）
     * @param x0 当前扩展状态 (16维: SRBD + 滑动副)
     * @param u_optimal 输出：最优控制序列第一步 (12维)
     * @return true 求解成功
     * @return false 求解失败
     */
    bool solve(const Eigen::VectorXd& x0, Eigen::VectorXd& u_optimal);
    
    /**
     * @brief 求解MPC优化问题（兼容12维SRBD状态）
     * @param srbd_state 当前SRBD状态 (12维)
     * @param sliding_positions 当前滑动副位置 (4维)
     * @param u_optimal 输出：最优控制序列第一步 (12维)
     * @return true 求解成功
     * @return false 求解失败
     */
    bool solve(const Eigen::VectorXd& srbd_state, 
               const Eigen::Vector4d& sliding_positions,
               Eigen::VectorXd& u_optimal);
    
    /**
     * @brief 获取预测轨迹（16维扩展状态）
     * @return 预测状态序列 (horizon × 16)
     */
    std::vector<Eigen::VectorXd> getPredictedTrajectory() const;
    
    /**
     * @brief 获取预测控制序列
     * @return 预测控制序列 (horizon × 12)
     */
    std::vector<Eigen::VectorXd> getPredictedControl() const;
    
    /**
     * @brief 获取求解时间
     * @return 求解时间 (ms)
     */
    double getSolveTime() const;
    
    /**
     * @brief 获取求解状态
     * @return OSQP状态码
     */
    int getSolveStatus() const;
    
    /**
     * @brief 更新MPC参数
     * @param params 新参数
     */
    void updateParameters(const Parameters& params);
    
    /**
     * @brief 获取滑动副约束对象
     */
    SlidingConstraints& getSlidingConstraints() { return sliding_constraints_; }
    
    /**
     * @brief 获取窗框约束对象
     */
    BoundaryConstraints& getBoundaryConstraints() { return boundary_constraints_; }
    
    /**
     * @brief 初始化越障功能
     * @param initial_state 初始机器人状态
     * @param window 窗框参数
     */
    void initializeCrossing(const CrossingStateMachine::RobotState& initial_state,
                           const CrossingStateMachine::WindowObstacle& window);
    
    /**
     * @brief 更新越障状态机
     * @param current_state 当前机器人状态
     * @param dt 时间步长
     * @return true 更新成功
     */
    bool updateCrossing(const CrossingStateMachine::RobotState& current_state, double dt);
    
    /**
     * @brief 获取当前越障状态
     */
    CrossingStateMachine::CrossingState getCurrentCrossingState() const;
    
    /**
     * @brief 获取越障进度 (0.0 ~ 1.0)
     */
    double getCrossingProgress() const;
    
    /**
     * @brief 生成越障参考轨迹
     * @param current_state 当前机器人状态
     * @param dt 时间步长
     * @return 参考轨迹
     */
    std::vector<Eigen::VectorXd> generateCrossingReference(
        const CrossingStateMachine::RobotState& current_state, double dt);
    
    /**
     * @brief 是否启用越障模式
     */
    bool isCrossingEnabled() const { return crossing_enabled_; }

private:
    /**
     * @brief 构建QP问题（16维扩展版本）
     * @param x0 初始扩展状态 (16维)
     * @param P Hessian矩阵 (输出)
     * @param q 线性项 (输出)
     * @param A 约束矩阵 (输出)
     * @param l 约束下界 (输出)
     * @param u 约束上界 (输出)
     */
    void buildQP(const Eigen::VectorXd& x0,
                 Eigen::SparseMatrix<double>& P,
                 Eigen::VectorXd& q,
                 Eigen::SparseMatrix<double>& A,
                 Eigen::VectorXd& l,
                 Eigen::VectorXd& u);
    
    /**
     * @brief 添加动力学约束（16维扩展）
     * @param x0 初始状态
     * @param A_triplets 约束矩阵三元组 (输出)
     * @param l_vec 约束下界向量 (输出)
     * @param u_vec 约束上界向量 (输出)
     */
    void addDynamicsConstraints(const Eigen::VectorXd& x0,
                               std::vector<Eigen::Triplet<double>>& A_triplets,
                               std::vector<double>& l_vec,
                               std::vector<double>& u_vec);
    
    /**
     * @brief 添加滑动副约束
     * @param A_triplets 约束矩阵三元组 (输出)
     * @param l_vec 约束下界向量 (输出)
     * @param u_vec 约束上界向量 (输出)
     */
    void addSlidingConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                              std::vector<double>& l_vec,
                              std::vector<double>& u_vec);
    
    /**
     * @brief 添加控制约束
     * @param A_triplets 约束矩阵三元组 (输出)
     * @param l_vec 约束下界向量 (输出)
     * @param u_vec 约束上界向量 (输出)
     */
    void addControlConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                              std::vector<double>& l_vec,
                              std::vector<double>& u_vec);
    
    /**
     * @brief 添加 rail+window 线性化碰撞约束（16D：x_CoM + d_i 的不等式）
     * @details 在越障跨越阶段，对部分腿的“世界系足端x/rail前端”施加窗框安全侧约束，避免穿越过程发生结构性碰撞。
     */
    void addRailWindowConstraints(std::vector<Eigen::Triplet<double>>& A_triplets,
                                  std::vector<double>& l_vec,
                                  std::vector<double>& u_vec);
    
    /**
     * @brief 线性化整个时域（16维扩展）
     * @param x0 初始扩展状态 (16维)
     */
    void linearizeHorizon(const Eigen::VectorXd& x0);
    
    /**
     * @brief 从优化变量中提取状态和控制（16维扩展）
     * @param solution 优化变量 (horizon*(16+12) 维)
     */
    void extractSolution(const Eigen::VectorXd& solution);
    
    /**
     * @brief 更新越障状态机（内部方法）
     * @param x0 当前状态
     */
    void updateCrossingState(const Eigen::VectorXd& x0);
    
    // 模型和约束
    std::unique_ptr<ExtendedSRBDModel> extended_srbd_model_;
    SlidingConstraints sliding_constraints_;
    BoundaryConstraints boundary_constraints_;
    std::unique_ptr<OSQPInterface> osqp_solver_;
    
    // 越障功能
    std::unique_ptr<CrossingStateMachine> crossing_state_machine_;
    std::unique_ptr<HybridGaitGenerator> hybrid_gait_generator_;
    bool crossing_enabled_;
    
    // MPC参数
    Parameters params_;

    // rail soft bound exact penalty 线性项权重
    double current_slack_weight_;

    // crossing 诊断：冻结 rail 目标和 rail-stage 约束
    bool freeze_crossing_rail_targets_;
    
    // 参考轨迹（16维扩展状态）
    std::vector<Eigen::VectorXd> x_ref_;
    
    // 基础足端位置和滑动副速度
    Eigen::MatrixXd base_foot_positions_;
    Eigen::Vector4d sliding_velocity_;
    
    // 线性化结果（16维扩展）
    std::vector<Eigen::MatrixXd> A_matrices_;  ///< 状态转移矩阵序列 (16×16)
    std::vector<Eigen::MatrixXd> B_matrices_;  ///< 控制输入矩阵序列 (16×12)
    std::vector<Eigen::MatrixXd> C_matrices_;  ///< 滑动副速度矩阵序列 (16×4)
    
    // 求解结果（16维扩展状态）
    std::vector<Eigen::VectorXd> x_predicted_;  ///< 预测状态 (16维)
    std::vector<Eigen::VectorXd> u_predicted_;  ///< 预测控制 (12维)
    
    // 求解统计
    double last_solve_time_;
    int last_solve_status_;
    
    // 是否已初始化
    bool initialized_;
};

} // namespace dog2_mpc

#endif // DOG2_MPC_MPC_CONTROLLER_HPP
