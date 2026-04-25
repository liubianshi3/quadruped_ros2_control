#include "dog2_mpc/crossing_state_machine.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>

namespace dog2_mpc {

CrossingStateMachine::CrossingStateMachine()
    : current_state_(CrossingState::APPROACH),
      state_start_time_(0.0),
      total_time_(0.0) {
}

void CrossingStateMachine::initialize(const RobotState& initial_state, const WindowObstacle& window) {
    initial_state_ = initial_state;
    window_ = window;
    current_state_ = CrossingState::APPROACH;
    state_start_time_ = 0.0;
    total_time_ = 0.0;
    transition_stable_elapsed_ = 0.0;
    
    std::cout << "[CrossingStateMachine] 初始化完成" << std::endl;
    std::cout << "  初始位置: x=" << initial_state.position.x() << "m" << std::endl;
    std::cout << "  窗框位置: x=" << window.x_position << "m" << std::endl;
    std::cout << "  距离窗框: " << (window.x_position - initial_state.position.x()) << "m" << std::endl;
}

bool CrossingStateMachine::update(const RobotState& current_state, double dt) {
    total_time_ += dt;
    
    if (current_state_ == CrossingState::COMPLETED) {
        transition_stable_elapsed_ = 0.0;
        return true;
    }

    // stable transition graph：
    // 对当前阶段 guard 连续满足一段稳定保持时间后才允许切换，避免噪声触发抖动。
    const bool guard_ok = canTransitionToNext(current_state);
    if (guard_ok) {
        transition_stable_elapsed_ += dt;
    } else {
        transition_stable_elapsed_ = 0.0;
    }
    
    if (transition_stable_elapsed_ >= transition_stable_time_) {
        transitionToNextState();
        transition_stable_elapsed_ = 0.0;
    }
    
    return true;
}

std::string CrossingStateMachine::getStateName(CrossingState state) {
    switch (state) {
        case CrossingState::APPROACH:           return "初始接近";
        case CrossingState::BODY_FORWARD_SHIFT: return "机身前探";
        case CrossingState::FRONT_LEGS_TRANSIT: return "前腿穿越";
        case CrossingState::HYBRID_GAIT_WALKING: return "混合构型行走";
        case CrossingState::RAIL_ALIGNMENT:     return "精确停车定位";
        case CrossingState::REAR_LEGS_TRANSIT:  return "后腿穿越";
        case CrossingState::ALL_KNEE_STATE:     return "全膝式状态";
        case CrossingState::RECOVERY:           return "恢复常态";
        case CrossingState::CONTINUE_FORWARD:   return "继续前进";
        case CrossingState::COMPLETED:          return "越障完成";
        default:                                return "未知状态";
    }
}

CrossingStateMachine::RobotState CrossingStateMachine::getTargetState() const {
    switch (current_state_) {
        case CrossingState::APPROACH:           return computeApproachTarget();
        case CrossingState::BODY_FORWARD_SHIFT: return computeBodyForwardShiftTarget();
        case CrossingState::FRONT_LEGS_TRANSIT: return computeFrontLegsTransitTarget();
        case CrossingState::HYBRID_GAIT_WALKING: return computeHybridGaitWalkingTarget();
        case CrossingState::RAIL_ALIGNMENT:     return computeRailAlignmentTarget();
        case CrossingState::REAR_LEGS_TRANSIT:  return computeRearLegsTransitTarget();
        case CrossingState::ALL_KNEE_STATE:     return computeAllKneeStateTarget();
        case CrossingState::RECOVERY:           return computeRecoveryTarget();
        case CrossingState::CONTINUE_FORWARD:   return computeContinueForwardTarget();
        default:                                return initial_state_;
    }
}

CrossingStateMachine::StageConstraints CrossingStateMachine::getCurrentConstraints() const {
    StageConstraints constraints;
    
    // Canonical Dog2 rail order and limits:
    // [lf_rail_joint, lh_rail_joint, rh_rail_joint, rf_rail_joint].
    // These values are derived from dog2.urdf.xacro / urdf_joint_limits.py;
    // do not collapse them into a single signed constant because mirrored
    // legs have different valid q ranges.
    constraints.sliding_min << 0.0, -0.111, 0.0, -0.111;
    constraints.sliding_max << 0.111, 0.0, 0.111, 0.0;
    constraints.sliding_vel_max << 1.0, 1.0, 1.0, 1.0;  // 1.0 m/s
    
    // 根据当前状态调整约束
    switch (current_state_) {
        case CrossingState::APPROACH:
            // 正常接近阶段只通过目标函数偏向 neutral，不再硬锁到 0。
            // 这样 measured rail 若已在物理限位附近，不会把 QP 直接推成不可行。
            break;
            
        case CrossingState::BODY_FORWARD_SHIFT:
            // Rail compact-body posture inside the real per-joint limits:
            // lf/rh move toward +q, lh/rf move toward -q.
            constraints.sliding_min << 0.100, -0.111, 0.100, -0.111;
            constraints.sliding_max << 0.111, -0.100, 0.111, -0.100;
            break;
            
        case CrossingState::HYBRID_GAIT_WALKING:
            // 混合构型，需要特殊的工作空间约束
            constraints.min_leg_distance = 0.15;  // 腿间最小距离
            
            // 前腿（膝式）工作空间偏后
            for (int i : {0, 3}) {  // leg1, leg4
                constraints.foot_workspace_min[i] = Eigen::Vector3d(-0.15, -0.15, 0.0);
                constraints.foot_workspace_max[i] = Eigen::Vector3d(-0.05, 0.15, 0.12);
            }
            
            // 后腿（肘式）工作空间偏前
            for (int i : {1, 2}) {  // leg2, leg3
                constraints.foot_workspace_min[i] = Eigen::Vector3d(-0.05, -0.15, 0.0);
                constraints.foot_workspace_max[i] = Eigen::Vector3d(0.15, 0.15, 0.12);
            }
            break;
            
        default:
            // 使用默认约束
            break;
    }
    
    // 质心约束
    constraints.com_min = Eigen::Vector3d(-10.0, -1.0, 0.25);
    constraints.com_max = Eigen::Vector3d(10.0, 1.0, 0.35);
    
    return constraints;
}

bool CrossingStateMachine::canTransitionToNext(const RobotState& current_state) const {
    bool progress_ok = false;
    switch (current_state_) {
        case CrossingState::APPROACH:
            progress_ok = checkApproachComplete(current_state);
            break;
        case CrossingState::BODY_FORWARD_SHIFT:
            progress_ok = checkBodyForwardShiftComplete(current_state);
            break;
        case CrossingState::FRONT_LEGS_TRANSIT:
            progress_ok = checkFrontLegsTransitComplete(current_state);
            break;
        case CrossingState::HYBRID_GAIT_WALKING:
            progress_ok = checkHybridGaitWalkingComplete(current_state);
            break;
        case CrossingState::RAIL_ALIGNMENT:
            progress_ok = checkRailAlignmentComplete(current_state);
            break;
        case CrossingState::REAR_LEGS_TRANSIT:
            progress_ok = checkRearLegsTransitComplete(current_state);
            break;
        case CrossingState::ALL_KNEE_STATE:
            progress_ok = checkAllKneeStateComplete(current_state);
            break;
        case CrossingState::RECOVERY:
            progress_ok = checkRecoveryComplete(current_state);
            break;
        case CrossingState::CONTINUE_FORWARD:
            return true;  // 最后阶段，直接完成
        default:
            return false;
    }

    if (!progress_ok) {
        return false;
    }

    // 额外 guard：rail tracking + support polygon，避免阶段几何时序错配
    const double rail_tracking_error = computeRailTrackingError(current_state);
    const bool railStable =
        current_state_ == CrossingState::APPROACH ||
        rail_tracking_error < rail_tracking_error_threshold_;

    int contact_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (current_state.foot_contacts[i]) {
            contact_count++;
        }
    }

    const double support_margin = computeSupportPolygonMargin(current_state);
    bool supportStable = false;

    // 支撑域退化（对角线 Trot：仅2足接触）时，使用 Capture Point (CP) 动态稳定裕度
    // 以 CP 到两足连线线段的垂直距离 d_cp 判断是否允许 transition。
    if (contact_count == 2) {
        const double d_cp = support_margin;  // computeSupportPolygonMargin 在此退化分支返回 d_cp
        supportStable = (d_cp < 0.025);
    } else if (contact_count >= 3) {
        // 多足支撑：使用几何支撑域裕度 margin
        supportStable = (support_margin > support_polygon_margin_threshold_);
    } else {
        // 不足两点接触时无法稳定支撑
        supportStable = false;
    }

    return railStable && supportStable;
}

double CrossingStateMachine::computeRailTrackingError(const RobotState& state) const {
    const RobotState target = getTargetState();
    // 最大逐腿位移偏差（单位：m）
    return (state.sliding_positions - target.sliding_positions).cwiseAbs().maxCoeff();
}

double CrossingStateMachine::computeSupportPolygonMargin(const RobotState& state) const {
    // 2D support margin:
    // - 若足端接触数>=3：对接触点在 (x,y) 平面的凸包，计算 COM 到每条凸包边界的“有符号距离”
    //   (CCW 凸包内侧为正，凸包外侧为负)，取最小值作为 margin。
    // - 若足端接触数==2：支撑域退化为线段，margin 为 COM 到线段的垂距（投影落在线段内）；
    //   否则返回负值以强制 guard 失效。

    std::vector<Eigen::Vector2d> contact_points;
    contact_points.reserve(4);

    for (int i = 0; i < 4; ++i) {
        if (!state.foot_contacts[i]) {
            continue;
        }
        contact_points.emplace_back(
            state.foot_positions[i].x(),
            state.foot_positions[i].y());
    }

    if (contact_points.size() < 2) {
        return -std::numeric_limits<double>::infinity();
    }

    // 去重（避免凸包算法数值抖动）
    std::sort(contact_points.begin(), contact_points.end(),
              [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                  if (a.x() != b.x()) return a.x() < b.x();
                  return a.y() < b.y();
              });
    contact_points.erase(
        std::unique(contact_points.begin(), contact_points.end(),
                    [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                        return (a - b).norm() < 1e-9;
                    }),
        contact_points.end());

    const Eigen::Vector2d com(state.position.x(), state.position.y());

    auto cross_z = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() * b.y() - a.y() * b.x();
    };

    auto convexHull = [&](const std::vector<Eigen::Vector2d>& pts) {
        // Monotonic chain; 返回 CCW 顺序且不重复首尾点
        if (pts.size() <= 2) {
            return pts;
        }

        std::vector<Eigen::Vector2d> H;
        H.reserve(pts.size() * 2);

        auto cross_3 = [&](const Eigen::Vector2d& O,
                           const Eigen::Vector2d& A,
                           const Eigen::Vector2d& B) {
            return cross_z(A - O, B - O);
        };

        // lower hull
        for (const auto& p : pts) {
            while (H.size() >= 2 && cross_3(H[H.size() - 2], H.back(), p) <= 0.0) {
                H.pop_back();
            }
            H.push_back(p);
        }

        // upper hull
        const size_t lower_size = H.size();
        for (int i = static_cast<int>(pts.size()) - 2; i >= 0; --i) {
            const auto& p = pts[i];
            while (H.size() > lower_size &&
                   cross_3(H[H.size() - 2], H.back(), p) <= 0.0) {
                H.pop_back();
            }
            H.push_back(p);
        }

        // H 首尾会重复到同一点，移除最后一个
        if (!H.empty()) {
            H.pop_back();
        }
        return H;
    };

    std::vector<Eigen::Vector2d> hull = convexHull(contact_points);

    // 退化情况：线段
    if (hull.size() == 2) {
        const Eigen::Vector2d a = hull[0];
        const Eigen::Vector2d b = hull[1];
        const Eigen::Vector2d ab = b - a;
        const double ab2 = ab.squaredNorm();
        if (ab2 < 1e-12) {
            return -std::numeric_limits<double>::infinity();
        }

        // Capture Point: CP = [x_com + sqrt(h/g)*x_dot, y_com + sqrt(h/g)*y_dot]
        const double g = 9.81;
        const double h = std::max(1e-6, state.position.z());
        const double k = std::sqrt(h / g);
        const Eigen::Vector2d cp(
            state.position.x() + k * state.velocity.x(),
            state.position.y() + k * state.velocity.y());

        // distance from CP to segment [a,b]
        const double t = (cp - a).dot(ab) / ab2;
        const double t_clamped = std::max(0.0, std::min(1.0, t));
        const Eigen::Vector2d closest = a + t_clamped * ab;
        const double d_cp = (cp - closest).norm();

        return d_cp;
    }

    if (hull.size() < 3) {
        return -std::numeric_limits<double>::infinity();
    }

    // 计算凸包朝向（用于符号统一）
    double area2 = 0.0;
    for (size_t i = 0; i < hull.size(); ++i) {
        const auto& p = hull[i];
        const auto& q = hull[(i + 1) % hull.size()];
        area2 += p.x() * q.y() - p.y() * q.x();
    }
    const double orientation_sign = (area2 >= 0.0) ? 1.0 : -1.0;  // CCW => +1

    // 有符号距离：对每条边，使用 cross_z(e, com-a)/|e|
    double margin = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < hull.size(); ++i) {
        const Eigen::Vector2d p = hull[i];
        const Eigen::Vector2d q = hull[(i + 1) % hull.size()];
        const Eigen::Vector2d e = q - p;
        const double elen = e.norm();
        if (elen < 1e-12) {
            continue;
        }

        const double signed_dist = orientation_sign * (cross_z(e, com - p) / elen);
        margin = std::min(margin, signed_dist);
    }

    return margin;
}

void CrossingStateMachine::transitionToNextState() {
    CrossingState next_state;
    
    switch (current_state_) {
        case CrossingState::APPROACH:           next_state = CrossingState::BODY_FORWARD_SHIFT; break;
        case CrossingState::BODY_FORWARD_SHIFT: next_state = CrossingState::FRONT_LEGS_TRANSIT; break;
        case CrossingState::FRONT_LEGS_TRANSIT: next_state = CrossingState::HYBRID_GAIT_WALKING; break;
        case CrossingState::HYBRID_GAIT_WALKING: next_state = CrossingState::RAIL_ALIGNMENT; break;
        case CrossingState::RAIL_ALIGNMENT:     next_state = CrossingState::REAR_LEGS_TRANSIT; break;
        case CrossingState::REAR_LEGS_TRANSIT:  next_state = CrossingState::ALL_KNEE_STATE; break;
        case CrossingState::ALL_KNEE_STATE:     next_state = CrossingState::RECOVERY; break;
        case CrossingState::RECOVERY:           next_state = CrossingState::CONTINUE_FORWARD; break;
        case CrossingState::CONTINUE_FORWARD:   next_state = CrossingState::COMPLETED; break;
        default:                                next_state = CrossingState::COMPLETED; break;
    }
    
    std::cout << "[CrossingStateMachine] 状态转换: " 
              << getStateName(current_state_) << " -> " << getStateName(next_state) << std::endl;
    
    current_state_ = next_state;
    state_start_time_ = total_time_;
    transition_stable_elapsed_ = 0.0;
}

double CrossingStateMachine::getProgress() const {
    // 进度定义为 0~1，对应 9 个阶段 (APPROACH..CONTINUE_FORWARD)；
    // COMPLETED 为终态，进度固定为 1.0。
    if (current_state_ == CrossingState::COMPLETED) {
        return 1.0;
    }

    const int last_stage_index = static_cast<int>(CrossingState::CONTINUE_FORWARD);  // 8
    int current_index = static_cast<int>(current_state_);
    current_index = std::max(0, std::min(current_index, last_stage_index));
    return static_cast<double>(current_index) / static_cast<double>(last_stage_index == 0 ? 1 : last_stage_index);
}

// 各阶段完成条件检查
bool CrossingStateMachine::checkApproachComplete(const RobotState& state) const {
    // 接近完成条件：到达窗框前约 0.2m，并处于低速可切换状态。
    // 不要求完全静止，否则 Gazebo 中低速通过窗口前沿时很难稳定进入
    // rail compact-body 阶段。
    double distance_to_window = window_.x_position - state.position.x();
    bool position_ok = distance_to_window <= 0.2;
    bool velocity_ok = std::abs(state.velocity.x()) < 0.45;
    
    return position_ok && velocity_ok;
}

bool CrossingStateMachine::checkBodyForwardShiftComplete(const RobotState& state) const {
    // 机身前探完成条件：四个 rail 到达真实限位内的 compact-body 姿态。
    bool diagonal_positive_extended = (state.sliding_positions[0] >= 0.10) &&
                                      (state.sliding_positions[2] >= 0.10);
    bool diagonal_negative_extended = (state.sliding_positions[1] <= -0.10) &&
                                      (state.sliding_positions[3] <= -0.10);
    
    return diagonal_positive_extended && diagonal_negative_extended;
}

bool CrossingStateMachine::checkFrontLegsTransitComplete(const RobotState& state) const {
    // 前腿穿越完成条件：前腿构型为膝式，且足端在窗框后方
    bool front_legs_knee = (state.leg_configs[0] == LegConfiguration::KNEE) &&
                          (state.leg_configs[3] == LegConfiguration::KNEE);
    
    bool front_feet_through = (state.foot_positions[0].x() > window_.x_position) &&
                             (state.foot_positions[3].x() > window_.x_position);
    
    return front_legs_knee && front_feet_through;
}

bool CrossingStateMachine::checkHybridGaitWalkingComplete(const RobotState& state) const {
    // 混合构型行走完成条件：后腿导轨前端穿过窗框
    double rear_rail_front = state.position.x() + state.sliding_positions[1];  // 后腿导轨前端
    bool rear_rails_through = rear_rail_front > window_.x_position;
    
    return rear_rails_through;
}

bool CrossingStateMachine::checkRailAlignmentComplete(const RobotState& state) const {
    // 精确停车完成条件：机身停止，位置精确
    bool velocity_zero = state.velocity.norm() < 0.01;
    bool position_aligned = std::abs(state.position.x() - (window_.x_position + 0.15)) < 0.02;
    
    return velocity_zero && position_aligned;
}

bool CrossingStateMachine::checkRearLegsTransitComplete(const RobotState& state) const {
    // 后腿穿越完成条件：后腿构型为膝式，且足端在窗框后方
    bool rear_legs_knee = (state.leg_configs[1] == LegConfiguration::KNEE) &&
                         (state.leg_configs[2] == LegConfiguration::KNEE);
    
    bool rear_feet_through = (state.foot_positions[1].x() > window_.x_position) &&
                            (state.foot_positions[2].x() > window_.x_position);
    
    return rear_legs_knee && rear_feet_through;
}

bool CrossingStateMachine::checkAllKneeStateComplete(const RobotState& state) const {
    // 全膝式状态完成条件：所有腿都是膝式，机身稳定
    bool all_knee = true;
    for (int i = 0; i < 4; ++i) {
        if (state.leg_configs[i] != LegConfiguration::KNEE) {
            all_knee = false;
            break;
        }
    }
    
    bool stable = state.velocity.norm() < 0.05;
    
    return all_knee && stable;
}

bool CrossingStateMachine::checkRecoveryComplete(const RobotState& state) const {
    // 恢复完成条件：所有腿恢复肘式，滑动副回到中立位置
    bool all_elbow = true;
    for (int i = 0; i < 4; ++i) {
        if (state.leg_configs[i] != LegConfiguration::ELBOW) {
            all_elbow = false;
            break;
        }
    }
    
    bool slides_neutral = state.sliding_positions.norm() < 0.02;
    
    return all_elbow && slides_neutral;
}

// 各阶段目标状态计算
CrossingStateMachine::RobotState CrossingStateMachine::computeApproachTarget() const {
    RobotState target = initial_state_;
    
    // 目标：接近窗框到0.2m距离
    target.position.x() = window_.x_position - 0.2;
    target.velocity.setZero();
    target.sliding_positions.setZero();
    
    // 所有腿保持肘式
    for (int i = 0; i < 4; ++i) {
        target.leg_configs[i] = LegConfiguration::ELBOW;
        target.foot_contacts[i] = true;
    }
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeBodyForwardShiftTarget() const {
    RobotState target = initial_state_;
    
    // 目标：机身前移0.111m
    target.position.x() = window_.x_position - 0.2 + 0.111;
    target.velocity.setZero();
    
    // Rail compact-body target in real joint order:
    // lf_rail_joint -> +limit, lh_rail_joint -> -limit,
    // rh_rail_joint -> +limit, rf_rail_joint -> -limit.
    target.sliding_positions << 0.111, -0.111, 0.111, -0.111;
    
    // 所有腿保持肘式，接触地面
    for (int i = 0; i < 4; ++i) {
        target.leg_configs[i] = LegConfiguration::ELBOW;
        target.foot_contacts[i] = true;
    }
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeFrontLegsTransitTarget() const {
    RobotState target = computeBodyForwardShiftTarget();
    
    // 前腿切换为膝式
    target.leg_configs[0] = LegConfiguration::KNEE;  // leg1
    target.leg_configs[3] = LegConfiguration::KNEE;  // leg4
    
    // 前腿足端穿过窗框
    target.foot_positions[0].x() = window_.x_position + 0.1;
    target.foot_positions[3].x() = window_.x_position + 0.1;
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeHybridGaitWalkingTarget() const {
    RobotState target = computeFrontLegsTransitTarget();
    
    // 目标：机身继续前移，使后腿导轨前端穿过窗框
    target.position.x() = window_.x_position + 0.15;
    target.velocity.x() = 0.1;  // 慢速前进
    
    // 混合构型：前腿膝式，后腿肘式
    target.leg_configs[0] = LegConfiguration::KNEE;   // leg1
    target.leg_configs[1] = LegConfiguration::ELBOW;  // leg2
    target.leg_configs[2] = LegConfiguration::ELBOW;  // leg3
    target.leg_configs[3] = LegConfiguration::KNEE;   // leg4
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeRailAlignmentTarget() const {
    RobotState target = computeHybridGaitWalkingTarget();
    
    // 目标：精确停车
    target.position.x() = window_.x_position + 0.15;
    target.velocity.setZero();
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeRearLegsTransitTarget() const {
    RobotState target = computeRailAlignmentTarget();
    
    // 后腿切换为膝式
    target.leg_configs[1] = LegConfiguration::KNEE;  // leg2
    target.leg_configs[2] = LegConfiguration::KNEE;  // leg3
    
    // 后腿足端穿过窗框
    target.foot_positions[1].x() = window_.x_position + 0.1;
    target.foot_positions[2].x() = window_.x_position + 0.1;
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeAllKneeStateTarget() const {
    RobotState target = computeRearLegsTransitTarget();
    
    // 所有腿都是膝式
    for (int i = 0; i < 4; ++i) {
        target.leg_configs[i] = LegConfiguration::KNEE;
    }
    
    target.velocity.setZero();
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeRecoveryTarget() const {
    RobotState target = computeAllKneeStateTarget();
    
    // 恢复到肘式
    for (int i = 0; i < 4; ++i) {
        target.leg_configs[i] = LegConfiguration::ELBOW;
    }
    
    // 滑动副回到中立位置
    target.sliding_positions.setZero();
    
    return target;
}

CrossingStateMachine::RobotState CrossingStateMachine::computeContinueForwardTarget() const {
    RobotState target = computeRecoveryTarget();
    
    // 继续前进
    target.position.x() = window_.x_position + 1.0;
    target.velocity.x() = 0.2;
    
    return target;
}

void CrossingStateMachine::forceTransitionTo(CrossingState state) {
    std::cout << "[CrossingStateMachine] 强制转换到状态: " << getStateName(state) << std::endl;
    current_state_ = state;
    state_start_time_ = total_time_;
    transition_stable_elapsed_ = 0.0;
}

} // namespace dog2_mpc
