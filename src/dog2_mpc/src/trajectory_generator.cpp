#include "dog2_mpc/trajectory_generator.hpp"
#include <cmath>
#include <iostream>

namespace dog2_mpc {

TrajectoryGenerator::TrajectoryGenerator(const Parameters& params)
    : params_(params), current_mode_(Mode::HOVER) {
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::generateHoverTrajectory(
    const Eigen::VectorXd& current_state,
    int horizon,
    double dt) {
    
    std::vector<Eigen::VectorXd> trajectory(horizon);
    
    // 目标：保持当前位置和姿态，速度为0
    Eigen::VectorXd target_state = Eigen::VectorXd::Zero(16);
    
    // 保持当前位置
    target_state.segment<3>(0) = current_state.segment<3>(0);
    
    // 保持期望高度
    target_state(2) = params_.default_height;
    
    // 保持当前yaw，roll和pitch为0
    target_state(5) = current_state(5);  // yaw
    
    // 速度和角速度为0
    // 滑动副位置保持当前值
    target_state.segment<4>(12) = current_state.segment<4>(12);
    
    // 所有时间步使用相同的目标
    for (int k = 0; k < horizon; ++k) {
        trajectory[k] = target_state;
    }
    
    return trajectory;
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::generateWalkingTrajectory(
    const Eigen::VectorXd& current_state,
    const Eigen::Vector3d& velocity_cmd,
    const HybridGaitGenerator::GaitState& gait_state,
    int horizon,
    double dt) {
    
    std::vector<Eigen::VectorXd> trajectory(horizon);
    
    // 当前状态
    Eigen::Vector3d current_pos = current_state.segment<3>(0);
    Eigen::Vector3d current_ori = current_state.segment<3>(3);
    double current_yaw = current_ori(2);
    
    // 速度命令
    double vx_cmd = velocity_cmd(0);
    double vy_cmd = velocity_cmd(1);
    double vyaw_cmd = velocity_cmd(2);
    
    // 生成轨迹
    for (int k = 0; k < horizon; ++k) {
        Eigen::VectorXd ref_state = Eigen::VectorXd::Zero(16);
        
        double t = (k + 1) * dt;
        
        // 位置：根据速度命令积分
        double yaw_future = current_yaw + vyaw_cmd * t;
        ref_state(0) = current_pos(0) + vx_cmd * std::cos(yaw_future) * t;
        ref_state(1) = current_pos(1) + vx_cmd * std::sin(yaw_future) * t;
        ref_state(2) = params_.default_height;  // 保持高度
        
        // 姿态：roll和pitch为0，yaw根据命令变化
        ref_state(3) = 0.0;  // roll
        ref_state(4) = 0.0;  // pitch
        ref_state(5) = yaw_future;  // yaw
        
        // 速度：保持命令速度
        ref_state(6) = vx_cmd * std::cos(yaw_future);  // vx (world frame)
        ref_state(7) = vx_cmd * std::sin(yaw_future);  // vy (world frame)
        ref_state(8) = 0.0;  // vz
        
        // 角速度
        ref_state(9) = 0.0;   // wx
        ref_state(10) = 0.0;  // wy
        ref_state(11) = vyaw_cmd;  // wz
        
        // 阶段 3 简化 walking 中，rail_joint 默认保持当前锁定姿态，
        // 不把 rail 参考强行拉回 0，避免 rail 约束主导主控制链路。
        ref_state.segment<4>(12) = current_state.segment<4>(12);
        
        trajectory[k] = ref_state;
    }
    
    // 平滑轨迹
    trajectory = smoothTrajectory(trajectory, params_.trajectory_smoothness);
    
    return trajectory;
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::generateCrossingTrajectory(
    const Eigen::VectorXd& current_state,
    const CrossingStateMachine::CrossingState& crossing_state,
    const CrossingStateMachine::WindowObstacle& window,
    int horizon,
    double dt) {
    
    std::vector<Eigen::VectorXd> trajectory(horizon);
    
    // 当前状态
    Eigen::Vector3d current_pos = current_state.segment<3>(0);
    Eigen::Vector4d current_sliding = current_state.segment<4>(12);
    
    // 根据越障阶段生成不同的轨迹
    switch (crossing_state) {
        case CrossingStateMachine::CrossingState::APPROACH: {
            // 接近阶段：正常行走接近窗框
            Eigen::Vector3d velocity_cmd(params_.crossing_speed, 0.0, 0.0);
            HybridGaitGenerator::GaitState dummy_gait;
            trajectory = generateWalkingTrajectory(current_state, velocity_cmd, 
                                                  dummy_gait, horizon, dt);
            break;
        }
        
        case CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT: {
            // 机身前探：滑动副伸展，身体前移
            for (int k = 0; k < horizon; ++k) {
                Eigen::VectorXd ref_state = current_state;
                
                double progress = (k + 1.0) / horizon;
                
                // 位置：略微前移
                ref_state(0) = current_pos(0) + 0.05 * progress;
                ref_state(2) = params_.default_height;
                
                // 速度为0（静态）
                ref_state.segment<3>(6).setZero();
                ref_state.segment<3>(9).setZero();
                
                // 滑动副：逐渐伸展
                Eigen::Vector4d target_sliding;
                target_sliding << 0.08, 0.08, 0.08, 0.08;  // 伸展8cm
                ref_state.segment<4>(12) = current_sliding + 
                    (target_sliding - current_sliding) * progress;
                
                trajectory[k] = ref_state;
            }
            break;
        }
        
        case CrossingStateMachine::CrossingState::FRONT_LEGS_TRANSIT: {
            // 前腿穿越：保持位置，准备穿越
            for (int k = 0; k < horizon; ++k) {
                trajectory[k] = current_state;
                trajectory[k](2) = params_.default_height;
                trajectory[k].segment<3>(6).setZero();
            }
            break;
        }
        
        case CrossingStateMachine::CrossingState::REAR_LEGS_TRANSIT: {
            // 后腿穿越：继续前移
            for (int k = 0; k < horizon; ++k) {
                Eigen::VectorXd ref_state = current_state;
                double progress = (k + 1.0) / horizon;
                
                ref_state(0) = current_pos(0) + 0.15 * progress;
                ref_state(2) = params_.default_height;
                ref_state(6) = params_.crossing_speed;
                
                trajectory[k] = ref_state;
            }
            break;
        }
        
        case CrossingStateMachine::CrossingState::COMPLETED: {
            // 完成：恢复正常行走
            Eigen::Vector3d velocity_cmd(params_.walking_speed, 0.0, 0.0);
            HybridGaitGenerator::GaitState dummy_gait;
            trajectory = generateWalkingTrajectory(current_state, velocity_cmd,
                                                  dummy_gait, horizon, dt);
            break;
        }
        
        default:
            trajectory = generateHoverTrajectory(current_state, horizon, dt);
            break;
    }
    
    return trajectory;
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::smoothTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory,
    double smoothness) {
    
    if (trajectory.size() < 2 || smoothness < 0.01) {
        return trajectory;
    }
    
    std::vector<Eigen::VectorXd> smoothed = trajectory;
    
    // 简单的移动平均平滑
    for (size_t k = 1; k < trajectory.size() - 1; ++k) {
        smoothed[k] = smoothness * trajectory[k] + 
                     (1.0 - smoothness) * 0.5 * (trajectory[k-1] + trajectory[k+1]);
    }
    
    return smoothed;
}

Eigen::VectorXd TrajectoryGenerator::interpolateState(
    const Eigen::VectorXd& state1,
    const Eigen::VectorXd& state2,
    double alpha) {
    
    return (1.0 - alpha) * state1 + alpha * state2;
}

Eigen::Vector4d TrajectoryGenerator::planSlidingPositions(
    const CrossingStateMachine::CrossingState& state,
    double progress) {
    
    Eigen::Vector4d sliding_pos = Eigen::Vector4d::Zero();
    
    switch (state) {
        case CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT:
            // 机身前探：滑动副伸展
            sliding_pos << 0.08, 0.08, 0.08, 0.08;
            sliding_pos *= progress;
            break;
            
        case CrossingStateMachine::CrossingState::HYBRID_GAIT_WALKING:
            // 混合构型：保持伸展
            sliding_pos << 0.08, 0.08, 0.08, 0.08;
            break;
            
        case CrossingStateMachine::CrossingState::COMPLETED:
            // 完成：收回滑动副
            sliding_pos << 0.08, 0.08, 0.08, 0.08;
            sliding_pos *= (1.0 - progress);
            break;
            
        default:
            sliding_pos.setZero();
            break;
    }
    
    return sliding_pos;
}

} // namespace dog2_mpc
