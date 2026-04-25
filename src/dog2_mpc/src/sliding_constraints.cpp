#include "dog2_mpc/sliding_constraints.hpp"

namespace dog2_mpc {

SlidingConstraints::SlidingConstraints()
    : v_slide_max_(1.0),
      epsilon_sym_(0.02),
      enable_symmetry_(true),
      enable_coordination_(true) {
    // Canonical rail limits from dog2.urdf.xacro:
    // lf/rh in [0.0, 0.111], lh/rf in [-0.111, 0.0].
    d_min_ << 0.0, -0.111, 0.0, -0.111;
    d_max_ << 0.111, 0.0, 0.111, 0.0;
}

void SlidingConstraints::setPositionLimits(const Eigen::Vector4d& lower,
                                          const Eigen::Vector4d& upper) {
    d_min_ = lower;
    d_max_ = upper;
}

void SlidingConstraints::setVelocityLimit(double v_max) {
    v_slide_max_ = v_max;
}

void SlidingConstraints::setSymmetryTolerance(double epsilon) {
    epsilon_sym_ = epsilon;
}

void SlidingConstraints::enableSymmetryConstraint(bool enable) {
    enable_symmetry_ = enable;
}

void SlidingConstraints::enableCoordinationConstraint(bool enable) {
    enable_coordination_ = enable;
}

void SlidingConstraints::addPositionConstraints(
    int horizon,
    std::vector<Eigen::Triplet<double>>& A_triplets,
    Eigen::VectorXd& l_ineq,
    Eigen::VectorXd& u_ineq,
    int& constraint_index) const {
    
    // 对每个时间步的每个滑动副添加位置约束
    // 假设状态向量中滑动副位置在特定位置（需要根据实际状态定义调整）
    // 这里假设状态是 [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, d1, d2, d3, d4]
    // 滑动副位置在索引 12-15
    
    const int state_dim = 16;  // 12 (SRBD) + 4 (sliding joints)
    const int sliding_start_idx = 12;
    
    for (int k = 0; k < horizon; ++k) {
        for (int i = 0; i < NUM_SLIDING_JOINTS; ++i) {
            int state_idx = k * state_dim + sliding_start_idx + i;
            
            // d_min[i] <= d_i <= d_max[i]
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, state_idx, 1.0));
            
            l_ineq(constraint_index) = d_min_(i);
            u_ineq(constraint_index) = d_max_(i);
            
            constraint_index++;
        }
    }
}

void SlidingConstraints::addVelocityConstraints(
    int horizon,
    double dt,
    std::vector<Eigen::Triplet<double>>& A_triplets,
    Eigen::VectorXd& l_ineq,
    Eigen::VectorXd& u_ineq,
    int& constraint_index) const {
    
    const int state_dim = 16;
    const int sliding_start_idx = 12;
    
    // 速度约束：|d[k+1] - d[k]| / dt <= v_max
    // 等价于：-v_max*dt <= d[k+1] - d[k] <= v_max*dt
    
    for (int k = 0; k < horizon - 1; ++k) {
        for (int i = 0; i < NUM_SLIDING_JOINTS; ++i) {
            int state_idx_k = k * state_dim + sliding_start_idx + i;
            int state_idx_k1 = (k + 1) * state_dim + sliding_start_idx + i;
            
            // d[k+1] - d[k]
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, state_idx_k1, 1.0));
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, state_idx_k, -1.0));
            
            l_ineq(constraint_index) = -v_slide_max_ * dt;
            u_ineq(constraint_index) = v_slide_max_ * dt;
            
            constraint_index++;
        }
    }
}

void SlidingConstraints::addSymmetryConstraints(
    int horizon,
    std::vector<Eigen::Triplet<double>>& A_triplets,
    Eigen::VectorXd& l_ineq,
    Eigen::VectorXd& u_ineq,
    int& constraint_index) const {
    
    if (!enable_symmetry_) {
        return;
    }
    
    const int state_dim = 16;
    const int sliding_start_idx = 12;
    
    // 对称约束：
    // |d1 - d3| <= epsilon  =>  -epsilon <= d1 - d3 <= epsilon
    // |d2 - d4| <= epsilon  =>  -epsilon <= d2 - d4 <= epsilon
    
    for (int k = 0; k < horizon; ++k) {
        // d1 - d3
        {
            int d1_idx = k * state_dim + sliding_start_idx + 0;
            int d3_idx = k * state_dim + sliding_start_idx + 2;
            
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, d1_idx, 1.0));
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, d3_idx, -1.0));
            
            l_ineq(constraint_index) = -epsilon_sym_;
            u_ineq(constraint_index) = epsilon_sym_;
            
            constraint_index++;
        }
        
        // d2 - d4
        {
            int d2_idx = k * state_dim + sliding_start_idx + 1;
            int d4_idx = k * state_dim + sliding_start_idx + 3;
            
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, d2_idx, 1.0));
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, d4_idx, -1.0));
            
            l_ineq(constraint_index) = -epsilon_sym_;
            u_ineq(constraint_index) = epsilon_sym_;
            
            constraint_index++;
        }
    }
}

void SlidingConstraints::addCoordinationConstraints(
    int horizon,
    std::vector<Eigen::Triplet<double>>& A_triplets,
    Eigen::VectorXd& l_ineq,
    Eigen::VectorXd& u_ineq,
    int& constraint_index) const {
    
    if (!enable_coordination_) {
        return;
    }
    
    const int state_dim = 16;
    const int sliding_start_idx = 12;
    const double coord_tolerance = 0.05;  // 5cm总和容差
    
    // 协调约束：Σ d_i ≈ 0
    // -tolerance <= d1 + d2 + d3 + d4 <= tolerance
    
    for (int k = 0; k < horizon; ++k) {
        for (int i = 0; i < NUM_SLIDING_JOINTS; ++i) {
            int state_idx = k * state_dim + sliding_start_idx + i;
            
            A_triplets.push_back(Eigen::Triplet<double>(
                constraint_index, state_idx, 1.0));
        }
        
        l_ineq(constraint_index) = -coord_tolerance;
        u_ineq(constraint_index) = coord_tolerance;
        
        constraint_index++;
    }
}

Eigen::MatrixXd SlidingConstraints::extractSlidingPositions(
    const Eigen::VectorXd& state_vector,
    int horizon) {
    
    const int state_dim = 16;
    const int sliding_start_idx = 12;
    
    Eigen::MatrixXd sliding_positions(horizon, NUM_SLIDING_JOINTS);
    
    for (int k = 0; k < horizon; ++k) {
        for (int i = 0; i < NUM_SLIDING_JOINTS; ++i) {
            int idx = k * state_dim + sliding_start_idx + i;
            sliding_positions(k, i) = state_vector(idx);
        }
    }
    
    return sliding_positions;
}

} // namespace dog2_mpc
