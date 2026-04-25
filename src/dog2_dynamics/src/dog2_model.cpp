#include "dog2_dynamics/dog2_model.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <iostream>

namespace dog2_dynamics {

Dog2Model::Dog2Model(const std::string& urdf_path) {
    // 加载URDF
    try {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        
        // 缓存足端frame ID
        cacheFrameIds();
        
        std::cout << "✓ Dog2 Model loaded successfully" << std::endl;
        std::cout << "  URDF: " << urdf_path << std::endl;
        std::cout << "  nq = " << model_.nq << ", nv = " << model_.nv << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "✗ Failed to load URDF: " << e.what() << std::endl;
        throw;
    }
}

void Dog2Model::cacheFrameIds() {
    foot_frame_ids_.clear();
    for (const auto& foot_name : FOOT_NAMES) {
        if (model_.existFrame(foot_name)) {
            foot_frame_ids_.push_back(model_.getFrameId(foot_name));
        } else {
            std::cerr << "Warning: Foot frame '" << foot_name << "' not found in URDF" << std::endl;
        }
    }
}

double Dog2Model::mass() const {
    return pinocchio::computeTotalMass(model_);
}

void Dog2Model::forwardKinematics(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateGlobalPlacements(model_, data_);
}

void Dog2Model::forwardKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    pinocchio::forwardKinematics(model_, data_, q, v);
    pinocchio::updateGlobalPlacements(model_, data_);
}

void Dog2Model::forwardKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
                                  const Eigen::VectorXd& a) {
    pinocchio::forwardKinematics(model_, data_, q, v, a);
    pinocchio::updateGlobalPlacements(model_, data_);
}

Eigen::Vector3d Dog2Model::centerOfMass(const Eigen::VectorXd& q) {
    return pinocchio::centerOfMass(model_, data_, q);
}

Eigen::Vector3d Dog2Model::centerOfMassVelocity(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    pinocchio::centerOfMass(model_, data_, q, v);
    return data_.vcom[0];
}

Eigen::Vector3d Dog2Model::footPosition(const std::string& foot_name, const Eigen::VectorXd& q) {
    forwardKinematics(q);
    
    if (!model_.existFrame(foot_name)) {
        std::cerr << "Error: Frame '" << foot_name << "' not found" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    auto frame_id = model_.getFrameId(foot_name);
    return data_.oMf[frame_id].translation();
}

std::vector<Eigen::Vector3d> Dog2Model::allFootPositions(const Eigen::VectorXd& q) {
    forwardKinematics(q);
    
    std::vector<Eigen::Vector3d> positions;
    for (auto frame_id : foot_frame_ids_) {
        positions.push_back(data_.oMf[frame_id].translation());
    }
    return positions;
}

Eigen::MatrixXd Dog2Model::footJacobian(const std::string& foot_name, const Eigen::VectorXd& q) {
    if (!model_.existFrame(foot_name)) {
        std::cerr << "Error: Frame '" << foot_name << "' not found" << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }
    
    auto frame_id = model_.getFrameId(foot_name);
    Eigen::MatrixXd J(6, model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::WORLD, J);
    return J;
}

Eigen::MatrixXd Dog2Model::comJacobian(const Eigen::VectorXd& q) {
    pinocchio::jacobianCenterOfMass(model_, data_, q);
    return data_.Jcom;
}

Eigen::MatrixXd Dog2Model::massMatrix(const Eigen::VectorXd& q) {
    pinocchio::crba(model_, data_, q);
    // 对称化质量矩阵（CRBA只计算上三角）
    data_.M.triangularView<Eigen::StrictlyLower>() = 
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data_.M;
}

Eigen::VectorXd Dog2Model::nonlinearEffects(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    return pinocchio::nonLinearEffects(model_, data_, q, v);
}

Eigen::VectorXd Dog2Model::gravityVector(const Eigen::VectorXd& q) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
    return pinocchio::rnea(model_, data_, q, v, a);
}

Dog2Model::SlidingJointState Dog2Model::getSlidingJointState(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    
    SlidingJointState state;
    
    // Dog2的前4个速度自由度是滑动副
    // 注意：nq=28（包含SO(2)表示），但nv=16
    // 滑动副在速度空间的前4维
    state.positions = q.head<4>();
    state.velocities = v.head<4>();
    state.forces.setZero();  // 力需要从逆动力学计算
    
    return state;
}

Eigen::Vector4d Dog2Model::slidingJointLowerLimits() const {
    // 从URDF中的关节限位
    Eigen::Vector4d limits;
    limits << 0.0, -0.111, 0.0, -0.111;  // lf, lh, rh, rf (单位：m)
    return limits;
}

Eigen::Vector4d Dog2Model::slidingJointUpperLimits() const {
    Eigen::Vector4d limits;
    limits << 0.111, 0.0, 0.111, 0.0;  // lf, lh, rh, rf (单位：m)
    return limits;
}

} // namespace dog2_dynamics
