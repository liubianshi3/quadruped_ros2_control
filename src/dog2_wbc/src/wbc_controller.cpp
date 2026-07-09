#include "dog2_wbc/wbc_controller.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace dog2_wbc
{

namespace
{

constexpr const char * kLegPrefixes[4] = {"lf", "lh", "rh", "rf"};
constexpr const char * kFootFrames[4] = {
  "lf_foot_link",
  "lh_foot_link",
  "rh_foot_link",
  "rf_foot_link"
};

}  // namespace

WBCController::WBCController(const Parameters & params)
: params_(params)
{
  last_stats_.success = false;
  last_stats_.solve_time_ms = 0.0;
  last_stats_.torque_norm = 0.0;
}

void WBCController::initializeFromRobotDescription(const std::string & robot_description)
{
  if (robot_description.empty()) {
    dog2_model_.reset();
    return;
  }
  dog2_model_ = std::make_unique<dog2_dynamics::Dog2Model>(
    dog2_dynamics::Dog2Model::fromUrdfXml(robot_description));
}

Eigen::VectorXd WBCController::computeTorques(
  const Eigen::VectorXd & foot_forces,
  const std::array<LegState, 4> & leg_states)
{

  auto start_time = std::chrono::high_resolution_clock::now();

  // 输出：12个旋转关节力矩 + 4个滑动副力
  Eigen::VectorXd torques = Eigen::VectorXd::Zero(16);

  // 对每条腿计算力矩：τ = J^T * f
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::MatrixXd J = computeLegJacobian(leg, leg_states[leg]);

    if (leg_states[leg].swing) {
      const Eigen::Vector3d p = forwardKinematics(leg, leg_states[leg]);
      const Eigen::Vector3d v =
        J.leftCols<3>() * leg_states[leg].joint_velocities;
      const Eigen::Vector3d f_virtual =
        params_.swing_kp.cwiseProduct(leg_states[leg].swing_pos_des - p) +
        params_.swing_kd.cwiseProduct(leg_states[leg].swing_vel_des - v);
      const Eigen::Vector4d leg_torques = J.transpose() * f_virtual;
      torques.segment<3>(leg * 3) = leg_torques.segment<3>(0);
      continue;
    }

    if (!leg_states[leg].in_contact) {
      continue;
    }

    // 提取该腿的足端力（MPC 输出，世界系），旋回 base 系再做 J^T：
    // 雅可比在固定基座模型的 base 系表达，躯干俯仰时直接用世界系力
    // 会把竖直支撑按 sin(tilt) 泄漏成水平推搡。
    Eigen::Vector3d f_leg = R_wb_.transpose() * foot_forces.segment<3>(leg * 3);

    // 计算力矩：τ = sign * J^T * f
    // J是3×4矩阵，J^T是4×3
    // τ = [τ_hip_roll, τ_hip_pitch, τ_knee, f_sliding]^T
    Eigen::Vector4d leg_torques = params_.foot_force_sign * J.transpose() * f_leg;

    // 分配到输出向量
    // 旋转关节力矩
    torques.segment<3>(leg * 3) = leg_torques.segment<3>(0);

    // 滑动副力
    torques(12 + leg) = leg_torques(3);
  }

  // Static gravity compensation. The fixed-base model treats base_link as
  // ground, so g(q) reports the joint torques needed to hold each link
  // against gravity at the current configuration. Adding this to J^T*f
  // eliminates the bias that previously got hand-tuned in the mux.
  // Gravity itself must also be expressed in the (possibly tilted) base
  // frame: g_base = R_wb^T * (0, 0, -9.81).
  //
  // SWING legs need this term the most: each leg is ~1.5 kg (the legs are
  // half the robot mass), so without its own-weight feedforward the
  // task-space swing PD (kp_z 300) sags ~5 cm at steady state -- more than
  // the whole 4-5 cm swing apex. The swing foot never actually cleared the
  // ground; it dragged and tripped the trunk once per stride (the violent
  // per-stride wx/wy spikes of run40-54).
  if (params_.gravity_compensation && dog2_model_) {
    const Eigen::VectorXd q = buildFullPinocchioConfiguration(leg_states);
    const Eigen::Vector3d gravity_base =
      R_wb_.transpose() * Eigen::Vector3d(0.0, 0.0, -9.81);
    const Eigen::VectorXd g = dog2_model_->gravityVector(q, gravity_base);
    for (int leg = 0; leg < 4; ++leg) {
      if (!leg_states[leg].swing && !leg_states[leg].in_contact) {
        continue;
      }
      const std::string prefix(kLegPrefixes[leg]);
      const int coxa_v = jointVelocityIndex(prefix + "_coxa_joint");
      const int femur_v = jointVelocityIndex(prefix + "_femur_joint");
      const int tibia_v = jointVelocityIndex(prefix + "_tibia_joint");
      const int rail_v = jointVelocityIndex(prefix + "_rail_joint");
      torques(leg * 3 + 0) += g(coxa_v);
      torques(leg * 3 + 1) += g(femur_v);
      torques(leg * 3 + 2) += g(tibia_v);
      torques(12 + leg) += g(rail_v);
    }
  }

  // 应用力矩限制
  applyTorqueLimits(torques);

  // 统计
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
    end_time - start_time);

  last_stats_.solve_time_ms = duration.count() / 1000.0;
  last_stats_.torque_norm = torques.norm();
  last_stats_.success = true;

  return torques;
}

Eigen::MatrixXd WBCController::computeLegJacobian(
  int leg_id,
  const LegState & leg_state)
{

  if (dog2_model_) {
    return computeUrdfJacobian(leg_id, leg_state);
  }

  // 根据构型选择不同的雅可比计算
  if (leg_state.config == LegConfiguration::ELBOW) {
    return computeElbowJacobian(
      leg_id,
      leg_state.joint_angles,
      leg_state.sliding_position);
  } else {
    return computeKneeJacobian(
      leg_id,
      leg_state.joint_angles,
      leg_state.sliding_position);
  }
}

Eigen::MatrixXd WBCController::computeUrdfJacobian(
  int leg_id,
  const LegState & leg_state)
{

  if (!dog2_model_) {
    throw std::runtime_error("URDF Jacobian requested before Dog2Model initialization");
  }
  if (leg_id < 0 || leg_id >= 4) {
    throw std::out_of_range("leg_id must be in [0, 3]");
  }

  const Eigen::VectorXd q = buildPinocchioConfiguration(leg_id, leg_state);
  const Eigen::MatrixXd full_jacobian =
    dog2_model_->footJacobian(kFootFrames[leg_id], q);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);
  const std::string prefix(kLegPrefixes[leg_id]);
  const int coxa_col = jointVelocityIndex(prefix + "_coxa_joint");
  const int femur_col = jointVelocityIndex(prefix + "_femur_joint");
  const int tibia_col = jointVelocityIndex(prefix + "_tibia_joint");
  const int rail_col = jointVelocityIndex(prefix + "_rail_joint");

  J.col(0) = full_jacobian.topRows<3>().col(coxa_col);
  J.col(1) = full_jacobian.topRows<3>().col(femur_col);
  J.col(2) = full_jacobian.topRows<3>().col(tibia_col);
  J.col(3) = full_jacobian.topRows<3>().col(rail_col);
  return J;
}

Eigen::VectorXd WBCController::buildPinocchioConfiguration(
  int leg_id,
  const LegState & leg_state) const
{

  if (!dog2_model_) {
    return Eigen::VectorXd();
  }
  if (leg_id < 0 || leg_id >= 4) {
    throw std::out_of_range("leg_id must be in [0, 3]");
  }

  const auto & model = dog2_model_->getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  const std::string prefix(kLegPrefixes[leg_id]);

  auto set_joint_q = [&](const std::string & joint_name, double value) {
      if (!model.existJointName(joint_name)) {
        throw std::runtime_error("Missing joint in URDF model: " + joint_name);
      }
      const auto joint_id = model.getJointId(joint_name);
      if (model.nqs[joint_id] != 1) {
        throw std::runtime_error("Expected single-DoF joint in URDF model: " + joint_name);
      }
      q(model.idx_qs[joint_id]) = value;
    };

  set_joint_q(prefix + "_rail_joint", leg_state.sliding_position);
  set_joint_q(prefix + "_coxa_joint", leg_state.joint_angles(0));
  set_joint_q(prefix + "_femur_joint", leg_state.joint_angles(1));
  set_joint_q(prefix + "_tibia_joint", leg_state.joint_angles(2));
  return q;
}

Eigen::VectorXd WBCController::buildFullPinocchioConfiguration(
  const std::array<LegState, 4> & leg_states) const
{

  if (!dog2_model_) {
    return Eigen::VectorXd();
  }

  const auto & model = dog2_model_->getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  auto set_joint_q = [&](const std::string & joint_name, double value) {
      if (!model.existJointName(joint_name)) {
        throw std::runtime_error("Missing joint in URDF model: " + joint_name);
      }
      const auto joint_id = model.getJointId(joint_name);
      q(model.idx_qs[joint_id]) = value;
    };

  for (int leg = 0; leg < 4; ++leg) {
    const std::string prefix(kLegPrefixes[leg]);
    set_joint_q(prefix + "_rail_joint", leg_states[leg].sliding_position);
    set_joint_q(prefix + "_coxa_joint", leg_states[leg].joint_angles(0));
    set_joint_q(prefix + "_femur_joint", leg_states[leg].joint_angles(1));
    set_joint_q(prefix + "_tibia_joint", leg_states[leg].joint_angles(2));
  }
  return q;
}

int WBCController::jointVelocityIndex(const std::string & joint_name) const
{
  if (!dog2_model_) {
    throw std::runtime_error("jointVelocityIndex called without Dog2Model");
  }
  const auto & model = dog2_model_->getModel();
  if (!model.existJointName(joint_name)) {
    throw std::runtime_error("Missing joint in URDF model: " + joint_name);
  }
  const auto joint_id = model.getJointId(joint_name);
  if (model.nvs[joint_id] != 1) {
    throw std::runtime_error("Expected single-DoF joint velocity in URDF model: " + joint_name);
  }
  return model.idx_vs[joint_id];
}

Eigen::MatrixXd WBCController::computeElbowJacobian(
  int leg_id,
  const Eigen::Vector3d & q,
  double d)
{

  // 肘式构型的雅可比矩阵
  // q = [q_hip_roll, q_hip_pitch, q_knee]
  // d = 滑动副位置

  double l1 = params_.l1;
  double l2 = params_.l2;

  double q1 = q(0);    // hip_roll
  double q2 = q(1);    // hip_pitch
  double q3 = q(2);    // knee

  // 肘式：膝关节向后突出
  // 足端位置（简化）：
  // x = d + l1*cos(q2) + l2*cos(q2+q3)
  // y = hip_offset_y + l1*sin(q1)*sin(q2) + l2*sin(q1)*sin(q2+q3)
  // z = -l1*cos(q1)*sin(q2) - l2*cos(q1)*sin(q2+q3)

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);

  double s1 = std::sin(q1);
  double c1 = std::cos(q1);
  double s2 = std::sin(q2);
  double c2 = std::cos(q2);
  double s23 = std::sin(q2 + q3);
  double c23 = std::cos(q2 + q3);

  // ∂p/∂q1 (hip_roll)
  J(0, 0) = 0.0;
  J(1, 0) = l1 * c1 * s2 + l2 * c1 * s23;
  J(2, 0) = l1 * s1 * s2 + l2 * s1 * s23;

  // ∂p/∂q2 (hip_pitch)
  J(0, 1) = -l1 * s2 - l2 * s23;
  J(1, 1) = l1 * s1 * c2 + l2 * s1 * c23;
  J(2, 1) = -l1 * c1 * c2 - l2 * c1 * c23;

  // ∂p/∂q3 (knee)
  J(0, 2) = -l2 * s23;
  J(1, 2) = l2 * s1 * c23;
  J(2, 2) = -l2 * c1 * c23;

  // ∂p/∂d (sliding)
  J(0, 3) = 1.0;
  J(1, 3) = 0.0;
  J(2, 3) = 0.0;

  return J;
}

Eigen::MatrixXd WBCController::computeKneeJacobian(
  int leg_id,
  const Eigen::Vector3d & q,
  double d)
{

  // 膝式构型的雅可比矩阵
  // 膝关节向前收束

  double l1 = params_.l1;
  double l2 = params_.l2;

  double q1 = q(0);    // hip_roll
  double q2 = q(1);    // hip_pitch
  double q3 = q(2);    // knee

  // 膝式：膝关节向前收束
  // 足端位置（简化）：
  // x = d + l1*cos(q2) - l2*cos(q2-q3)  // 注意符号变化
  // y = hip_offset_y + l1*sin(q1)*sin(q2) - l2*sin(q1)*sin(q2-q3)
  // z = -l1*cos(q1)*sin(q2) + l2*cos(q1)*sin(q2-q3)

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);

  double s1 = std::sin(q1);
  double c1 = std::cos(q1);
  double s2 = std::sin(q2);
  double c2 = std::cos(q2);
  double s2_3 = std::sin(q2 - q3);    // 注意符号
  double c2_3 = std::cos(q2 - q3);

  // ∂p/∂q1 (hip_roll)
  J(0, 0) = 0.0;
  J(1, 0) = l1 * c1 * s2 - l2 * c1 * s2_3;
  J(2, 0) = l1 * s1 * s2 - l2 * s1 * s2_3;

  // ∂p/∂q2 (hip_pitch)
  J(0, 1) = -l1 * s2 + l2 * s2_3;
  J(1, 1) = l1 * s1 * c2 - l2 * s1 * c2_3;
  J(2, 1) = -l1 * c1 * c2 + l2 * c1 * c2_3;

  // ∂p/∂q3 (knee)
  J(0, 2) = l2 * s2_3;    // 符号变化
  J(1, 2) = -l2 * s1 * c2_3;
  J(2, 2) = l2 * c1 * c2_3;

  // ∂p/∂d (sliding)
  J(0, 3) = 1.0;
  J(1, 3) = 0.0;
  J(2, 3) = 0.0;

  return J;
}

Eigen::Vector3d WBCController::forwardKinematics(
  int leg_id,
  const LegState & leg_state)
{

  if (dog2_model_) {
    return dog2_model_->footPosition(
      kFootFrames[leg_id],
      buildPinocchioConfiguration(leg_id, leg_state));
  }

  double l1 = params_.l1;
  double l2 = params_.l2;

  double q1 = leg_state.joint_angles(0);
  double q2 = leg_state.joint_angles(1);
  double q3 = leg_state.joint_angles(2);
  double d = leg_state.sliding_position;

  Eigen::Vector3d foot_pos;

  if (leg_state.config == LegConfiguration::ELBOW) {
    // 肘式
    foot_pos(0) = d + l1 * std::cos(q2) + l2 * std::cos(q2 + q3);
    foot_pos(1) = params_.hip_offset_y +
      l1 * std::sin(q1) * std::sin(q2) +
      l2 * std::sin(q1) * std::sin(q2 + q3);
    foot_pos(2) = -l1 * std::cos(q1) * std::sin(q2) -
      l2 * std::cos(q1) * std::sin(q2 + q3);
  } else {
    // 膝式
    foot_pos(0) = d + l1 * std::cos(q2) - l2 * std::cos(q2 - q3);
    foot_pos(1) = params_.hip_offset_y +
      l1 * std::sin(q1) * std::sin(q2) -
      l2 * std::sin(q1) * std::sin(q2 - q3);
    foot_pos(2) = -l1 * std::cos(q1) * std::sin(q2) +
      l2 * std::cos(q1) * std::sin(q2 - q3);
  }

  // 考虑腿的位置（前腿/后腿，左/右）
  double sign_x = (leg_id == 0 || leg_id == 3) ? -1.0 : 1.0;    // 前腿负，后腿正
  double sign_y = (leg_id == 0 || leg_id == 1) ? -1.0 : 1.0;    // 左腿负，右腿正

  foot_pos(0) = sign_x * params_.hip_offset_x + foot_pos(0);
  foot_pos(1) = sign_y * foot_pos(1);

  return foot_pos;
}

void WBCController::applyTorqueLimits(Eigen::VectorXd & torques)
{
  // 旋转关节力矩限制
  for (int i = 0; i < 12; ++i) {
    if (torques(i) > params_.max_torque) {
      std::cout << "[WBCController] HARD CLAMP Joint " << i << " torque " << torques(i) << " -> " <<
        params_.max_torque << std::endl;
      torques(i) = params_.max_torque;
    } else if (torques(i) < -params_.max_torque) {
      std::cout << "[WBCController] HARD CLAMP Joint " << i << " torque " << torques(i) <<
        " -> -" << params_.max_torque << std::endl;
      torques(i) = -params_.max_torque;
    }
  }

  // 滑动副力限制
  for (int i = 12; i < 16; ++i) {
    if (torques(i) > params_.max_sliding_force) {
      std::cout << "[WBCController] HARD CLAMP Rail " << i - 12 << " force " << torques(i) <<
        " -> " << params_.max_sliding_force << std::endl;
      torques(i) = params_.max_sliding_force;
    } else if (torques(i) < -params_.max_sliding_force) {
      std::cout << "[WBCController] HARD CLAMP Rail " << i - 12 << " force " << torques(i) <<
        " -> -" << params_.max_sliding_force << std::endl;
      torques(i) = -params_.max_sliding_force;
    }
  }
}

} // namespace dog2_wbc
