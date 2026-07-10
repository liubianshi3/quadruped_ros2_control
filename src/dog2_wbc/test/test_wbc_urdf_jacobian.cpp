#include "dog2_wbc/wbc_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dog2_dynamics/dog2_model.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

constexpr const char * kLegPrefixes[4] = {"lf", "lh", "rh", "rf"};
constexpr const char * kFootFrames[4] = {
  "lf_foot_link",
  "lh_foot_link",
  "rh_foot_link",
  "rf_foot_link"
};

std::string shellQuote(const std::string & value)
{
  std::string quoted = "'";
  for (char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

std::string runXacro(const std::string & xacro_path, const std::string & controllers_yaml)
{
  const std::string command =
    "xacro " + shellQuote(xacro_path) +
    " controllers_yaml:=" + shellQuote(controllers_yaml);

  FILE * pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to run xacro command");
  }

  std::array<char, 4096> buffer{};
  std::string output;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }

  const int status = pclose(pipe);
  if (status != 0 || output.empty()) {
    std::ostringstream msg;
    msg << "xacro failed with status " << status << ": " << command;
    throw std::runtime_error(msg.str());
  }
  return output;
}

int jointVelocityIndex(
  const pinocchio::Model & model,
  const std::string & joint_name)
{
  if (!model.existJointName(joint_name)) {
    throw std::runtime_error("Missing joint: " + joint_name);
  }
  const auto joint_id = model.getJointId(joint_name);
  if (model.nvs[joint_id] != 1) {
    throw std::runtime_error("Expected single-DoF joint: " + joint_name);
  }
  return model.idx_vs[joint_id];
}

Eigen::VectorXd buildQ(
  const pinocchio::Model & model,
  int leg,
  const dog2_wbc::WBCController::LegState & state)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  const std::string prefix(kLegPrefixes[leg]);

  auto set_joint = [&](const std::string & name, double value) {
      const auto joint_id = model.getJointId(name);
      q(model.idx_qs[joint_id]) = value;
    };

  set_joint(prefix + "_rail_joint", state.sliding_position);
  set_joint(prefix + "_coxa_joint", state.joint_angles(0));
  set_joint(prefix + "_femur_joint", state.joint_angles(1));
  set_joint(prefix + "_tibia_joint", state.joint_angles(2));
  return q;
}

Eigen::MatrixXd directLegJacobian(
  dog2_dynamics::Dog2Model & model,
  int leg,
  const dog2_wbc::WBCController::LegState & state)
{
  const auto & pin_model = model.getModel();
  const Eigen::VectorXd q = buildQ(pin_model, leg, state);
  const Eigen::MatrixXd full = model.footJacobian(kFootFrames[leg], q);
  const std::string prefix(kLegPrefixes[leg]);

  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(3, 4);
  expected.col(0) = full.topRows<3>().col(jointVelocityIndex(pin_model, prefix + "_coxa_joint"));
  expected.col(1) = full.topRows<3>().col(jointVelocityIndex(pin_model, prefix + "_femur_joint"));
  expected.col(2) = full.topRows<3>().col(jointVelocityIndex(pin_model, prefix + "_tibia_joint"));
  expected.col(3) = full.topRows<3>().col(jointVelocityIndex(pin_model, prefix + "_rail_joint"));
  return expected;
}

}  // namespace

int main()
{
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("dog2_description");
  const std::string urdf_xml = runXacro(
    share_dir + "/urdf/dog2_symmetric.urdf.xacro",
    share_dir + "/config/ros2_controllers.yaml");

  dog2_wbc::WBCController wbc;
  wbc.initializeFromRobotDescription(urdf_xml);

  auto reference_model = dog2_dynamics::Dog2Model::fromUrdfXml(urdf_xml);

  dog2_wbc::WBCController::LegState state;
  state.joint_angles = Eigen::Vector3d(0.12, 0.34, -0.56);
  state.sliding_position = 0.0;
  state.config = dog2_wbc::WBCController::LegConfiguration::ELBOW;
  state.in_contact = true;

  // Ground truth: footJacobian linear rows must match a finite difference of
  // footPosition. Catches a wrong Pinocchio frame (e.g. WORLD), which the
  // self-referential comparison below cannot.
  {
    const auto & pin_model = reference_model.getModel();
    constexpr double kDelta = 1e-6;
    constexpr double kFdTolerance = 1e-5;
    const char * kJointSuffix[4] = {"_coxa_joint", "_femur_joint", "_tibia_joint", "_rail_joint"};
    for (int leg = 0; leg < 4; ++leg) {
      const std::string prefix(kLegPrefixes[leg]);
      const Eigen::VectorXd q = buildQ(pin_model, leg, state);
      const Eigen::MatrixXd J = reference_model.footJacobian(kFootFrames[leg], q).topRows<3>();
      for (int j = 0; j < 4; ++j) {
        const auto joint_id = pin_model.getJointId(prefix + kJointSuffix[j]);
        const int qi = pin_model.idx_qs[joint_id];
        const int vi = pin_model.idx_vs[joint_id];
        Eigen::VectorXd qp = q, qm = q;
        qp(qi) += kDelta;
        qm(qi) -= kDelta;
        const Eigen::Vector3d fd =
          (reference_model.footPosition(kFootFrames[leg], qp) -
          reference_model.footPosition(kFootFrames[leg], qm)) / (2.0 * kDelta);
        const double err = (J.col(vi) - fd).cwiseAbs().maxCoeff();
        if (err > kFdTolerance) {
          std::cerr << "footJacobian vs finite-diff mismatch, leg " << prefix
                    << " joint " << kJointSuffix[j] << ": max_err=" << err
                    << "\njacobian col:\n" << J.col(vi).transpose()
                    << "\nfinite diff:\n" << fd.transpose() << std::endl;
          return 1;
        }
      }
    }
  }

  constexpr double kTolerance = 1e-10;
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::MatrixXd actual = wbc.computeLegJacobian(leg, state);
    const Eigen::MatrixXd expected = directLegJacobian(reference_model, leg, state);
    const double err = (actual - expected).cwiseAbs().maxCoeff();
    if (err > kTolerance) {
      std::cerr << "WBC URDF Jacobian mismatch for leg " << kLegPrefixes[leg]
                << ": max_err=" << err << "\nactual:\n" << actual
                << "\nexpected:\n" << expected << std::endl;
      return 1;
    }
  }

  dog2_wbc::WBCController::Parameters positive_params;
  positive_params.foot_force_sign = 1.0;
  positive_params.gravity_compensation = false;
  dog2_wbc::WBCController positive_wbc(positive_params);
  positive_wbc.initializeFromRobotDescription(urdf_xml);

  dog2_wbc::WBCController::Parameters negative_params;
  negative_params.foot_force_sign = -1.0;
  negative_params.gravity_compensation = false;
  dog2_wbc::WBCController negative_wbc(negative_params);
  negative_wbc.initializeFromRobotDescription(urdf_xml);

  std::array<dog2_wbc::WBCController::LegState, 4> leg_states;
  for (auto & leg_state : leg_states) {
    leg_state = state;
  }
  const Eigen::VectorXd foot_forces = Eigen::VectorXd::Constant(12, 1.0);
  const Eigen::VectorXd positive_torques =
    positive_wbc.computeTorques(foot_forces, leg_states);
  const Eigen::VectorXd negative_torques =
    negative_wbc.computeTorques(foot_forces, leg_states);
  const double sign_err = (positive_torques + negative_torques).cwiseAbs().maxCoeff();
  if (sign_err > kTolerance) {
    std::cerr << "WBC foot_force_sign should only flip torque sign, max_err="
              << sign_err << "\npositive:\n" << positive_torques.transpose()
              << "\nnegative:\n" << negative_torques.transpose() << std::endl;
    return 1;
  }

  for (auto & leg_state : leg_states) {
    leg_state.in_contact = false;
    leg_state.swing = false;
  }
  const Eigen::VectorXd no_target_torques =
    positive_wbc.computeTorques(Eigen::VectorXd::Zero(12), leg_states);
  if (no_target_torques.norm() > kTolerance) {
    std::cerr << "Swing legs without a target should not receive torque, got "
              << no_target_torques.transpose() << std::endl;
    return 1;
  }

  leg_states[0].swing = true;
  leg_states[0].swing_pos_des =
    positive_wbc.forwardKinematics(0, leg_states[0]) + Eigen::Vector3d(0.0, 0.0, 0.02);
  leg_states[0].swing_vel_des.setZero();
  leg_states[0].joint_velocities.setZero();
  const Eigen::VectorXd swing_torques =
    positive_wbc.computeTorques(Eigen::VectorXd::Zero(12), leg_states);
  if (swing_torques.segment<3>(0).norm() <= 1e-6) {
    std::cerr << "Swing Cartesian PD should produce rotational joint torques, got "
              << swing_torques.transpose() << std::endl;
    return 1;
  }
  if (swing_torques.tail<4>().norm() > kTolerance) {
    std::cerr << "Swing Cartesian PD must not command rail efforts, got "
              << swing_torques.transpose() << std::endl;
    return 1;
  }

  {
    dog2_wbc::WBCController::Parameters limit_params;
    limit_params.gravity_compensation = false;
    limit_params.joint_limit_margin = 0.12;
    limit_params.joint_limit_kp = 35.0;
    limit_params.joint_limit_kd = 1.5;
    dog2_wbc::WBCController limit_wbc(limit_params);
    limit_wbc.initializeFromRobotDescription(urdf_xml);

    std::array<dog2_wbc::WBCController::LegState, 4> limit_legs;
    for (auto & leg : limit_legs) {
      leg.joint_angles.setZero();
      leg.joint_velocities.setZero();
      leg.sliding_position = 0.0;
      leg.in_contact = false;
      leg.swing = false;
    }
    limit_legs[2].joint_angles(1) = 2.75;
    limit_legs[2].joint_velocities(1) = 0.2;
    const Eigen::VectorXd limit_torques =
      limit_wbc.computeTorques(Eigen::VectorXd::Zero(12), limit_legs);
    if (limit_torques(2 * 3 + 1) >= 0.0) {
      std::cerr << "Upper soft limit must produce a negative restoring torque, got "
                << limit_torques(2 * 3 + 1) << std::endl;
      return 1;
    }
  }

  // Method A: standalone stance check. With gravity compensation on, the WBC
  // torques must equal J^T*f + g(q) sliced at the four joint v-indices. The
  // old fixed expectations (-7.54 / -9.88) baked in J^T*f only.
  {
    dog2_wbc::WBCController::LegState stance;
    stance.joint_angles = Eigen::Vector3d(0.0, -0.3, -0.6);
    stance.sliding_position = 0.0;
    stance.config = dog2_wbc::WBCController::LegConfiguration::ELBOW;
    stance.in_contact = true;
    stance.swing = false;

    std::array<dog2_wbc::WBCController::LegState, 4> legs;
    for (auto & l : legs) {
      l = stance;
    }

    constexpr double kMass = 12.003;
    constexpr double kG = 9.81;
    const double fz = kMass * kG / 4.0;
    Eigen::VectorXd foot_forces(12);
    for (int i = 0; i < 4; ++i) {
      foot_forces.segment<3>(i * 3) = Eigen::Vector3d(0.0, 0.0, fz);
    }

    dog2_wbc::WBCController::Parameters params;
    params.foot_force_sign = 1.0;
    params.gravity_compensation = true;
    dog2_wbc::WBCController stance_wbc(params);
    stance_wbc.initializeFromRobotDescription(urdf_xml);
    const Eigen::VectorXd tau = stance_wbc.computeTorques(foot_forces, legs);

    auto reference_model_local = dog2_dynamics::Dog2Model::fromUrdfXml(urdf_xml);
    const auto & pin = reference_model_local.getModel();
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(pin.nq);
    for (int leg = 0; leg < 4; ++leg) {
      const std::string prefix(kLegPrefixes[leg]);
      auto set_q = [&](const std::string & name, double value) {
          const auto jid = pin.getJointId(name);
          q_full(pin.idx_qs[jid]) = value;
        };
      set_q(prefix + "_rail_joint", legs[leg].sliding_position);
      set_q(prefix + "_coxa_joint", legs[leg].joint_angles(0));
      set_q(prefix + "_femur_joint", legs[leg].joint_angles(1));
      set_q(prefix + "_tibia_joint", legs[leg].joint_angles(2));
    }
    const Eigen::VectorXd g_full = reference_model_local.gravityVector(q_full);

    std::cout << "[stance check] per-leg fz=" << fz << " N (gravity comp ON)\n";
    constexpr double kStanceTol = 1e-6;
    for (int leg = 0; leg < 4; ++leg) {
      const std::string prefix(kLegPrefixes[leg]);
      const Eigen::MatrixXd J = stance_wbc.computeLegJacobian(leg, legs[leg]);
      const Eigen::Vector4d jt_f =
        J.transpose() * foot_forces.segment<3>(leg * 3);
      const int coxa_v = pin.idx_vs[pin.getJointId(prefix + "_coxa_joint")];
      const int femur_v = pin.idx_vs[pin.getJointId(prefix + "_femur_joint")];
      const int tibia_v = pin.idx_vs[pin.getJointId(prefix + "_tibia_joint")];
      const int rail_v = pin.idx_vs[pin.getJointId(prefix + "_rail_joint")];
      const double exp_coxa = jt_f(0) + g_full(coxa_v);
      const double exp_femur = jt_f(1) + g_full(femur_v);
      const double exp_tibia = jt_f(2) + g_full(tibia_v);
      const double exp_rail = jt_f(3) + g_full(rail_v);
      std::cout << "  " << prefix
                << " coxa=" << tau(leg * 3 + 0) << "(exp " << exp_coxa << ")"
                << " femur=" << tau(leg * 3 + 1) << "(exp " << exp_femur << ")"
                << " tibia=" << tau(leg * 3 + 2) << "(exp " << exp_tibia << ")"
                << " rail=" << tau(12 + leg) << "(exp " << exp_rail << ")\n";
      if (std::abs(tau(leg * 3 + 0) - exp_coxa) > kStanceTol ||
        std::abs(tau(leg * 3 + 1) - exp_femur) > kStanceTol ||
        std::abs(tau(leg * 3 + 2) - exp_tibia) > kStanceTol ||
        std::abs(tau(12 + leg) - exp_rail) > kStanceTol)
      {
        std::cerr << "Stance torques disagree with J^T*f + g(q) for leg " << prefix
                  << std::endl;
        return 1;
      }
    }
    std::cout << "[PASS] stance torques = J^T*f + g(q)\n";

    // Gravity-only: zero foot force, gravity comp on, must match g(q) slice.
    const Eigen::VectorXd tau_grav =
      stance_wbc.computeTorques(Eigen::VectorXd::Zero(12), legs);
    for (int leg = 0; leg < 4; ++leg) {
      const std::string prefix(kLegPrefixes[leg]);
      const int coxa_v = pin.idx_vs[pin.getJointId(prefix + "_coxa_joint")];
      const int femur_v = pin.idx_vs[pin.getJointId(prefix + "_femur_joint")];
      const int tibia_v = pin.idx_vs[pin.getJointId(prefix + "_tibia_joint")];
      const int rail_v = pin.idx_vs[pin.getJointId(prefix + "_rail_joint")];
      if (std::abs(tau_grav(leg * 3 + 0) - g_full(coxa_v)) > kStanceTol ||
        std::abs(tau_grav(leg * 3 + 1) - g_full(femur_v)) > kStanceTol ||
        std::abs(tau_grav(leg * 3 + 2) - g_full(tibia_v)) > kStanceTol ||
        std::abs(tau_grav(12 + leg) - g_full(rail_v)) > kStanceTol)
      {
        std::cerr << "Gravity-only torques != g(q) slice for leg " << prefix
                  << std::endl;
        return 1;
      }
    }
    std::cout << "[PASS] gravity-only torques = g(q) slice\n";
  }

  std::cout << "[PASS] WBC URDF Jacobian matches Dog2Model footJacobian for lf/lh/rh/rf"
            << std::endl;
  return 0;
}
