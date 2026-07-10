#ifndef DOG2_MPC_FLAT_LOCOMOTION_MPC_HPP
#define DOG2_MPC_FLAT_LOCOMOTION_MPC_HPP

#include <Eigen/Dense>

#include <array>
#include <memory>

#include "dog2_mpc/osqp_interface.hpp"

namespace dog2_mpc
{

struct FlatBodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct FlatBodyReference
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> wrench_feedforward =
    Eigen::Matrix<double, 6, 1>::Zero();
};

struct FlatMPCSolution
{
  Eigen::Matrix<double, 12, 1> foot_forces =
    Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 6, 1> desired_wrench =
    Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> achieved_wrench =
    Eigen::Matrix<double, 6, 1>::Zero();
  double wrench_residual = 0.0;
};

class FlatLocomotionMPC
{
public:
  struct Parameters
  {
    int horizon = 20;
    double dt = 0.02;
    double friction_coefficient = 0.65;
    double min_stance_force = 2.0;
    double max_stance_force = 95.0;
    double force_regularization = 2.0e-3;
    Eigen::Matrix<double, 12, 1> state_weights =
      (Eigen::Matrix<double, 12, 1>() <<
      12.0, 20.0, 160.0,
      220.0, 260.0, 40.0,
      20.0, 35.0, 50.0,
      24.0, 28.0, 8.0).finished();
    Eigen::Matrix<double, 6, 1> wrench_weights =
      (Eigen::Matrix<double, 6, 1>() <<
      0.08, 0.08, 0.025, 0.06, 0.06, 0.08).finished();
    Eigen::Matrix<double, 6, 1> allocation_weights =
      (Eigen::Matrix<double, 6, 1>() <<
      1.0, 1.0, 3.0, 12.0, 12.0, 6.0).finished();
    Eigen::Vector3d max_force = Eigen::Vector3d(55.0, 40.0, 185.0);
    Eigen::Vector3d max_torque = Eigen::Vector3d(20.0, 24.0, 10.0);
  };

  FlatLocomotionMPC(
    double mass,
    const Eigen::Matrix3d & inertia);

  FlatLocomotionMPC(
    double mass,
    const Eigen::Matrix3d & inertia,
    const Parameters & parameters);

  bool solve(
    const FlatBodyState & state,
    const FlatBodyReference & reference,
    const Eigen::Matrix<double, 4, 3> & feet_relative_world,
    const std::array<bool, 4> & contacts,
    FlatMPCSolution & solution);

  static bool hasThreeFootStaticSupportMargin(
    const Eigen::Matrix<double, 4, 3> & feet_relative_world,
    int swing_leg,
    double total_weight,
    double minimum_normal_force,
    Eigen::Vector3d * normal_forces = nullptr);

  void reset();

private:
  Eigen::Matrix<double, 6, 12> finiteHorizonGain() const;
  Eigen::Matrix<double, 6, 1> desiredWrench(
    const FlatBodyState & state,
    const FlatBodyReference & reference) const;
  bool allocateWrench(
    const Eigen::Matrix<double, 6, 1> & desired_wrench,
    const Eigen::Matrix<double, 4, 3> & feet_relative_world,
    const std::array<bool, 4> & contacts,
    FlatMPCSolution & solution);
  static double wrapAngle(double angle);

  double mass_;
  Eigen::Matrix3d inertia_;
  Parameters parameters_;
  std::unique_ptr<OSQPInterface> allocator_;
  Eigen::Matrix<double, 12, 1> previous_forces_ =
    Eigen::Matrix<double, 12, 1>::Zero();
};

}  // namespace dog2_mpc

#endif  // DOG2_MPC_FLAT_LOCOMOTION_MPC_HPP
