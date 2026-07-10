#include "dog2_mpc/flat_locomotion_mpc.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

namespace
{

Eigen::Matrix<double, 4, 3> nominalFeet()
{
  Eigen::Matrix<double, 4, 3> feet;
  feet <<
    -0.138, -0.118, -0.20,
    0.138, -0.118, -0.20,
    0.138, 0.118, -0.20,
    -0.138, 0.118, -0.20;
  return feet;
}

dog2_mpc::FlatLocomotionMPC makeController()
{
  return dog2_mpc::FlatLocomotionMPC(
    12.0,
    (Eigen::Vector3d(0.20, 0.35, 0.42)).asDiagonal());
}

void testStaticSupport()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = 0.20;
  reference.position.z() = 0.20;
  dog2_mpc::FlatMPCSolution solution;

  const bool solved = controller.solve(
    state, reference, nominalFeet(), {true, true, true, true}, solution);
  assert(solved);
  assert(std::abs(solution.achieved_wrench.z() - 12.0 * 9.81) < 1.0);
  assert(solution.achieved_wrench.tail<3>().norm() < 0.1);
  for (int leg = 0; leg < 4; ++leg) {
    assert(solution.foot_forces(leg * 3 + 2) > 20.0);
  }
}

void testSwingLegIsUnloaded()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = reference.position.z() = 0.20;
  dog2_mpc::FlatMPCSolution solution;

  const bool solved = controller.solve(
    state, reference, nominalFeet(), {true, false, true, true}, solution);
  assert(solved);
  assert(solution.foot_forces.segment<3>(3).norm() < 1.0e-4);
  assert(std::abs(solution.achieved_wrench.z() - 12.0 * 9.81) < 2.0);
  assert(solution.achieved_wrench.tail<3>().norm() < 0.3);
}

void testShiftedThreeFootSupportBalancesMoment()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = reference.position.z() = 0.20;
  Eigen::Matrix<double, 4, 3> feet = nominalFeet();
  feet.rowwise() += Eigen::RowVector3d(0.030, -0.026, 0.0);
  dog2_mpc::FlatMPCSolution solution;

  assert(
    controller.solve(
      state, reference, feet, {true, false, true, true}, solution));
  assert(solution.foot_forces.segment<3>(3).norm() < 1.0e-4);
  assert(std::abs(solution.achieved_wrench.z() - 12.0 * 9.81) < 2.0);
  assert(solution.achieved_wrench.tail<3>().norm() < 0.2);
}

void testAttitudeMomentRestoresPitch()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = reference.position.z() = 0.20;
  state.rpy.y() = 0.12;
  dog2_mpc::FlatMPCSolution solution;

  const bool solved = controller.solve(
    state, reference, nominalFeet(), {true, true, true, true}, solution);
  assert(solved);
  assert(solution.desired_wrench(4) < 0.0);
  assert(solution.achieved_wrench(4) < 0.0);
}

void testFrictionConeAndNoContactFailure()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = reference.position.z() = 0.20;
  reference.linear_velocity.x() = 2.0;
  dog2_mpc::FlatMPCSolution solution;

  const std::array<bool, 4> contacts{true, true, true, false};
  assert(
    controller.solve(
      state, reference, nominalFeet(), contacts, solution));
  for (int leg = 0; leg < 4; ++leg) {
    const double fx = solution.foot_forces(leg * 3);
    const double fy = solution.foot_forces(leg * 3 + 1);
    const double fz = solution.foot_forces(leg * 3 + 2);
    assert(std::abs(fx) <= 0.65 * fz + 1.0e-3);
    assert(std::abs(fy) <= 0.65 * fz + 1.0e-3);
  }

  assert(
    !controller.solve(
      state, reference, nominalFeet(), {false, false, false, false}, solution));
}

void testSupportShiftFeedforwardIsApplied()
{
  auto controller = makeController();
  dog2_mpc::FlatBodyState state;
  dog2_mpc::FlatBodyReference reference;
  state.position.z() = reference.position.z() = 0.20;
  reference.wrench_feedforward.x() = 6.0;
  reference.wrench_feedforward.y() = -4.0;
  dog2_mpc::FlatMPCSolution solution;

  assert(
    controller.solve(
      state, reference, nominalFeet(), {true, true, true, true}, solution));
  assert(std::abs(solution.desired_wrench.x() - 6.0) < 1.0e-6);
  assert(std::abs(solution.desired_wrench.y() + 4.0) < 1.0e-6);
}

void testMeasuredStartupSupportRejectsImpossibleHindUnload()
{
  Eigen::Matrix<double, 4, 3> feet;
  feet <<
    -0.185, -0.131, -0.20,
    0.079, -0.130, -0.20,
    0.079, 0.105, -0.20,
    -0.186, 0.105, -0.20;
  Eigen::Vector3d normal_forces;

  assert(
    dog2_mpc::FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
      feet, 0, 12.0 * 9.81, 2.0, &normal_forces));
  assert(normal_forces.minCoeff() > 20.0);
  assert(
    !dog2_mpc::FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
      feet, 1, 12.0 * 9.81, 2.0));
}

}  // namespace

int main()
{
  testStaticSupport();
  testSwingLegIsUnloaded();
  testShiftedThreeFootSupportBalancesMoment();
  testAttitudeMomentRestoresPitch();
  testFrictionConeAndNoContactFailure();
  testSupportShiftFeedforwardIsApplied();
  testMeasuredStartupSupportRejectsImpossibleHindUnload();
  std::cout << "flat locomotion MPC tests passed" << std::endl;
  return 0;
}
