#include "dog2_mpc/flat_locomotion_mpc.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

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

void testAdaptiveShiftDoesNotRewriteIntegratedReference()
{
  const Eigen::Vector2d integrated_reference(0.30, -0.20);
  const Eigen::Vector2d adaptive_shift(0.12, 0.06);
  const Eigen::Vector2d measured_position =
    integrated_reference + adaptive_shift;

  const Eigen::Vector2d bounded =
    dog2_mpc::boundIntegratedPositionReference(
    integrated_reference, adaptive_shift, measured_position, 0.10);

  assert((bounded - integrated_reference).norm() < 1.0e-12);
}

void testShiftedTargetTrackingErrorRemainsBounded()
{
  const Eigen::Vector2d integrated_reference(0.20, -0.10);
  const Eigen::Vector2d adaptive_shift(0.12, 0.06);
  const Eigen::Vector2d measured_position = Eigen::Vector2d::Zero();

  const Eigen::Vector2d bounded =
    dog2_mpc::boundIntegratedPositionReference(
    integrated_reference, adaptive_shift, measured_position, 0.10);
  const Eigen::Vector2d bounded_target = bounded + adaptive_shift;
  const Eigen::Vector2d original_error =
    integrated_reference + adaptive_shift - measured_position;

  assert(std::abs((bounded_target - measured_position).norm() - 0.10) < 1.0e-12);
  assert(
    std::abs(
      bounded_target.normalized().dot(original_error.normalized()) - 1.0) <
    1.0e-12);
}

void testZeroAdaptiveShiftRetainsLegacyBound()
{
  const Eigen::Vector2d bounded =
    dog2_mpc::boundIntegratedPositionReference(
    Eigen::Vector2d(0.20, 0.0), Eigen::Vector2d::Zero(),
    Eigen::Vector2d::Zero(), 0.10);

  assert((bounded - Eigen::Vector2d(0.10, 0.0)).norm() < 1.0e-12);
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

Eigen::Matrix<double, 4, 3> measuredStartupFeet()
{
  Eigen::Matrix<double, 4, 3> feet;
  feet <<
    -0.185, -0.131, -0.20,
    0.079, -0.130, -0.20,
    0.079, 0.105, -0.20,
    -0.186, 0.105, -0.20;
  return feet;
}

Eigen::Matrix<double, 4, 3> shiftedFeet(
  const Eigen::Matrix<double, 4, 3> & feet, const Eigen::Vector2d & com_shift)
{
  Eigen::Matrix<double, 4, 3> shifted = feet;
  shifted.col(0).array() -= com_shift.x();
  shifted.col(1).array() -= com_shift.y();
  return shifted;
}

void testNearestSupportShiftKeepsFeasibleUnloadInPlace()
{
  const Eigen::Matrix<double, 4, 3> feet = measuredStartupFeet();
  const double weight = 12.0 * 9.81;
  // Front unloads already satisfy the 15 N gate at the measured startup
  // stance, so the nearest safe point is the current COM itself.
  for (const int swing_leg : {0, 3}) {
    const Eigen::Vector2d shift =
      dog2_mpc::FlatLocomotionMPC::nearestThreeFootSupportShift(
      feet, swing_leg, weight, 15.0);
    assert(shift.norm() < 1.0e-12);
  }
}

void testNearestSupportShiftIsMinimalAndReachesGate()
{
  const Eigen::Matrix<double, 4, 3> feet = measuredStartupFeet();
  const double weight = 12.0 * 9.81;
  for (const int swing_leg : {1, 2}) {
    const Eigen::Vector2d shift =
      dog2_mpc::FlatLocomotionMPC::nearestThreeFootSupportShift(
      feet, swing_leg, weight, 15.0);

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (int leg = 0; leg < 4; ++leg) {
      if (leg != swing_leg) {
        centroid += feet.row(leg).head<2>().transpose();
      }
    }
    centroid /= 3.0;

    assert(shift.norm() > 0.02);
    assert(shift.norm() < centroid.norm() - 0.02);
    assert(
      dog2_mpc::FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
        shiftedFeet(feet, shift), swing_leg, weight, 15.0 - 1.0e-6));
    assert(
      !dog2_mpc::FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
        shiftedFeet(feet, 0.98 * shift), swing_leg, weight, 15.0));
  }
}

void testNearestSupportShiftInfeasibleRequestFallsBackToCentroid()
{
  const Eigen::Matrix<double, 4, 3> feet = measuredStartupFeet();
  const double weight = 12.0 * 9.81;
  const int swing_leg = 1;
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (int leg = 0; leg < 4; ++leg) {
    if (leg != swing_leg) {
      centroid += feet.row(leg).head<2>().transpose();
    }
  }
  centroid /= 3.0;

  const Eigen::Vector2d shift =
    dog2_mpc::FlatLocomotionMPC::nearestThreeFootSupportShift(
    feet, swing_leg, weight, 0.4 * weight);
  assert((shift - centroid).norm() < 1.0e-12);
}

void testAxisSplitKeepsRouteLagReachableInX()
{
  const Eigen::Vector2d measured(0.150, 0.000);
  const Eigen::Vector2d integrated(0.0, 0.0);
  const Eigen::Vector2d support(0.030, 0.000);

  const Eigen::Vector2d shift =
    dog2_mpc::composeAxisSplitPreShiftShift(
    measured, integrated, support, 0.12, 0.06);

  // Body-anchored x: the reference target integrated + shift must reach
  // measured + support even with 0.15 m of route lag.
  assert(std::abs(integrated.x() + shift.x() - (measured.x() + support.x())) <
    1.0e-12);
}

void testAxisSplitCapsLateralExcursionFromRoute()
{
  const Eigen::Vector2d integrated(0.0, 0.0);
  // Body drifted +0.05 m laterally and the support component pushes
  // further out: the world-frame reference offset from the route must
  // still be capped at the lateral envelope.
  const Eigen::Vector2d measured(0.0, 0.050);
  const Eigen::Vector2d outward(0.0, 0.060);

  const Eigen::Vector2d shift =
    dog2_mpc::composeAxisSplitPreShiftShift(
    measured, integrated, outward, 0.12, 0.06);
  assert(std::abs(shift.y() - 0.06) < 1.0e-12);

  // An inward support component still pulls the reference back toward the
  // route instead of following the drifted body.
  const Eigen::Vector2d inward(0.0, -0.060);
  const Eigen::Vector2d pulled =
    dog2_mpc::composeAxisSplitPreShiftShift(
    measured, integrated, inward, 0.12, 0.06);
  assert(std::abs(pulled.y() - (-0.010)) < 1.0e-12);
}

void testAxisSplitNonFiniteInputsReturnZero()
{
  const Eigen::Vector2d bad(std::numeric_limits<double>::quiet_NaN(), 0.0);
  const Eigen::Vector2d good(0.01, 0.02);

  assert(
    dog2_mpc::composeAxisSplitPreShiftShift(
      bad, good, good, 0.12, 0.06).norm() < 1.0e-12);
  assert(
    dog2_mpc::composeAxisSplitPreShiftShift(
      good, good, good, 0.12,
      std::numeric_limits<double>::quiet_NaN()).norm() < 1.0e-12);
}

}  // namespace

int main()
{
  testAdaptiveShiftDoesNotRewriteIntegratedReference();
  testShiftedTargetTrackingErrorRemainsBounded();
  testZeroAdaptiveShiftRetainsLegacyBound();
  testStaticSupport();
  testSwingLegIsUnloaded();
  testShiftedThreeFootSupportBalancesMoment();
  testAttitudeMomentRestoresPitch();
  testFrictionConeAndNoContactFailure();
  testSupportShiftFeedforwardIsApplied();
  testMeasuredStartupSupportRejectsImpossibleHindUnload();
  testNearestSupportShiftKeepsFeasibleUnloadInPlace();
  testNearestSupportShiftIsMinimalAndReachesGate();
  testNearestSupportShiftInfeasibleRequestFallsBackToCentroid();
  testAxisSplitKeepsRouteLagReachableInX();
  testAxisSplitCapsLateralExcursionFromRoute();
  testAxisSplitNonFiniteInputsReturnZero();
  std::cout << "flat locomotion MPC tests passed" << std::endl;
  return 0;
}
