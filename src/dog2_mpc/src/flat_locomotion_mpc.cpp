#include "dog2_mpc/flat_locomotion_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace dog2_mpc
{

namespace
{

Eigen::Matrix3d skew(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d result;
  result <<
    0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return result;
}

double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

}  // namespace

Eigen::Vector2d boundIntegratedPositionReference(
  const Eigen::Vector2d & integrated_reference,
  const Eigen::Vector2d & adaptive_shift,
  const Eigen::Vector2d & measured_position,
  double maximum_tracking_error)
{
  if (!integrated_reference.allFinite() || !adaptive_shift.allFinite() ||
    !measured_position.allFinite() || !std::isfinite(maximum_tracking_error))
  {
    return integrated_reference;
  }

  const double bounded_maximum = std::max(0.0, maximum_tracking_error);
  Eigen::Vector2d tracking_error =
    integrated_reference + adaptive_shift - measured_position;
  const double error_norm = tracking_error.norm();
  if (error_norm <= bounded_maximum || error_norm <= 1.0e-12) {
    return integrated_reference;
  }

  tracking_error *= bounded_maximum / error_norm;
  return measured_position - adaptive_shift + tracking_error;
}

// Axis-split anchoring for the pre-shift target. x is body-anchored so
// accumulated route-tracking lag cannot make the support target unreachable
// (the acceptance corridor does not constrain the route direction). y is
// anchored to the integrated route reference with a hard world-frame cap so
// the reference cannot follow lateral body drift: chasing that drift was
// observed to feed a tilt/drift spiral out of the acceptance corridor.
Eigen::Vector2d composeAxisSplitPreShiftShift(
  const Eigen::Vector2d & measured_position,
  const Eigen::Vector2d & integrated_reference,
  const Eigen::Vector2d & support_component,
  double maximum_forward_offset,
  double maximum_lateral_offset)
{
  if (!measured_position.allFinite() || !integrated_reference.allFinite() ||
    !support_component.allFinite() || !std::isfinite(maximum_forward_offset) ||
    !std::isfinite(maximum_lateral_offset))
  {
    return Eigen::Vector2d::Zero();
  }

  const Eigen::Vector2d route_anchor = measured_position - integrated_reference;
  const double forward_bound = std::max(0.0, maximum_forward_offset);
  const double lateral_bound = std::max(0.0, maximum_lateral_offset);
  Eigen::Vector2d result;
  result.x() = route_anchor.x() +
    clampValue(support_component.x(), -forward_bound, forward_bound);
  result.y() = clampValue(
    route_anchor.y() + support_component.y(), -lateral_bound, lateral_bound);
  return result;
}

Eigen::Vector2d FlatLocomotionMPC::nearestThreeFootSupportShift(
  const Eigen::Matrix<double, 4, 3> & feet_relative_world,
  int swing_leg,
  double total_weight,
  double minimum_normal_force)
{
  if (!feet_relative_world.allFinite() || swing_leg < 0 || swing_leg >= 4 ||
    !std::isfinite(total_weight) || total_weight <= 0.0 ||
    !std::isfinite(minimum_normal_force))
  {
    return Eigen::Vector2d::Zero();
  }

  std::array<Eigen::Vector2d, 3> stance;
  int count = 0;
  for (int leg = 0; leg < 4; ++leg) {
    if (leg == swing_leg) {
      continue;
    }
    stance[count] = feet_relative_world.row(leg).head<2>().transpose();
    ++count;
  }
  const Eigen::Vector2d centroid =
    (stance[0] + stance[1] + stance[2]) / 3.0;

  const auto cross2 =
    [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      return a.x() * b.y() - a.y() * b.x();
    };
  const double doubled_area =
    cross2(stance[1] - stance[0], stance[2] - stance[0]);
  const double load_fraction =
    std::max(0.0, minimum_normal_force) / total_weight;
  // The centroid is the equal-load point and therefore maximises the
  // minimum normal force; it is the best effort answer whenever the request
  // cannot be met strictly inside the triangle.
  if (std::abs(doubled_area) < 1.0e-9 || load_fraction >= 1.0 / 3.0) {
    return centroid;
  }

  // Feet are expressed relative to the current COM, so the static normal
  // force of stance foot i equals total_weight times the barycentric
  // coordinate lambda_i of the COM target inside the stance triangle. The
  // feasible set {lambda_i >= load_fraction for all i} is the stance
  // triangle shrunk toward its centroid by that barycentric margin.
  std::array<Eigen::Vector2d, 3> shrunk;
  for (int vertex = 0; vertex < 3; ++vertex) {
    shrunk[vertex] = (1.0 - 3.0 * load_fraction) * stance[vertex] +
      3.0 * load_fraction * centroid;
  }

  const double orientation = doubled_area > 0.0 ? 1.0 : -1.0;
  bool inside = true;
  for (int edge = 0; edge < 3; ++edge) {
    const Eigen::Vector2d & a = shrunk[edge];
    const Eigen::Vector2d & b = shrunk[(edge + 1) % 3];
    if (orientation * cross2(b - a, -a) < 0.0) {
      inside = false;
      break;
    }
  }
  if (inside) {
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d best = shrunk[0];
  double best_norm = best.norm();
  for (int edge = 0; edge < 3; ++edge) {
    const Eigen::Vector2d & a = shrunk[edge];
    const Eigen::Vector2d segment = shrunk[(edge + 1) % 3] - a;
    const double length_squared = segment.squaredNorm();
    double along = 0.0;
    if (length_squared > 1.0e-12) {
      along = clampValue(-a.dot(segment) / length_squared, 0.0, 1.0);
    }
    const Eigen::Vector2d candidate = a + along * segment;
    const double candidate_norm = candidate.norm();
    if (candidate_norm < best_norm) {
      best = candidate;
      best_norm = candidate_norm;
    }
  }
  return best;
}

FlatLocomotionMPC::FlatLocomotionMPC(
  double mass,
  const Eigen::Matrix3d & inertia)
: FlatLocomotionMPC(mass, inertia, Parameters())
{
}

FlatLocomotionMPC::FlatLocomotionMPC(
  double mass,
  const Eigen::Matrix3d & inertia,
  const Parameters & parameters)
: mass_(mass),
  inertia_(inertia),
  parameters_(parameters),
  allocator_(std::make_unique<OSQPInterface>())
{
  if (!(mass_ > 0.0) || !std::isfinite(mass_)) {
    throw std::invalid_argument("FlatLocomotionMPC mass must be positive and finite");
  }
  if (!inertia_.allFinite() || inertia_.determinant() <= 1.0e-9) {
    throw std::invalid_argument("FlatLocomotionMPC inertia must be finite positive definite");
  }
  parameters_.horizon = std::max(1, parameters_.horizon);
  parameters_.dt = std::max(1.0e-3, parameters_.dt);
  parameters_.friction_coefficient =
    std::max(0.05, parameters_.friction_coefficient);
  parameters_.min_stance_force =
    std::max(0.0, parameters_.min_stance_force);
  parameters_.max_stance_force =
    std::max(parameters_.min_stance_force, parameters_.max_stance_force);
  parameters_.force_regularization =
    std::max(1.0e-8, parameters_.force_regularization);
}

bool FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
  const Eigen::Matrix<double, 4, 3> & feet_relative_world,
  int swing_leg,
  double total_weight,
  double minimum_normal_force,
  Eigen::Vector3d * normal_forces)
{
  if (!feet_relative_world.allFinite() || swing_leg < 0 || swing_leg >= 4 ||
    !std::isfinite(total_weight) || total_weight <= 0.0 ||
    !std::isfinite(minimum_normal_force))
  {
    return false;
  }

  Eigen::Matrix3d equilibrium;
  int column = 0;
  for (int leg = 0; leg < 4; ++leg) {
    if (leg == swing_leg) {
      continue;
    }
    equilibrium(0, column) = 1.0;
    equilibrium(1, column) = feet_relative_world(leg, 1);
    equilibrium(2, column) = -feet_relative_world(leg, 0);
    ++column;
  }

  Eigen::FullPivLU<Eigen::Matrix3d> decomposition(equilibrium);
  if (decomposition.rank() < 3) {
    return false;
  }
  const Eigen::Vector3d load(total_weight, 0.0, 0.0);
  const Eigen::Vector3d forces = decomposition.solve(load);
  if (normal_forces != nullptr) {
    *normal_forces = forces;
  }
  const double residual = (equilibrium * forces - load).norm();
  return forces.allFinite() &&
         residual <= 1.0e-6 * std::max(1.0, total_weight) &&
         forces.minCoeff() >= std::max(0.0, minimum_normal_force);
}

double FlatLocomotionMPC::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Matrix<double, 6, 12> FlatLocomotionMPC::finiteHorizonGain() const
{
  Eigen::Matrix<double, 12, 12> A =
    Eigen::Matrix<double, 12, 12>::Identity();
  A.block<3, 3>(0, 6) =
    parameters_.dt * Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 9) =
    parameters_.dt * Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 12, 6> B =
    Eigen::Matrix<double, 12, 6>::Zero();
  B.block<3, 3>(6, 0) =
    (parameters_.dt / mass_) * Eigen::Matrix3d::Identity();
  B.block<3, 3>(9, 3) = parameters_.dt * inertia_.inverse();

  Eigen::Matrix<double, 12, 12> Q =
    parameters_.state_weights.asDiagonal();
  Eigen::Matrix<double, 6, 6> R =
    parameters_.wrench_weights.asDiagonal();
  Eigen::Matrix<double, 12, 12> P = 8.0 * Q;
  Eigen::Matrix<double, 6, 12> gain =
    Eigen::Matrix<double, 6, 12>::Zero();

  for (int step = 0; step < parameters_.horizon; ++step) {
    const Eigen::Matrix<double, 6, 6> S = R + B.transpose() * P * B;
    gain = S.ldlt().solve(B.transpose() * P * A);
    P = Q + A.transpose() * P * (A - B * gain);
    P = 0.5 * (P + P.transpose());
  }
  return gain;
}

Eigen::Matrix<double, 6, 1> FlatLocomotionMPC::desiredWrench(
  const FlatBodyState & state,
  const FlatBodyReference & reference) const
{
  Eigen::Matrix<double, 12, 1> error;
  error.segment<3>(0) = state.position - reference.position;
  error.segment<3>(3) = state.rpy - reference.rpy;
  for (int axis = 0; axis < 3; ++axis) {
    error(3 + axis) = wrapAngle(error(3 + axis));
  }
  error.segment<3>(6) =
    state.linear_velocity - reference.linear_velocity;
  error.segment<3>(9) =
    state.angular_velocity - reference.angular_velocity;

  Eigen::Matrix<double, 6, 1> wrench = -finiteHorizonGain() * error;
  wrench.z() += mass_ * 9.81;
  wrench += reference.wrench_feedforward;

  wrench.x() = clampValue(
    wrench.x(), -parameters_.max_force.x(), parameters_.max_force.x());
  wrench.y() = clampValue(
    wrench.y(), -parameters_.max_force.y(), parameters_.max_force.y());
  wrench.z() = clampValue(
    wrench.z(), 0.40 * mass_ * 9.81, parameters_.max_force.z());
  for (int axis = 0; axis < 3; ++axis) {
    wrench(3 + axis) = clampValue(
      wrench(3 + axis),
      -parameters_.max_torque(axis),
      parameters_.max_torque(axis));
  }
  return wrench;
}

bool FlatLocomotionMPC::allocateWrench(
  const Eigen::Matrix<double, 6, 1> & desired_wrench,
  const Eigen::Matrix<double, 4, 3> & feet_relative_world,
  const std::array<bool, 4> & contacts,
  FlatMPCSolution & solution)
{
  if (!desired_wrench.allFinite() || !feet_relative_world.allFinite()) {
    return false;
  }
  if (std::none_of(contacts.begin(), contacts.end(), [](bool value) {return value;})) {
    return false;
  }

  Eigen::Matrix<double, 6, 12> wrench_map =
    Eigen::Matrix<double, 6, 12>::Zero();
  for (int leg = 0; leg < 4; ++leg) {
    wrench_map.block<3, 3>(0, leg * 3) = Eigen::Matrix3d::Identity();
    wrench_map.block<3, 3>(3, leg * 3) =
      skew(feet_relative_world.row(leg).transpose());
    if (!contacts[leg]) {
      previous_forces_.segment<3>(leg * 3).setZero();
    }
  }

  const Eigen::Matrix<double, 6, 6> wrench_weight =
    parameters_.allocation_weights.asDiagonal();
  Eigen::Matrix<double, 12, 12> hessian =
    2.0 * (
    wrench_map.transpose() * wrench_weight * wrench_map +
    parameters_.force_regularization *
    Eigen::Matrix<double, 12, 12>::Identity());
  Eigen::Matrix<double, 12, 1> gradient =
    -2.0 * wrench_map.transpose() * wrench_weight * desired_wrench -
    2.0 * parameters_.force_regularization * previous_forces_;

  constexpr int constraints_per_leg = 5;
  Eigen::Matrix<double, 20, 12> constraint_matrix =
    Eigen::Matrix<double, 20, 12>::Zero();
  Eigen::Matrix<double, 20, 1> lower;
  Eigen::Matrix<double, 20, 1> upper;
  const double infinity = static_cast<double>(OSQP_INFTY);
  const double mu = parameters_.friction_coefficient;

  for (int leg = 0; leg < 4; ++leg) {
    const int row = constraints_per_leg * leg;
    const int col = 3 * leg;
    constraint_matrix(row, col + 2) = 1.0;
    lower(row) = contacts[leg] ? parameters_.min_stance_force : 0.0;
    upper(row) = contacts[leg] ? parameters_.max_stance_force : 0.0;

    constraint_matrix(row + 1, col) = 1.0;
    constraint_matrix(row + 1, col + 2) = -mu;
    lower(row + 1) = -infinity;
    upper(row + 1) = 0.0;

    constraint_matrix(row + 2, col) = -1.0;
    constraint_matrix(row + 2, col + 2) = -mu;
    lower(row + 2) = -infinity;
    upper(row + 2) = 0.0;

    constraint_matrix(row + 3, col + 1) = 1.0;
    constraint_matrix(row + 3, col + 2) = -mu;
    lower(row + 3) = -infinity;
    upper(row + 3) = 0.0;

    constraint_matrix(row + 4, col + 1) = -1.0;
    constraint_matrix(row + 4, col + 2) = -mu;
    lower(row + 4) = -infinity;
    upper(row + 4) = 0.0;
  }

  std::vector<Eigen::Triplet<double>> hessian_triplets;
  for (int col = 0; col < hessian.cols(); ++col) {
    for (int row = 0; row <= col; ++row) {
      if (std::abs(hessian(row, col)) > 1.0e-12) {
        hessian_triplets.emplace_back(row, col, hessian(row, col));
      }
    }
  }
  Eigen::SparseMatrix<double> hessian_sparse(12, 12);
  hessian_sparse.setFromTriplets(
    hessian_triplets.begin(), hessian_triplets.end());
  const Eigen::SparseMatrix<double> constraint_sparse =
    constraint_matrix.sparseView();

  if (!allocator_->setup(
      hessian_sparse, gradient, constraint_sparse, lower, upper))
  {
    return false;
  }
  Eigen::VectorXd allocated;
  if (!allocator_->solve(allocated) || allocated.size() != 12 ||
    !allocated.allFinite())
  {
    return false;
  }

  solution.foot_forces = allocated;
  solution.desired_wrench = desired_wrench;
  solution.achieved_wrench = wrench_map * solution.foot_forces;
  solution.wrench_residual =
    (wrench_weight *
    (solution.achieved_wrench - solution.desired_wrench)).norm();
  previous_forces_ = solution.foot_forces;
  return true;
}

bool FlatLocomotionMPC::solve(
  const FlatBodyState & state,
  const FlatBodyReference & reference,
  const Eigen::Matrix<double, 4, 3> & feet_relative_world,
  const std::array<bool, 4> & contacts,
  FlatMPCSolution & solution)
{
  const auto state_is_finite =
    state.position.allFinite() && state.rpy.allFinite() &&
    state.linear_velocity.allFinite() && state.angular_velocity.allFinite();
  const auto reference_is_finite =
    reference.position.allFinite() && reference.rpy.allFinite() &&
    reference.linear_velocity.allFinite() &&
    reference.angular_velocity.allFinite() &&
    reference.wrench_feedforward.allFinite();
  if (!state_is_finite || !reference_is_finite) {
    return false;
  }
  return allocateWrench(
    desiredWrench(state, reference), feet_relative_world, contacts, solution);
}

void FlatLocomotionMPC::reset()
{
  previous_forces_.setZero();
  allocator_->cleanup();
}

}  // namespace dog2_mpc
