#include "dog2_dynamics/dog2_model.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <stdexcept>
#include <iostream>

namespace dog2_dynamics
{

namespace {

Eigen::Vector4d readRailPositionLimits(
    const pinocchio::Model& model,
    const char* const (&joint_names)[4],
    bool upper) {
    Eigen::Vector4d limits;

    for (int i = 0; i < 4; ++i) {
        const char* joint_name = joint_names[i];
        if (!model.existJointName(joint_name)) {
            throw std::runtime_error(
                std::string("Missing rail joint in Pinocchio model: ") + joint_name);
        }

        const auto joint_id = model.getJointId(joint_name);
        if (model.nqs[joint_id] != 1) {
            throw std::runtime_error(
                std::string("Rail joint is expected to have nq=1: ") + joint_name);
        }

        const int q_index = model.idx_qs[joint_id];
        limits(i) = upper ? model.upperPositionLimit[q_index]
                          : model.lowerPositionLimit[q_index];
    }

    return limits;
}

}  // namespace

Dog2Model::Dog2Model(const std::string & urdf_path)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
    finishInitialization("URDF path: " + urdf_path);
  } catch (const std::exception & e) {
    std::cerr << "✗ Failed to load URDF: " << e.what() << std::endl;
    throw;
  }
}

Dog2Model Dog2Model::fromUrdfXml(const std::string & urdf_xml)
{
  Dog2Model robot;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, robot.model_);
    robot.finishInitialization("URDF XML");
    return robot;
  } catch (const std::exception & e) {
    std::cerr << "✗ Failed to load URDF XML: " << e.what() << std::endl;
    throw;
  }
}

void Dog2Model::finishInitialization(const std::string & source_label)
{
  data_ = pinocchio::Data(model_);
  cacheFrameIds();

  std::cout << "✓ Dog2 Model loaded successfully" << std::endl;
  std::cout << "  Source: " << source_label << std::endl;
  std::cout << "  nq = " << model_.nq << ", nv = " << model_.nv << std::endl;
}

void Dog2Model::cacheFrameIds()
{
  foot_frame_ids_.clear();
  for (const auto & foot_name : FOOT_NAMES) {
    if (model_.existFrame(foot_name)) {
      foot_frame_ids_.push_back(model_.getFrameId(foot_name));
    } else {
      throw std::runtime_error(
              std::string("Foot frame '") + foot_name +
              "' not found in URDF. All foot frames are required for dynamics.");
    }
  }
}

double Dog2Model::mass() const
{
  double total_mass = 0.0;
  for (const auto & inertia : model_.inertias) {
    total_mass += inertia.mass();
  }
  return total_mass;
}

void Dog2Model::forwardKinematics(const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

void Dog2Model::forwardKinematics(const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
}

void Dog2Model::forwardKinematics(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::forwardKinematics(model_, data_, q, v, a);
  pinocchio::updateFramePlacements(model_, data_);
}

Eigen::Vector3d Dog2Model::centerOfMass(const Eigen::VectorXd & q)
{
  // pinocchio::centerOfMass only sweeps the articulated joints (1..N).
  // In this fixed-base model the trunk is welded to the universe and its
  // inertia lives in model_.inertias[0] -- half the robot mass. Skipping
  // it put the reported COM ~5 cm too far toward the hind feet and made
  // every consumer (MPC vertical force split, stance margins) allocate
  // support forces backwards. Merge the root inertia explicitly.
  const Eigen::Vector3d com_articulated = pinocchio::centerOfMass(model_, data_, q);
  const auto & root_inertia = model_.inertias[0];
  const double root_mass = root_inertia.mass();
  if (root_mass <= 0.0) {
    return com_articulated;
  }
  const double articulated_mass = pinocchio::computeTotalMass(model_);
  const double total_mass = articulated_mass + root_mass;
  return (articulated_mass * com_articulated + root_mass * root_inertia.lever()) /
         total_mass;
}

Eigen::Vector3d Dog2Model::centerOfMassVelocity(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v)
{
  pinocchio::centerOfMass(model_, data_, q, v);
  // The trunk (root inertia) is stationary in the base frame, so the
  // articulated-COM velocity must be scaled by its mass share.
  const double root_mass = model_.inertias[0].mass();
  if (root_mass <= 0.0) {
    return data_.vcom[0];
  }
  const double articulated_mass = pinocchio::computeTotalMass(model_);
  return data_.vcom[0] * (articulated_mass / (articulated_mass + root_mass));
}

Eigen::Vector3d Dog2Model::footPosition(const std::string & foot_name, const Eigen::VectorXd & q)
{
  forwardKinematics(q);

  if (!model_.existFrame(foot_name)) {
    throw std::runtime_error(
            std::string("Foot frame '") + foot_name +
            "' not found in URDF. Returning a zero vector would hide a model/URDF mismatch.");
  }

  auto frame_id = model_.getFrameId(foot_name);
  return data_.oMf[frame_id].translation();
}

std::vector<Eigen::Vector3d> Dog2Model::allFootPositions(const Eigen::VectorXd & q)
{
  forwardKinematics(q);

  std::vector<Eigen::Vector3d> positions;
  for (auto frame_id : foot_frame_ids_) {
    positions.push_back(data_.oMf[frame_id].translation());
  }
  return positions;
}

Eigen::MatrixXd Dog2Model::footJacobian(const std::string & foot_name, const Eigen::VectorXd & q)
{
  if (!model_.existFrame(foot_name)) {
    throw std::runtime_error(
            std::string("Foot frame '") + foot_name +
            "' not found in URDF. Returning a zero Jacobian would hide a model/URDF mismatch.");
  }

  auto frame_id = model_.getFrameId(foot_name);
  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
  return J;
}

Eigen::MatrixXd Dog2Model::comJacobian(const Eigen::VectorXd & q)
{
  pinocchio::jacobianCenterOfMass(model_, data_, q);
  return data_.Jcom;
}

Eigen::MatrixXd Dog2Model::massMatrix(const Eigen::VectorXd & q)
{
  pinocchio::crba(model_, data_, q);
  // 对称化质量矩阵（CRBA只计算上三角）
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}

Eigen::VectorXd Dog2Model::nonlinearEffects(const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  return pinocchio::nonLinearEffects(model_, data_, q, v);
}

Eigen::VectorXd Dog2Model::gravityVector(const Eigen::VectorXd & q)
{
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
  return pinocchio::rnea(model_, data_, q, v, a);
}

Eigen::VectorXd Dog2Model::gravityVector(
  const Eigen::VectorXd & q,
  const Eigen::Vector3d & gravity_linear)
{
  const auto saved_gravity = model_.gravity;
  model_.gravity.linear(gravity_linear);
  model_.gravity.angular(Eigen::Vector3d::Zero());
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd g = pinocchio::rnea(model_, data_, q, v, a);
  model_.gravity = saved_gravity;
  return g;
}

Dog2Model::SlidingJointState Dog2Model::getSlidingJointState(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v) const
{

  SlidingJointState state;

  // Dog2 的前4个位置/速度自由度是滑动副。
  // 当前 URDF 经 Pinocchio 构建后 nq=16, nv=16；不要按旧 SO(2) 注释推断索引。
  // 滑动副在 q/v 的前4维，顺序为 [lf, lh, rh, rf]。
  state.positions = q.head<4>();
  state.velocities = v.head<4>();
  state.forces.setZero();    // 力需要从逆动力学计算

  return state;
}

Eigen::Vector4d Dog2Model::slidingJointLowerLimits() const
{
  return readRailPositionLimits(model_, RAIL_JOINT_NAMES, false);
}

Eigen::Vector4d Dog2Model::slidingJointUpperLimits() const
{
  return readRailPositionLimits(model_, RAIL_JOINT_NAMES, true);
}

} // namespace dog2_dynamics
