#include "dog2_mpc/flat_locomotion_mpc.hpp"

#include <dog2_interfaces/msg/contact_phase.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "dog2_dynamics/dog2_model.hpp"

namespace dog2_mpc
{

namespace
{

constexpr std::array<const char *, 4> kLegNames{"lf", "lh", "rh", "rf"};
constexpr std::array<const char *, 4> kFootFrames{
  "lf_foot_link", "lh_foot_link", "rh_foot_link", "rf_foot_link"};
constexpr std::array<const char *, 4> kJointSuffixes{
  "rail_joint", "coxa_joint", "femur_joint", "tibia_joint"};

double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double wallNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Quaterniond q = quaternion.normalized();
  const double sin_roll = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cos_roll = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double sin_pitch = clampValue(
    2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
  const double sin_yaw = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cos_yaw = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return Eigen::Vector3d(
    std::atan2(sin_roll, cos_roll),
    std::asin(sin_pitch),
    std::atan2(sin_yaw, cos_yaw));
}

}  // namespace

class FlatMPCNode : public rclcpp::Node
{
public:
  FlatMPCNode()
  : Node("flat_mpc_node")
  {
    declareParameters();
    loadModel();
    createInterfaces();
    const double frequency = get_parameter("control_frequency").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, frequency)),
      std::bind(&FlatMPCNode::controlLoop, this));
    RCLCPP_INFO(
      get_logger(),
      "Flat MPC ready: mass=%.3f kg, frequency=%.1f Hz, height=%.3f m",
      mass_, frequency, target_height_);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("contact_phase_topic", "/dog2/gait/contact_phase");
    declare_parameter<std::string>("body_shift_topic", "/dog2/gait/body_shift");
    declare_parameter<std::string>(
      "body_shift_error_topic", "/dog2/mpc/body_shift_error");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter("control_frequency", 50.0);
    declare_parameter("target_height", 0.20);
    declare_parameter("state_freshness_sec", 0.20);
    declare_parameter("command_timeout_sec", 0.30);
    declare_parameter("velocity_filter_tau", 0.10);
    declare_parameter("max_linear_speed", 0.12);
    declare_parameter("max_angular_speed", 0.30);
    declare_parameter("max_linear_acceleration", 0.20);
    declare_parameter("max_angular_acceleration", 0.50);
    declare_parameter("max_position_error", 0.10);
    declare_parameter("max_yaw_error", 0.35);
    declare_parameter("body_shift_kp", 800.0);
    declare_parameter("body_shift_kd", 80.0);
    declare_parameter("body_shift_max_linear_speed", 0.025);
    declare_parameter("body_shift_max_angular_speed", 0.15);
    declare_parameter("body_shift_max_tilt", 0.12);
    declare_parameter("support_ready_min_normal_force", 2.0);
    declare_parameter("support_target_force_margin", 25.0);
    declare_parameter("support_center_gain", 1.0);
    declare_parameter("support_center_max_speed", 0.10);
    declare_parameter("support_center_max_forward_offset", 0.08);
    declare_parameter("support_center_max_lateral_offset", 0.04);
    declare_parameter("horizon", 20);
    declare_parameter("mpc_dt", 0.02);
    declare_parameter("friction_coefficient", 0.65);
    declare_parameter("min_stance_force", 2.0);
    declare_parameter("max_stance_force", 95.0);
    declare_parameter("force_regularization", 0.002);
    declare_parameter(
      "body_inertia_diagonal",
      std::vector<double>{0.13567, 0.25110, 0.32313});
    declare_parameter(
      "state_weights",
      std::vector<double>{
        12.0, 20.0, 160.0,
        220.0, 260.0, 40.0,
        20.0, 35.0, 50.0,
        24.0, 28.0, 8.0});
    declare_parameter(
      "wrench_weights",
      std::vector<double>{0.08, 0.08, 0.025, 0.06, 0.06, 0.08});
    declare_parameter(
      "allocation_weights",
      std::vector<double>{1.0, 1.0, 3.0, 12.0, 12.0, 6.0});
    declare_parameter(
      "max_net_force",
      std::vector<double>{55.0, 40.0, 185.0});
    declare_parameter(
      "max_net_torque",
      std::vector<double>{20.0, 24.0, 10.0});
  }

  template<int Size>
  Eigen::Matrix<double, Size, 1> vectorParameter(
    const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != static_cast<std::size_t>(Size)) {
      throw std::runtime_error(
              name + " must contain " + std::to_string(Size) + " values");
    }
    Eigen::Matrix<double, Size, 1> result;
    for (int index = 0; index < Size; ++index) {
      result(index) = values[index];
    }
    return result;
  }

  void loadModel()
  {
    const std::string description =
      get_parameter("robot_description").as_string();
    if (description.empty()) {
      throw std::runtime_error("flat_mpc_node requires robot_description");
    }
    model_ = std::make_unique<dog2_dynamics::Dog2Model>(
      dog2_dynamics::Dog2Model::fromUrdfXml(description));
    mass_ = model_->mass();
    current_q_ = Eigen::VectorXd::Zero(model_->nq());

    const auto & pin_model = model_->getModel();
    for (int leg = 0; leg < 4; ++leg) {
      for (const char * suffix : kJointSuffixes) {
        const std::string name =
          std::string(kLegNames[leg]) + "_" + suffix;
        if (!pin_model.existJointName(name)) {
          throw std::runtime_error("flat MPC model missing joint " + name);
        }
        const auto joint_id = pin_model.getJointId(name);
        joint_q_index_[name] = pin_model.idx_qs[joint_id];
        joint_seen_[name] = false;
      }
    }

    const Eigen::Vector3d inertia_diagonal =
      vectorParameter<3>("body_inertia_diagonal");
    FlatLocomotionMPC::Parameters parameters;
    parameters.horizon = get_parameter("horizon").as_int();
    parameters.dt = get_parameter("mpc_dt").as_double();
    parameters.friction_coefficient =
      get_parameter("friction_coefficient").as_double();
    parameters.min_stance_force =
      get_parameter("min_stance_force").as_double();
    parameters.max_stance_force =
      get_parameter("max_stance_force").as_double();
    parameters.force_regularization =
      get_parameter("force_regularization").as_double();
    parameters.state_weights = vectorParameter<12>("state_weights");
    parameters.wrench_weights = vectorParameter<6>("wrench_weights");
    parameters.allocation_weights = vectorParameter<6>("allocation_weights");
    parameters.max_force = vectorParameter<3>("max_net_force");
    parameters.max_torque = vectorParameter<3>("max_net_torque");
    controller_ = std::make_unique<FlatLocomotionMPC>(
      mass_, inertia_diagonal.asDiagonal(), parameters);

    target_height_ = get_parameter("target_height").as_double();
    state_freshness_sec_ = get_parameter("state_freshness_sec").as_double();
    command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
    velocity_filter_tau_ = get_parameter("velocity_filter_tau").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    max_linear_acceleration_ =
      get_parameter("max_linear_acceleration").as_double();
    max_angular_acceleration_ =
      get_parameter("max_angular_acceleration").as_double();
    max_position_error_ = get_parameter("max_position_error").as_double();
    max_yaw_error_ = get_parameter("max_yaw_error").as_double();
    body_shift_kp_ = get_parameter("body_shift_kp").as_double();
    body_shift_kd_ = get_parameter("body_shift_kd").as_double();
    body_shift_max_linear_speed_ =
      get_parameter("body_shift_max_linear_speed").as_double();
    body_shift_max_angular_speed_ =
      get_parameter("body_shift_max_angular_speed").as_double();
    body_shift_max_tilt_ = get_parameter("body_shift_max_tilt").as_double();
    support_ready_min_normal_force_ =
      get_parameter("support_ready_min_normal_force").as_double();
    support_target_force_margin_ = std::max(
      0.0, get_parameter("support_target_force_margin").as_double());
    support_center_gain_ =
      get_parameter("support_center_gain").as_double();
    support_center_max_speed_ =
      get_parameter("support_center_max_speed").as_double();
    support_center_max_forward_offset_ =
      get_parameter("support_center_max_forward_offset").as_double();
    support_center_max_lateral_offset_ =
      get_parameter("support_center_max_lateral_offset").as_double();
    control_dt_ =
      1.0 / std::max(1.0, get_parameter("control_frequency").as_double());
  }

  void createInterfaces()
  {
    force_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/dog2/mpc/foot_forces", 10);
    shift_error_publisher_ =
      create_publisher<std_msgs::msg::Float64>(
      get_parameter("body_shift_error_topic").as_string(), 10);
    state_publisher_ =
      create_publisher<std_msgs::msg::String>("/dog2/mpc/crossing_state", 10);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 20,
      std::bind(&FlatMPCNode::odomCallback, this, std::placeholders::_1));
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&FlatMPCNode::jointCallback, this, std::placeholders::_1));
    contact_subscription_ =
      create_subscription<dog2_interfaces::msg::ContactPhase>(
      get_parameter("contact_phase_topic").as_string(), 20,
      std::bind(&FlatMPCNode::contactCallback, this, std::placeholders::_1));
    shift_subscription_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
      get_parameter("body_shift_topic").as_string(), 20,
      std::bind(&FlatMPCNode::shiftCallback, this, std::placeholders::_1));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 20,
      std::bind(&FlatMPCNode::commandCallback, this, std::placeholders::_1));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & pose = message->pose.pose;
    Eigen::Vector3d position(
      pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!position.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 1.0e-6)
    {
      return;
    }
    orientation.normalize();

    double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (!(stamp > 0.0)) {
      stamp = now().seconds();
    }
    if (odom_received_) {
      const double dt = stamp - last_odom_stamp_;
      if (dt > 1.0e-4 && dt < 0.25) {
        const Eigen::Vector3d raw_linear =
          (position - body_state_.position) / dt;
        Eigen::Quaterniond delta = orientation * body_orientation_.conjugate();
        if (delta.w() < 0.0) {
          delta.coeffs() *= -1.0;
        }
        const Eigen::AngleAxisd angle_axis(delta.normalized());
        const Eigen::Vector3d raw_angular =
          angle_axis.axis() * angle_axis.angle() / dt;
        const double alpha = dt / (std::max(0.0, velocity_filter_tau_) + dt);
        body_state_.linear_velocity +=
          alpha * (raw_linear - body_state_.linear_velocity);
        body_state_.angular_velocity +=
          alpha * (raw_angular - body_state_.angular_velocity);
      }
    }
    body_state_.position = position;
    body_state_.rpy = quaternionToRpy(orientation);
    body_orientation_ = orientation;
    last_odom_stamp_ = stamp;
    last_odom_receive_sec_ = wallNowSec();
    odom_received_ = true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const std::size_t count =
      std::min(message->name.size(), message->position.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto found = joint_q_index_.find(message->name[index]);
      if (found == joint_q_index_.end() ||
        !std::isfinite(message->position[index]))
      {
        continue;
      }
      current_q_(found->second) = message->position[index];
      joint_seen_[message->name[index]] = true;
    }
    joints_received_ = std::all_of(
      joint_seen_.begin(), joint_seen_.end(),
      [](const auto & item) {return item.second;});
    last_joint_receive_sec_ = wallNowSec();
  }

  void contactCallback(
    const dog2_interfaces::msg::ContactPhase::SharedPtr message)
  {
    const std::array<bool, 4> previous_contacts = contacts_;
    const std::size_t count =
      std::min(message->leg_names.size(), message->phase.size());
    for (std::size_t index = 0; index < count; ++index) {
      for (int leg = 0; leg < 4; ++leg) {
        if (message->leg_names[index] == kLegNames[leg]) {
          contacts_[leg] =
            message->phase[index] !=
            dog2_interfaces::msg::ContactPhase::SWING;
        }
      }
    }
    if (contacts_ != previous_contacts) {
      contact_transition_pending_ = true;
      RCLCPP_WARN(
        get_logger(), "flat MPC contact transition [%d%d%d%d] -> [%d%d%d%d]",
        previous_contacts[0], previous_contacts[1],
        previous_contacts[2], previous_contacts[3],
        contacts_[0], contacts_[1], contacts_[2], contacts_[3]);
    }
    contact_received_ = true;
    last_contact_receive_sec_ = wallNowSec();
  }

  void shiftCallback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    requested_body_shift_ <<
      message->vector.x, message->vector.y, message->vector.z;
    if (!requested_body_shift_.allFinite()) {
      requested_body_shift_.setZero();
    }
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    target_command_ <<
      clampValue(message->linear.x, -max_linear_speed_, max_linear_speed_),
      clampValue(message->linear.y, -max_linear_speed_, max_linear_speed_),
      clampValue(message->angular.z, -max_angular_speed_, max_angular_speed_);
    last_command_receive_sec_ = wallNowSec();
  }

  bool dataFresh() const
  {
    const double current = wallNowSec();
    return odom_received_ && joints_received_ && contact_received_ &&
           current - last_odom_receive_sec_ <= state_freshness_sec_ &&
           current - last_joint_receive_sec_ <= state_freshness_sec_ &&
           current - last_contact_receive_sec_ <= state_freshness_sec_;
  }

  void updateReference()
  {
    if (wallNowSec() - last_command_receive_sec_ > command_timeout_sec_) {
      target_command_.setZero();
    }
    for (int axis = 0; axis < 2; ++axis) {
      const double step = max_linear_acceleration_ * control_dt_;
      slewed_command_(axis) += clampValue(
        target_command_(axis) - slewed_command_(axis), -step, step);
    }
    const double yaw_step = max_angular_acceleration_ * control_dt_;
    slewed_command_(2) += clampValue(
      target_command_(2) - slewed_command_(2), -yaw_step, yaw_step);

    const bool moving =
      slewed_command_.head<2>().norm() > 1.0e-3 ||
      std::abs(slewed_command_(2)) > 1.0e-3;
    if (!reference_initialized_ || moving != was_moving_ ||
      (!moving && !walking_started_))
    {
      integrated_position_reference_ = body_state_.position;
      integrated_yaw_reference_ = body_state_.rpy.z();
      adaptive_body_shift_world_.setZero();
      reference_initialized_ = true;
      controller_->reset();
    }
    walking_started_ = walking_started_ || moving;
    was_moving_ = moving;

    const double yaw = body_state_.rpy.z();
    const Eigen::Matrix2d heading_rotation =
      Eigen::Rotation2Dd(yaw).toRotationMatrix();
    const bool all_stance = std::all_of(
      contacts_.begin(), contacts_.end(), [](bool value) {
        return value;
      });
    const int upcoming_swing_leg =
      static_cast<int>(std::lround(requested_body_shift_.z())) - 1;
    const bool valid_upcoming_leg =
      upcoming_swing_leg >= 0 && upcoming_swing_leg < 4;
    const bool pre_shifting =
      moving && all_stance && valid_upcoming_leg;
    const Eigen::Matrix<double, 4, 3> feet_relative_world =
      footLeverArmsWorld();
    Eigen::Vector2d velocity_world =
      heading_rotation * slewed_command_.head<2>();
    if (pre_shifting) {
      velocity_world.setZero();
    }
    integrated_position_reference_.head<2>() +=
      control_dt_ * velocity_world;
    integrated_yaw_reference_ = wrapAngle(
      integrated_yaw_reference_ + control_dt_ * slewed_command_(2));

    const double yaw_error = clampValue(
      wrapAngle(integrated_yaw_reference_ - body_state_.rpy.z()),
      -max_yaw_error_, max_yaw_error_);

    if (pre_shifting) {
      // The target is the nearest COM point whose three-foot static solve
      // reaches gate + margin; the 25 N margin saturates it to the
      // equal-load centroid so the 15 N gate is crossed mid-transit despite
      // the tilt-induced COM shortfall (an integral push was tried instead
      // and over-accumulated against this deep target, tipping the robot).
      const Eigen::Vector2d support_shift =
        FlatLocomotionMPC::nearestThreeFootSupportShift(
        feet_relative_world, upcoming_swing_leg, mass_ * 9.81,
        support_ready_min_normal_force_ + support_target_force_margin_);
      // Axis-split anchoring: x re-anchors to the measured body so route
      // lag cannot make the support target unreachable; y is capped in the
      // world frame around the integrated route reference so the reference
      // cannot chase lateral drift into the acceptance corridor.
      const Eigen::Vector2d desired_shift =
        composeAxisSplitPreShiftShift(
        body_state_.position.head<2>(),
        integrated_position_reference_.head<2>(),
        support_center_gain_ * support_shift,
        support_center_max_forward_offset_,
        support_center_max_lateral_offset_);
      Eigen::Vector2d center_step =
        desired_shift - adaptive_body_shift_world_;
      const double maximum_center_step =
        std::max(0.0, support_center_max_speed_) * control_dt_;
      if (center_step.norm() > maximum_center_step &&
        maximum_center_step > 0.0)
      {
        center_step *= maximum_center_step / center_step.norm();
      }
      adaptive_body_shift_world_ += center_step;
    }
    integrated_position_reference_.head<2>() =
      boundIntegratedPositionReference(
      integrated_position_reference_.head<2>(),
      adaptive_body_shift_world_,
      body_state_.position.head<2>(),
      max_position_error_);
    body_reference_.position = integrated_position_reference_;
    body_reference_.position.head<2>() += adaptive_body_shift_world_;
    body_reference_.position.z() = target_height_;
    body_reference_.rpy <<
      0.0, 0.0, wrapAngle(body_state_.rpy.z() + yaw_error);
    body_reference_.linear_velocity <<
      velocity_world.x(), velocity_world.y(), 0.0;
    body_reference_.angular_velocity << 0.0, 0.0, slewed_command_(2);
    body_reference_.wrench_feedforward.setZero();
    if (pre_shifting) {
      body_reference_.wrench_feedforward.head<2>() =
        body_shift_kp_ * (
        body_reference_.position.head<2>() -
        body_state_.position.head<2>()) -
        body_shift_kd_ * body_state_.linear_velocity.head<2>();
    }

    std_msgs::msg::Float64 shift_error;
    const bool dynamically_settled =
      body_state_.linear_velocity.head<2>().norm() <=
      body_shift_max_linear_speed_ &&
      body_state_.angular_velocity.head<2>().norm() <=
      body_shift_max_angular_speed_ &&
      body_state_.rpy.head<2>().norm() <= body_shift_max_tilt_;
    bool support_ready = true;
    Eigen::Vector3d static_normal_forces = Eigen::Vector3d::Zero();
    if (moving && valid_upcoming_leg) {
      support_ready =
        FlatLocomotionMPC::hasThreeFootStaticSupportMargin(
        feet_relative_world, upcoming_swing_leg, mass_ * 9.81,
        support_ready_min_normal_force_, &static_normal_forces);
      if (!support_ready) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "flat MPC waiting to unload leg=%d: static_fz=[%.2f %.2f %.2f] "
          "speed=%.3f angular=%.3f tilt=%.3f",
          upcoming_swing_leg, static_normal_forces.x(),
          static_normal_forces.y(), static_normal_forces.z(),
          body_state_.linear_velocity.head<2>().norm(),
          body_state_.angular_velocity.head<2>().norm(),
          body_state_.rpy.head<2>().norm());
      }
    }
    if (support_ready && !dynamically_settled && moving) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat MPC waiting for dynamic settle: speed=%.3f angular=%.3f "
        "tilt=%.3f",
        body_state_.linear_velocity.head<2>().norm(),
        body_state_.angular_velocity.head<2>().norm(),
        body_state_.rpy.head<2>().norm());
    }
    shift_error.data = dynamically_settled && support_ready ?
      0.0 : std::numeric_limits<double>::infinity();
    shift_error_publisher_->publish(shift_error);
  }

  Eigen::Matrix<double, 4, 3> footLeverArmsWorld()
  {
    Eigen::Matrix<double, 4, 3> feet;
    const Eigen::Vector3d com_body = model_->centerOfMass(current_q_);
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d foot_body =
        model_->footPosition(kFootFrames[leg], current_q_);
      feet.row(leg) =
        (body_orientation_ * (foot_body - com_body)).transpose();
    }
    return feet;
  }

  void publishForces(const Eigen::Matrix<double, 12, 1> & forces)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data.assign(forces.data(), forces.data() + forces.size());
    force_publisher_->publish(message);
  }

  void publishMode()
  {
    std_msgs::msg::String mode;
    mode.data = was_moving_ ? "FLAT_WALKING" : "HOVER";
    state_publisher_->publish(mode);
  }

  void controlLoop()
  {
    if (!dataFresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat MPC waiting for fresh odom, joints and contact phase");
      return;
    }
    updateReference();

    FlatMPCSolution solution;
    const Eigen::Matrix<double, 4, 3> feet = footLeverArmsWorld();
    if (!controller_->solve(
        body_state_, body_reference_, feet, contacts_, solution))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat MPC force allocation failed");
      return;
    }
    publishForces(solution.foot_forces);
    if (contact_transition_pending_) {
      RCLCPP_WARN(
        get_logger(),
        "flat MPC transition forces=[%.2f %.2f %.2f | %.2f %.2f %.2f | "
        "%.2f %.2f %.2f | %.2f %.2f %.2f] achieved_moment=[%.3f %.3f %.3f] "
        "desired_moment=[%.3f %.3f %.3f] residual=%.3f "
        "feet_xy=[%.3f %.3f | %.3f %.3f | %.3f %.3f | %.3f %.3f]",
        solution.foot_forces(0), solution.foot_forces(1),
        solution.foot_forces(2), solution.foot_forces(3),
        solution.foot_forces(4), solution.foot_forces(5),
        solution.foot_forces(6), solution.foot_forces(7),
        solution.foot_forces(8), solution.foot_forces(9),
        solution.foot_forces(10), solution.foot_forces(11),
        solution.achieved_wrench(3), solution.achieved_wrench(4),
        solution.achieved_wrench(5), solution.desired_wrench(3),
        solution.desired_wrench(4), solution.desired_wrench(5),
        solution.wrench_residual,
        feet(0, 0), feet(0, 1), feet(1, 0), feet(1, 1),
        feet(2, 0), feet(2, 1), feet(3, 0), feet(3, 1));
      contact_transition_pending_ = false;
    }
    publishMode();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "flat MPC mode=%s p=[%.3f %.3f %.3f] ref=[%.3f %.3f %.3f] "
      "global=[%.3f %.3f] adaptive=[%.3f %.3f] "
      "rpy=[%.3f %.3f %.3f] wrench=[%.1f %.1f %.1f %.2f %.2f %.2f] "
      "residual=%.3f contacts=[%d%d%d%d]",
      was_moving_ ? "walk" : "stand",
      body_state_.position.x(), body_state_.position.y(), body_state_.position.z(),
      body_reference_.position.x(), body_reference_.position.y(),
      body_reference_.position.z(),
      integrated_position_reference_.x(), integrated_position_reference_.y(),
      adaptive_body_shift_world_.x(), adaptive_body_shift_world_.y(),
      body_state_.rpy.x(), body_state_.rpy.y(), body_state_.rpy.z(),
      solution.desired_wrench(0), solution.desired_wrench(1),
      solution.desired_wrench(2), solution.desired_wrench(3),
      solution.desired_wrench(4), solution.desired_wrench(5),
      solution.wrench_residual,
      contacts_[0], contacts_[1], contacts_[2], contacts_[3]);
  }

  std::unique_ptr<dog2_dynamics::Dog2Model> model_;
  std::unique_ptr<FlatLocomotionMPC> controller_;
  double mass_ = 0.0;
  double target_height_ = 0.20;
  double state_freshness_sec_ = 0.20;
  double command_timeout_sec_ = 0.30;
  double velocity_filter_tau_ = 0.10;
  double max_linear_speed_ = 0.12;
  double max_angular_speed_ = 0.30;
  double max_linear_acceleration_ = 0.20;
  double max_angular_acceleration_ = 0.50;
  double max_position_error_ = 0.10;
  double max_yaw_error_ = 0.35;
  double body_shift_kp_ = 800.0;
  double body_shift_kd_ = 80.0;
  double body_shift_max_linear_speed_ = 0.025;
  double body_shift_max_angular_speed_ = 0.15;
  double body_shift_max_tilt_ = 0.12;
  double support_ready_min_normal_force_ = 2.0;
  double support_target_force_margin_ = 25.0;
  double support_center_gain_ = 1.0;
  double support_center_max_speed_ = 0.10;
  double support_center_max_forward_offset_ = 0.08;
  double support_center_max_lateral_offset_ = 0.04;
  double control_dt_ = 0.02;

  FlatBodyState body_state_;
  FlatBodyReference body_reference_;
  Eigen::Quaterniond body_orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::VectorXd current_q_;
  std::unordered_map<std::string, int> joint_q_index_;
  std::unordered_map<std::string, bool> joint_seen_;
  std::array<bool, 4> contacts_{true, true, true, true};
  Eigen::Vector3d target_command_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d slewed_command_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d requested_body_shift_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d integrated_position_reference_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d adaptive_body_shift_world_ = Eigen::Vector2d::Zero();
  double integrated_yaw_reference_ = 0.0;
  double last_odom_stamp_ = 0.0;
  bool odom_received_ = false;
  bool joints_received_ = false;
  bool contact_received_ = false;
  bool contact_transition_pending_ = false;
  bool reference_initialized_ = false;
  bool walking_started_ = false;
  bool was_moving_ = false;
  double last_odom_receive_sec_ = -1.0e30;
  double last_joint_receive_sec_ = -1.0e30;
  double last_contact_receive_sec_ = -1.0e30;
  double last_command_receive_sec_ = -1.0e30;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr shift_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<dog2_interfaces::msg::ContactPhase>::SharedPtr
    contact_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    shift_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog2_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dog2_mpc::FlatMPCNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flat_mpc_node"), "fatal: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
