#include "dog2_wbc/wbc_controller.hpp"

#include <dog2_interfaces/msg/contact_phase.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dog2_wbc
{

namespace
{

constexpr std::array<const char *, 4> kLegNames{"lf", "lh", "rh", "rf"};

struct JointAddress
{
  int leg = 0;
  int joint = 0;  // rail=0, coxa=1, femur=2, tibia=3
};

double wallNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class FlatWBCNode : public rclcpp::Node
{
public:
  FlatWBCNode()
  : Node("flat_wbc_node")
  {
    declareParameters();
    initializeController();
    createInterfaces();
    RCLCPP_INFO(
      get_logger(),
      "Flat WBC ready: force sign=%.1f, torque slew=%.1f Nm/s",
      foot_force_sign_, torque_slew_rate_);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>(
      "contact_phase_topic", "/dog2/gait/contact_phase");
    declare_parameter<std::string>(
      "swing_target_topic", "/dog2/gait/swing_foot_target");
    declare_parameter<std::string>(
      "foot_force_topic", "/dog2/mpc/foot_forces");
    declare_parameter<std::string>(
      "joint_effort_topic", "/dog2/wbc/joint_effort_command");
    declare_parameter<std::string>(
      "rail_effort_topic", "/dog2/wbc/rail_effort_command");
    declare_parameter("state_freshness_sec", 0.20);
    declare_parameter("max_torque", 50.0);
    declare_parameter("foot_force_sign", -1.0);
    declare_parameter("gravity_compensation", true);
    declare_parameter("torque_slew_rate", 400.0);
    declare_parameter(
      "swing_kp", std::vector<double>{180.0, 180.0, 220.0});
    declare_parameter(
      "swing_kd", std::vector<double>{8.0, 8.0, 10.0});
    declare_parameter("joint_limit_margin", 0.12);
    declare_parameter("joint_limit_kp", 35.0);
    declare_parameter("joint_limit_kd", 1.5);
  }

  Eigen::Vector3d vectorParameter(const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain three values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void initializeController()
  {
    const std::string description =
      get_parameter("robot_description").as_string();
    if (description.empty()) {
      throw std::runtime_error("flat_wbc_node requires robot_description");
    }

    WBCController::Parameters parameters;
    parameters.max_torque = get_parameter("max_torque").as_double();
    // Rail commands are intentionally discarded by this node and handled by
    // the dedicated position lock. Keep an ordinary internal limit so the
    // controller does not emit a hard-clamp log on every gravity solve.
    parameters.max_sliding_force = 100.0;
    parameters.foot_force_sign =
      get_parameter("foot_force_sign").as_double();
    parameters.gravity_compensation =
      get_parameter("gravity_compensation").as_bool();
    parameters.swing_kp = vectorParameter("swing_kp");
    parameters.swing_kd = vectorParameter("swing_kd");
    parameters.joint_limit_margin =
      get_parameter("joint_limit_margin").as_double();
    parameters.joint_limit_kp =
      get_parameter("joint_limit_kp").as_double();
    parameters.joint_limit_kd =
      get_parameter("joint_limit_kd").as_double();
    foot_force_sign_ = parameters.foot_force_sign;
    controller_ = std::make_unique<WBCController>(parameters);
    controller_->initializeFromRobotDescription(description);

    state_freshness_sec_ =
      get_parameter("state_freshness_sec").as_double();
    torque_slew_rate_ = get_parameter("torque_slew_rate").as_double();
    for (int leg = 0; leg < 4; ++leg) {
      leg_states_[leg].joint_angles.setZero();
      leg_states_[leg].joint_velocities.setZero();
      leg_states_[leg].sliding_position = 0.0;
      leg_states_[leg].config = WBCController::LegConfiguration::ELBOW;
      leg_states_[leg].in_contact = true;
      leg_states_[leg].swing = false;

      const std::string prefix(kLegNames[leg]);
      joint_addresses_[prefix + "_rail_joint"] = {leg, 0};
      joint_addresses_[prefix + "_coxa_joint"] = {leg, 1};
      joint_addresses_[prefix + "_femur_joint"] = {leg, 2};
      joint_addresses_[prefix + "_tibia_joint"] = {leg, 3};
      for (int joint = 0; joint < 4; ++joint) {
        joint_seen_[prefix + "_" +
          std::array<const char *, 4>{
            "rail_joint", "coxa_joint", "femur_joint", "tibia_joint"}[joint]] =
          false;
      }
    }
  }

  void createInterfaces()
  {
    effort_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      get_parameter("joint_effort_topic").as_string(), 10);
    rail_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      get_parameter("rail_effort_topic").as_string(), 10);

    force_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("foot_force_topic").as_string(), 20,
      std::bind(&FlatWBCNode::forceCallback, this, std::placeholders::_1));
    joint_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&FlatWBCNode::jointCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 20,
      std::bind(&FlatWBCNode::odomCallback, this, std::placeholders::_1));
    contact_subscription_ =
      create_subscription<dog2_interfaces::msg::ContactPhase>(
      get_parameter("contact_phase_topic").as_string(), 20,
      std::bind(&FlatWBCNode::contactCallback, this, std::placeholders::_1));
    swing_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
      get_parameter("swing_target_topic").as_string(), 20,
      std::bind(&FlatWBCNode::swingCallback, this, std::placeholders::_1));
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const std::size_t count =
      std::min(message->name.size(), message->position.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto found = joint_addresses_.find(message->name[index]);
      if (found == joint_addresses_.end() ||
        !std::isfinite(message->position[index]))
      {
        continue;
      }
      const JointAddress address = found->second;
      const double velocity =
        index < message->velocity.size() &&
        std::isfinite(message->velocity[index]) ?
        message->velocity[index] : 0.0;
      if (address.joint == 0) {
        leg_states_[address.leg].sliding_position = message->position[index];
      } else {
        leg_states_[address.leg].joint_angles(address.joint - 1) =
          message->position[index];
        leg_states_[address.leg].joint_velocities(address.joint - 1) = velocity;
      }
      joint_seen_[message->name[index]] = true;
    }
    joints_received_ = std::all_of(
      joint_seen_.begin(), joint_seen_.end(),
      [](const auto & item) {return item.second;});
    last_joint_receive_sec_ = wallNowSec();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & q = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-6) {
      return;
    }
    base_orientation_ = quaternion.normalized().toRotationMatrix();
    odom_received_ = true;
    last_odom_receive_sec_ = wallNowSec();
  }

  void contactCallback(
    const dog2_interfaces::msg::ContactPhase::SharedPtr message)
  {
    std::array<bool, 4> previous_contacts{};
    for (int leg = 0; leg < 4; ++leg) {
      previous_contacts[leg] = leg_states_[leg].in_contact;
    }
    const std::size_t count =
      std::min(message->leg_names.size(), message->phase.size());
    for (std::size_t index = 0; index < count; ++index) {
      for (int leg = 0; leg < 4; ++leg) {
        if (message->leg_names[index] == kLegNames[leg]) {
          leg_states_[leg].in_contact =
            message->phase[index] !=
            dog2_interfaces::msg::ContactPhase::SWING;
        }
      }
    }
    bool changed = false;
    for (int leg = 0; leg < 4; ++leg) {
      changed = changed ||
        previous_contacts[leg] != leg_states_[leg].in_contact;
    }
    if (changed) {
      contact_transition_pending_ = true;
      RCLCPP_WARN(
        get_logger(), "flat WBC contact transition [%d%d%d%d] -> [%d%d%d%d]",
        previous_contacts[0], previous_contacts[1],
        previous_contacts[2], previous_contacts[3],
        leg_states_[0].in_contact, leg_states_[1].in_contact,
        leg_states_[2].in_contact, leg_states_[3].in_contact);
    }
    contact_received_ = true;
    last_contact_receive_sec_ = wallNowSec();
  }

  void swingCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->data.size() != 28) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat WBC expected 28 swing target values, got %zu",
        message->data.size());
      return;
    }
    for (int leg = 0; leg < 4; ++leg) {
      leg_states_[leg].swing = message->data[leg] > 0.5;
      for (int axis = 0; axis < 3; ++axis) {
        leg_states_[leg].swing_pos_des(axis) =
          message->data[4 + leg * 3 + axis];
        leg_states_[leg].swing_vel_des(axis) =
          message->data[16 + leg * 3 + axis];
      }
    }
    swing_received_ = true;
    last_swing_receive_sec_ = wallNowSec();
  }

  bool stateFresh() const
  {
    const double current = wallNowSec();
    return joints_received_ && odom_received_ &&
           contact_received_ && swing_received_ &&
           current - last_joint_receive_sec_ <= state_freshness_sec_ &&
           current - last_odom_receive_sec_ <= state_freshness_sec_ &&
           current - last_contact_receive_sec_ <= state_freshness_sec_ &&
           current - last_swing_receive_sec_ <= state_freshness_sec_;
  }

  void forceCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->data.size() != 12 || !stateFresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat WBC waiting for valid force/state inputs");
      return;
    }
    Eigen::VectorXd forces(12);
    for (int index = 0; index < 12; ++index) {
      if (!std::isfinite(message->data[index])) {
        return;
      }
      forces(index) = message->data[index];
    }

    controller_->setBaseOrientation(base_orientation_);
    Eigen::VectorXd torques =
      controller_->computeTorques(forces, leg_states_);
    if (torques.size() != 16 || !torques.allFinite()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "flat WBC produced invalid torque output");
      return;
    }
    if (contact_transition_pending_) {
      RCLCPP_WARN(
        get_logger(),
        "flat WBC transition fz=[%.2f %.2f %.2f %.2f] "
        "tau=[%.2f %.2f %.2f | %.2f %.2f %.2f | "
        "%.2f %.2f %.2f | %.2f %.2f %.2f]",
        forces(2), forces(5), forces(8), forces(11),
        torques(0), torques(1), torques(2),
        torques(3), torques(4), torques(5),
        torques(6), torques(7), torques(8),
        torques(9), torques(10), torques(11));
      contact_transition_pending_ = false;
    }

    const double current = wallNowSec();
    double dt = 0.02;
    if (last_output_sec_ > 0.0) {
      dt = clampValue(current - last_output_sec_, 1.0e-3, 0.1);
    }
    const double max_step = std::max(0.0, torque_slew_rate_) * dt;
    for (int index = 0; index < 12; ++index) {
      torques(index) = clampValue(
        torques(index),
        previous_torques_(index) - max_step,
        previous_torques_(index) + max_step);
    }
    previous_torques_ = torques.head<12>();
    last_output_sec_ = current;

    std_msgs::msg::Float64MultiArray effort;
    effort.data.assign(torques.data(), torques.data() + 12);
    effort_publisher_->publish(effort);
    std_msgs::msg::Float64MultiArray rails;
    rails.data.assign(4, 0.0);
    rail_publisher_->publish(rails);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "flat WBC torque_norm=%.2f swing=[%d%d%d%d] contact=[%d%d%d%d]",
      previous_torques_.norm(),
      leg_states_[0].swing, leg_states_[1].swing,
      leg_states_[2].swing, leg_states_[3].swing,
      leg_states_[0].in_contact, leg_states_[1].in_contact,
      leg_states_[2].in_contact, leg_states_[3].in_contact);
    for (int leg = 0; leg < 4; ++leg) {
      if (!leg_states_[leg].swing) {
        continue;
      }
      const Eigen::Vector3d foot =
        controller_->forwardKinematics(leg, leg_states_[leg]);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "flat WBC swing %s q=[%.3f %.3f %.3f] "
        "foot=[%.3f %.3f %.3f] target=[%.3f %.3f %.3f]",
        kLegNames[leg], leg_states_[leg].joint_angles.x(),
        leg_states_[leg].joint_angles.y(),
        leg_states_[leg].joint_angles.z(),
        foot.x(), foot.y(), foot.z(),
        leg_states_[leg].swing_pos_des.x(),
        leg_states_[leg].swing_pos_des.y(),
        leg_states_[leg].swing_pos_des.z());
    }
  }

  static double clampValue(double value, double lower, double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  std::unique_ptr<WBCController> controller_;
  std::array<WBCController::LegState, 4> leg_states_;
  std::unordered_map<std::string, JointAddress> joint_addresses_;
  std::unordered_map<std::string, bool> joint_seen_;
  Eigen::Matrix3d base_orientation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 12, 1> previous_torques_ =
    Eigen::Matrix<double, 12, 1>::Zero();
  double foot_force_sign_ = -1.0;
  double state_freshness_sec_ = 0.20;
  double torque_slew_rate_ = 400.0;
  bool joints_received_ = false;
  bool odom_received_ = false;
  bool contact_received_ = false;
  bool swing_received_ = false;
  bool contact_transition_pending_ = false;
  double last_joint_receive_sec_ = -1.0e30;
  double last_odom_receive_sec_ = -1.0e30;
  double last_contact_receive_sec_ = -1.0e30;
  double last_swing_receive_sec_ = -1.0e30;
  double last_output_sec_ = -1.0e30;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rail_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    force_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<dog2_interfaces::msg::ContactPhase>::SharedPtr
    contact_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    swing_subscription_;
};

}  // namespace dog2_wbc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dog2_wbc::FlatWBCNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flat_wbc_node"), "fatal: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
