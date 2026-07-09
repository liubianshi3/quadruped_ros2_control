#include "dog2_wbc/wbc_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dog2_interfaces/msg/contact_phase.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace dog2_wbc
{

/**
 * @brief 完整的WBC节点
 *
 * 功能：
 * 1. 精确的雅可比计算
 * 2. 混合构型支持
 * 3. 滑动副力计算
 */
class WBCNodeComplete : public rclcpp::Node
{
public:
  WBCNodeComplete()
  : Node("wbc_node_complete")
  {
    initializeParameters();
    initializeController();
    initializePublishersSubscribers();

    RCLCPP_INFO(this->get_logger(), "Complete WBC Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Using accurate Jacobian calculation");
    RCLCPP_INFO(this->get_logger(), "  Supporting hybrid configuration");
  }

private:
  void initializeParameters()
  {
    // 声明参数
    this->declare_parameter("l1", 0.2);
    this->declare_parameter("l2", 0.2);
    this->declare_parameter("max_torque", 50.0);
    this->declare_parameter("max_sliding_force", 100.0);
    this->declare_parameter("foot_force_sign", 1.0);
    this->declare_parameter("gravity_compensation", true);
    this->declare_parameter("swing_kp_x", 300.0);
    this->declare_parameter("swing_kp_y", 300.0);
    this->declare_parameter("swing_kp_z", 300.0);
    this->declare_parameter("swing_kd_x", 8.0);
    this->declare_parameter("swing_kd_y", 8.0);
    this->declare_parameter("swing_kd_z", 8.0);
    this->declare_parameter("posture_pd_enabled", true);
    this->declare_parameter("posture_target_coxa", 0.0);
    this->declare_parameter("posture_target_femur", 1.05);
    this->declare_parameter("posture_target_tibia", -1.10);
    this->declare_parameter("posture_kp_coxa", 20.0);
    this->declare_parameter("posture_kp_femur", 35.0);
    this->declare_parameter("posture_kp_tibia", 35.0);
    this->declare_parameter("posture_kd_coxa", 2.0);
    this->declare_parameter("posture_kd_femur", 4.0);
    this->declare_parameter("posture_kd_tibia", 4.0);
    this->declare_parameter("posture_max_torque", 25.0);
    this->declare_parameter("rail_hold_enabled", false);
    this->declare_parameter("rail_hold_hover_enabled", false);
    this->declare_parameter("rail_hold_crossing_staging_enabled", false);
    this->declare_parameter("lock_rails_when_not_crossing_motion", true);
    this->declare_parameter("rail_hold_kp", 450.0);
    this->declare_parameter("rail_hold_kd", 25.0);
    this->declare_parameter("rail_hold_max_force", 70.0);
    this->declare_parameter<std::string>(
      "contact_phase_topic", "/dog2/gait/contact_phase");
    this->declare_parameter<std::string>(
      "swing_foot_target_topic", "/dog2/gait/swing_foot_target");
    this->declare_parameter<std::string>("robot_description", "");
    // 力坐标系修正：MPC 足端力是世界系，J^T 在 base 系。订阅 odom 姿态
    // 并把力旋回 base 系；odom 超时则退化为单位阵（等价旧行为）并告警。
    this->declare_parameter("rotate_forces_to_base", true);
    this->declare_parameter<std::string>(
      "odom_topic", "/dog2/state_estimation/odom");
    this->declare_parameter("odom_timeout_sec", 0.5);

    // 获取参数
    WBCController::Parameters params;
    params.l1 = this->get_parameter("l1").as_double();
    params.l2 = this->get_parameter("l2").as_double();
    params.max_torque = this->get_parameter("max_torque").as_double();
    params.max_sliding_force = this->get_parameter("max_sliding_force").as_double();
    params.foot_force_sign = this->get_parameter("foot_force_sign").as_double();
    params.gravity_compensation = this->get_parameter("gravity_compensation").as_bool();
    params.swing_kp << this->get_parameter("swing_kp_x").as_double(),
      this->get_parameter("swing_kp_y").as_double(),
      this->get_parameter("swing_kp_z").as_double();
    params.swing_kd << this->get_parameter("swing_kd_x").as_double(),
      this->get_parameter("swing_kd_y").as_double(),
      this->get_parameter("swing_kd_z").as_double();
    posture_pd_enabled_ = this->get_parameter("posture_pd_enabled").as_bool();
    posture_target_ << this->get_parameter("posture_target_coxa").as_double(),
      this->get_parameter("posture_target_femur").as_double(),
      this->get_parameter("posture_target_tibia").as_double();
    posture_kp_ << this->get_parameter("posture_kp_coxa").as_double(),
      this->get_parameter("posture_kp_femur").as_double(),
      this->get_parameter("posture_kp_tibia").as_double();
    posture_kd_ << this->get_parameter("posture_kd_coxa").as_double(),
      this->get_parameter("posture_kd_femur").as_double(),
      this->get_parameter("posture_kd_tibia").as_double();
    posture_max_torque_ = this->get_parameter("posture_max_torque").as_double();
    rail_hold_enabled_ = this->get_parameter("rail_hold_enabled").as_bool();
    rail_hold_hover_enabled_ = this->get_parameter("rail_hold_hover_enabled").as_bool();
    rail_hold_crossing_staging_enabled_ =
      this->get_parameter("rail_hold_crossing_staging_enabled").as_bool();
    lock_rails_when_not_crossing_motion_ =
      this->get_parameter("lock_rails_when_not_crossing_motion").as_bool();
    rail_hold_kp_ = this->get_parameter("rail_hold_kp").as_double();
    rail_hold_kd_ = this->get_parameter("rail_hold_kd").as_double();
    rail_hold_max_force_ = this->get_parameter("rail_hold_max_force").as_double();
    rotate_forces_to_base_ = this->get_parameter("rotate_forces_to_base").as_bool();
    odom_timeout_sec_ = this->get_parameter("odom_timeout_sec").as_double();

    wbc_params_ = params;
    RCLCPP_INFO(
      this->get_logger(),
      "WBC foot_force_sign=%.1f gravity_compensation=%s rotate_forces_to_base=%s "
      "(torque = sign * J^T * R_wb^T * f + [g(q) if enabled])",
      wbc_params_.foot_force_sign,
      wbc_params_.gravity_compensation ? "on" : "off",
      rotate_forces_to_base_ ? "on" : "off");
  }

  void initializeController()
  {
    wbc_controller_ = std::make_unique<WBCController>(wbc_params_);
    const std::string robot_description =
      this->get_parameter("robot_description").as_string();
    if (!robot_description.empty()) {
      try {
        wbc_controller_->initializeFromRobotDescription(robot_description);
        RCLCPP_INFO(this->get_logger(), "WBC loaded URDF-derived Pinocchio Jacobian model");
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to load WBC robot_description model: %s",
          exc.what());
        throw;
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "WBC robot_description is empty; using legacy simplified Jacobian fallback");
    }

    // 初始化腿状态（默认全部肘式）
    for (int i = 0; i < 4; ++i) {
      leg_states_[i].config = WBCController::LegConfiguration::ELBOW;
      leg_states_[i].in_contact = true;
      leg_states_[i].joint_angles.setZero();
      leg_states_[i].sliding_position = 0.0;
      joint_velocities_[i].setZero();
    }
    sliding_velocities_.setZero();
  }

  void initializePublishersSubscribers()
  {
    // 订阅MPC输出的足端力
    foot_force_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dog2/mpc/foot_forces", 10,
      std::bind(&WBCNodeComplete::footForceCallback, this, std::placeholders::_1));

    // 订阅关节状态
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&WBCNodeComplete::jointCallback, this, std::placeholders::_1));

    crossing_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/dog2/mpc/crossing_state", 10,
      std::bind(&WBCNodeComplete::crossingStateCallback, this, std::placeholders::_1));

    if (rotate_forces_to_base_) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 10,
        std::bind(&WBCNodeComplete::odomCallback, this, std::placeholders::_1));
    }

    contact_phase_sub_ =
      this->create_subscription<dog2_interfaces::msg::ContactPhase>(
      this->get_parameter("contact_phase_topic").as_string(), 10,
      std::bind(
        &WBCNodeComplete::contactPhaseCallback,
        this,
        std::placeholders::_1));

    swing_target_sub_ =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
      this->get_parameter("swing_foot_target_topic").as_string(), 10,
      std::bind(
        &WBCNodeComplete::swingTargetCallback,
        this,
        std::placeholders::_1));

    // 发布关节力矩命令（12个旋转关节）
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/joint_group_effort_controller/commands", 10);

    // 发布滑动副力命令（4个滑动副）
    sliding_force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/sliding_joint_effort_controller/commands", 10);
  }

  void footForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) {
      RCLCPP_ERROR(
        this->get_logger(), "Expected 12 foot forces, got %zu",
        msg->data.size());
      return;
    }

    if (!joint_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for joint states...");
      return;
    }

    // 提取足端力
    Eigen::VectorXd foot_forces(12);
    for (int i = 0; i < 12; ++i) {
      foot_forces(i) = msg->data[i];
    }
    for (int leg = 0; leg < 4; ++leg) {
      leg_states_[leg].joint_velocities = joint_velocities_[leg];
    }

    // 计算关节力矩（使用精确雅可比；力先按当前躯干姿态旋回 base 系）
    updateControllerBaseOrientation();
    Eigen::VectorXd torques = wbc_controller_->computeTorques(
      foot_forces, leg_states_);
    applyPostureBias(torques);
    applyRailPolicy(torques);

    // 发布旋转关节力矩
    auto torque_msg = std_msgs::msg::Float64MultiArray();
    torque_msg.data.resize(12);
    for (int i = 0; i < 12; ++i) {
      torque_msg.data[i] = torques(i);
    }
    torque_pub_->publish(torque_msg);

    // 发布滑动副力
    auto sliding_msg = std_msgs::msg::Float64MultiArray();
    sliding_msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
      sliding_msg.data[i] = torques(12 + i);
    }
    sliding_force_pub_->publish(sliding_msg);

    // 统计
    if (++control_count_ % 20 == 0) {
      auto stats = wbc_controller_->getLastSolveStats();
      double total_fz = foot_forces(2) + foot_forces(5) +
        foot_forces(8) + foot_forces(11);

      RCLCPP_INFO(
        this->get_logger(),
        "WBC: t=%.2fms, fz=%.1fN, τ_norm=%.2f, configs=[%s,%s,%s,%s]",
        stats.solve_time_ms,
        total_fz,
        stats.torque_norm,
        getConfigString(0).c_str(),
        getConfigString(1).c_str(),
        getConfigString(2).c_str(),
        getConfigString(3).c_str());
      logEffortBreakdown(foot_forces, torques);
    }
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 更新腿状态
    // 假设关节顺序：j1_hip_roll, j1_hip_pitch, j1_knee, j2_..., j3_..., j4_...
    // 滑动副：j1, j2, j3, j4

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string & name = msg->name[i];
      double pos = msg->position[i];
      double vel = 0.0;
      if (i < msg->velocity.size()) {
        vel = msg->velocity[i];
      }

      // 解析关节名称（兼容旧命名和 dog2.urdf.xacro 的 lf/rf/lh/rh 命名）
      if (name.find("hip_roll") != std::string::npos ||
        name.find("_haa_joint") != std::string::npos ||
        name.find("_coxa_joint") != std::string::npos)
      {
        int leg = getLegIdFromName(name);
        if (leg >= 0) {
          leg_states_[leg].joint_angles(0) = pos;
          joint_velocities_[leg](0) = vel;
        }
      } else if (name.find("hip_pitch") != std::string::npos ||
        name.find("_hfe_joint") != std::string::npos ||
        name.find("_femur_joint") != std::string::npos)
      {
        int leg = getLegIdFromName(name);
        if (leg >= 0) {
          leg_states_[leg].joint_angles(1) = pos;
          joint_velocities_[leg](1) = vel;
        }
      } else if (name.find("knee") != std::string::npos ||
        name.find("_kfe_joint") != std::string::npos ||
        name.find("_tibia_joint") != std::string::npos)
      {
        int leg = getLegIdFromName(name);
        if (leg >= 0) {
          leg_states_[leg].joint_angles(2) = pos;
          joint_velocities_[leg](2) = vel;
        }
      } else if (name == "j1" || name == "lf_rail_joint") {
        leg_states_[0].sliding_position = pos;
        sliding_velocities_(0) = vel;
      } else if (name == "j2" || name == "lh_rail_joint") {
        leg_states_[1].sliding_position = pos;
        sliding_velocities_(1) = vel;
      } else if (name == "j3" || name == "rh_rail_joint") {
        leg_states_[2].sliding_position = pos;
        sliding_velocities_(2) = vel;
      } else if (name == "j4" || name == "rf_rail_joint") {
        leg_states_[3].sliding_position = pos;
        sliding_velocities_(3) = vel;
      }
    }

    joint_received_ = true;
  }

  void crossingStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string state = msg->data;
    if (state == last_crossing_state_) {
      return;
    }

    last_crossing_state_ = state;
    applyLegConfigurationsFromCrossingState(state);
    RCLCPP_INFO(this->get_logger(), "WBC crossing state -> %s", state.c_str());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    const Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (std::abs(quat.norm() - 1.0) > 0.1) {
      return;
    }
    base_orientation_ = quat.normalized().toRotationMatrix();
    last_odom_time_ = this->now();
  }

  void updateControllerBaseOrientation()
  {
    if (!rotate_forces_to_base_) {
      return;
    }
    const bool fresh =
      last_odom_time_.nanoseconds() > 0 &&
      (this->now() - last_odom_time_).seconds() <= odom_timeout_sec_;
    if (fresh) {
      wbc_controller_->setBaseOrientation(base_orientation_);
    } else {
      // Stale attitude: fall back to identity (legacy behaviour) rather
      // than mapping forces through an outdated rotation.
      wbc_controller_->setBaseOrientation(Eigen::Matrix3d::Identity());
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "WBC odom attitude stale (> %.1f s); mapping forces with identity rotation",
        odom_timeout_sec_);
    }
  }

  void contactPhaseCallback(
    const dog2_interfaces::msg::ContactPhase::SharedPtr msg)
  {
    const size_t count = std::min(msg->leg_names.size(), msg->phase.size());
    for (size_t i = 0; i < count; ++i) {
      const int leg = getLegIdFromName(msg->leg_names[i]);
      if (leg >= 0) {
        leg_states_[leg].in_contact =
          msg->phase[i] != dog2_interfaces::msg::ContactPhase::SWING;
      }
    }
  }

  void swingTargetCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 28) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Expected 28 swing target values, got %zu",
        msg->data.size());
      return;
    }

    for (int leg = 0; leg < 4; ++leg) {
      leg_states_[leg].swing = msg->data[leg] > 0.5;
      const int pos_offset = 4 + leg * 3;
      const int vel_offset = 16 + leg * 3;
      leg_states_[leg].swing_pos_des <<
        msg->data[pos_offset],
        msg->data[pos_offset + 1],
        msg->data[pos_offset + 2];
      leg_states_[leg].swing_vel_des <<
        msg->data[vel_offset],
        msg->data[vel_offset + 1],
        msg->data[vel_offset + 2];
    }
  }

  void applyLegConfigurationsFromCrossingState(const std::string & state)
  {
    auto set_all = [&](WBCController::LegConfiguration config) {
        for (auto & leg_state : leg_states_) {
          leg_state.config = config;
        }
      };

    auto set_front_rear = [&](WBCController::LegConfiguration front,
        WBCController::LegConfiguration rear) {
        leg_states_[0].config = front;
        leg_states_[3].config = front;
        leg_states_[1].config = rear;
        leg_states_[2].config = rear;
      };

    if (state.rfind("CROSSING:", 0) != 0) {
      set_all(WBCController::LegConfiguration::ELBOW);
      return;
    }

    const std::string stage = state.substr(std::string("CROSSING:").size());
    if (stage == "FRONT_LEGS_TRANSIT" ||
      stage == "HYBRID_GAIT_WALKING" ||
      stage == "RAIL_ALIGNMENT")
    {
      set_front_rear(
        WBCController::LegConfiguration::KNEE,
        WBCController::LegConfiguration::ELBOW);
      return;
    }

    if (stage == "REAR_LEGS_TRANSIT" ||
      stage == "ALL_KNEE_STATE")
    {
      set_all(WBCController::LegConfiguration::KNEE);
      return;
    }

    set_all(WBCController::LegConfiguration::ELBOW);
  }

  bool shouldUseMpcRailEffort() const
  {
    if (last_crossing_state_.rfind("CROSSING:", 0) != 0) {
      return false;
    }

    const std::string stage =
      last_crossing_state_.substr(std::string("CROSSING:").size());
    return stage != "PRE_APPROACH" &&
           stage != "APPROACH" &&
           stage != "CONTINUE_FORWARD" &&
           stage != "COMPLETED";
  }

  bool shouldHoldRails() const
  {
    if (lock_rails_when_not_crossing_motion_ && !shouldUseMpcRailEffort()) {
      return true;
    }
    if (!rail_hold_enabled_) {
      return false;
    }
    if (rail_hold_hover_enabled_ && last_crossing_state_ == "HOVER") {
      return true;
    }
    if (rail_hold_crossing_staging_enabled_ &&
      (last_crossing_state_ == "CROSSING:PRE_APPROACH" ||
      last_crossing_state_ == "CROSSING:APPROACH"))
    {
      return true;
    }
    return false;
  }

  Eigen::Vector4d computeRailHoldEffort(Eigen::Vector4d & raw_efforts) const
  {
    Eigen::Vector4d hold_effort = Eigen::Vector4d::Zero();
    raw_efforts.setZero();

    for (int leg = 0; leg < 4; ++leg) {
      const double position = leg_states_[leg].sliding_position;
      const double velocity = sliding_velocities_(leg);
      const double raw_effort =
        rail_hold_kp_ * (rail_hold_targets_(leg) - position) -
        rail_hold_kd_ * velocity;
      raw_efforts(leg) = raw_effort;
      hold_effort(leg) =
        std::max(
        -rail_hold_max_force_,
        std::min(rail_hold_max_force_, raw_effort));
    }
    return hold_effort;
  }

  void applyRailPolicy(Eigen::VectorXd & torques)
  {
    if (torques.size() < 16 || !shouldHoldRails()) {
      return;
    }

    Eigen::Vector4d raw_efforts = Eigen::Vector4d::Zero();
    const Eigen::Vector4d hold_effort = computeRailHoldEffort(raw_efforts);
    const bool lock_mode =
      lock_rails_when_not_crossing_motion_ && !shouldUseMpcRailEffort();
    for (int leg = 0; leg < 4; ++leg) {
      if (lock_mode) {
        torques(12 + leg) = hold_effort(leg);
      } else {
        torques(12 + leg) += hold_effort(leg);
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000,
      "WBC rail_%s: state=%s target=[%.3f,%.3f,%.3f,%.3f] pos=[%.3f,%.3f,%.3f,%.3f] raw=[%.1f,%.1f,%.1f,%.1f] effort=[%.1f,%.1f,%.1f,%.1f]",
      lock_mode ? "lock" : "hold",
      last_crossing_state_.c_str(),
      rail_hold_targets_(0),
      rail_hold_targets_(1),
      rail_hold_targets_(2),
      rail_hold_targets_(3),
      leg_states_[0].sliding_position,
      leg_states_[1].sliding_position,
      leg_states_[2].sliding_position,
      leg_states_[3].sliding_position,
      raw_efforts(0),
      raw_efforts(1),
      raw_efforts(2),
      raw_efforts(3),
      hold_effort(0),
      hold_effort(1),
      hold_effort(2),
      hold_effort(3));
  }

  void applyPostureBias(Eigen::VectorXd & torques)
  {
    if (!posture_pd_enabled_ || torques.size() < 12) {
      return;
    }

    // Joint-space stance shaping is a stand-up / quiet-standing aid only.
    // During WALKING the body must rotate the stance joints to advance over
    // the feet, so a posture PD there fights forward progression.
    if (last_crossing_state_ != "HOVER") {
      return;
    }

    for (int leg = 0; leg < 4; ++leg) {
      if (leg_states_[leg].swing) {
        continue;
      }
      Eigen::Vector3d raw =
        posture_kp_.cwiseProduct(posture_target_ - leg_states_[leg].joint_angles) -
        posture_kd_.cwiseProduct(joint_velocities_[leg]);
      for (int joint = 0; joint < 3; ++joint) {
        raw(joint) = std::max(
          -posture_max_torque_,
          std::min(posture_max_torque_, raw(joint)));
      }
      torques.segment<3>(leg * 3) += raw;
    }
  }

  int getLegIdFromName(const std::string & name)
  {
    // Bare names ("lf") come from the gait scheduler contact phase topic;
    // prefixed names ("lf_coxa_joint") come from joint states.
    if (name == "lf" || name.find("lf_") != std::string::npos ||
      name.find("j1") != std::string::npos || name.find("leg1") != std::string::npos)
    {
      return 0;
    } else if (name == "lh" || name.find("lh_") != std::string::npos ||
      name.find("j2") != std::string::npos || name.find("leg2") != std::string::npos)
    {
      return 1;
    } else if (name == "rh" || name.find("rh_") != std::string::npos ||
      name.find("j3") != std::string::npos || name.find("leg3") != std::string::npos)
    {
      return 2;
    } else if (name == "rf" || name.find("rf_") != std::string::npos ||
      name.find("j4") != std::string::npos || name.find("leg4") != std::string::npos)
    {
      return 3;
    }
    return -1;
  }

  std::string getConfigString(int leg) const
  {
    return (leg_states_[leg].config == WBCController::LegConfiguration::ELBOW) ?
           "E" : "K";
  }

  void logEffortBreakdown(
    const Eigen::VectorXd & foot_forces,
    const Eigen::VectorXd & torques)
  {
    if (foot_forces.size() < 12 || torques.size() < 16) {
      return;
    }

    static constexpr const char * kLegNames[4] = {"lf", "lh", "rh", "rf"};
    constexpr double kNearSaturationRatio = 0.92;
    constexpr double kMinJacobianElement = 1e-4;
    std::ostringstream stream;
    stream << "WBC leg_effort:";
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d f_leg = foot_forces.segment<3>(leg * 3);
      const Eigen::Vector3d tau_leg = torques.segment<3>(leg * 3);
      const double rail_effort = torques(12 + leg);
      const Eigen::MatrixXd jacobian =
        wbc_controller_->computeLegJacobian(leg, leg_states_[leg]);
      const double femur_jz = std::abs(jacobian(2, 1));
      const double tibia_jz = std::abs(jacobian(2, 2));
      const double femur_util =
        std::abs(tau_leg(1)) / std::max(1e-6, wbc_params_.max_torque);
      const double tibia_util =
        std::abs(tau_leg(2)) / std::max(1e-6, wbc_params_.max_torque);
      const double rail_util =
        std::abs(rail_effort) / std::max(1e-6, wbc_params_.max_sliding_force);
      const double femur_fz_capacity =
        femur_jz > kMinJacobianElement ? wbc_params_.max_torque / femur_jz : 1e6;
      const double tibia_fz_capacity =
        tibia_jz > kMinJacobianElement ? wbc_params_.max_torque / tibia_jz : 1e6;
      const double fz_capacity =
        std::min(femur_fz_capacity, tibia_fz_capacity);
      const double fz_margin = fz_capacity - std::abs(f_leg.z());
      const double support_abs = std::abs(tau_leg(1)) + std::abs(tau_leg(2));
      const double mech_power =
        tau_leg.dot(joint_velocities_[leg]) + rail_effort * sliding_velocities_(leg);
      stream << " " << kLegNames[leg]
             << "[fz=" << f_leg.z()
             << " c=" << tau_leg(0)
             << " f=" << tau_leg(1)
             << " t=" << tau_leg(2)
             << " rail=" << rail_effort
             << " ft_abs=" << support_abs
             << " fu=" << femur_util
             << " tu=" << tibia_util
             << " ru=" << rail_util
             << " sat=" << ((femur_util >= kNearSaturationRatio ||
      tibia_util >= kNearSaturationRatio) ? 1 : 0)
             << " jzf=" << femur_jz
             << " jzt=" << tibia_jz
             << " fz_cap=" << fz_capacity
             << " fz_margin=" << fz_margin
             << " p=" << mech_power << "]";
      if (femur_util >= 0.99 || tibia_util >= 0.99 || rail_util >= 0.99) {
        RCLCPP_WARN(
          this->get_logger(),
          "SATURATION WARNING! Leg %s: femur_util=%.2f, tibia_util=%.2f, rail_util=%.2f",
          kLegNames[leg], femur_util, tibia_util, rail_util);
      }
    }
    RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());
  }

  std::unique_ptr<WBCController> wbc_controller_;
  WBCController::Parameters wbc_params_;
  bool rotate_forces_to_base_ = true;
  double odom_timeout_sec_ = 0.5;
  Eigen::Matrix3d base_orientation_ = Eigen::Matrix3d::Identity();
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::array<WBCController::LegState, 4> leg_states_;
  std::array<Eigen::Vector3d, 4> joint_velocities_;
  Eigen::Vector4d sliding_velocities_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d rail_hold_targets_ = Eigen::Vector4d::Zero();
  bool rail_hold_enabled_ = false;
  bool rail_hold_hover_enabled_ = false;
  bool rail_hold_crossing_staging_enabled_ = false;
  bool lock_rails_when_not_crossing_motion_ = true;
  double rail_hold_kp_ = 450.0;
  double rail_hold_kd_ = 25.0;
  double rail_hold_max_force_ = 70.0;
  bool posture_pd_enabled_ = true;
  Eigen::Vector3d posture_target_ = Eigen::Vector3d(0.0, -0.3, -0.6);
  Eigen::Vector3d posture_kp_ = Eigen::Vector3d(20.0, 35.0, 35.0);
  Eigen::Vector3d posture_kd_ = Eigen::Vector3d(2.0, 4.0, 4.0);
  double posture_max_torque_ = 25.0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr foot_force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr crossing_state_sub_;
  rclcpp::Subscription<dog2_interfaces::msg::ContactPhase>::SharedPtr contact_phase_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr swing_target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sliding_force_pub_;

  bool joint_received_ = false;
  int control_count_ = 0;
  std::string last_crossing_state_ = "HOVER";
};

} // namespace dog2_wbc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog2_wbc::WBCNodeComplete>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
