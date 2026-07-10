#include "dog2_mpc/mpc_controller.hpp"
#include "dog2_mpc/trajectory_generator.hpp"
#include "dog2_mpc/contact_detector.hpp"
#include "dog2_mpc/hybrid_gait_generator.hpp"
#include "dog2_mpc/crossing_state_machine.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <dog2_interfaces/msg/contact_phase.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include "dog2_dynamics/dog2_model.hpp"
#include <stdexcept>

namespace dog2_mpc {

/**
 * @brief 完整的16维MPC节点
 * 
 * 功能：
 * 1. 行走控制（Trot步态）
 * 2. 窗框越障
 * 3. 滑动副协调控制
 * 4. 足端接触检测
 * 5. 参考轨迹生成
 */
class MPCNodeComplete : public rclcpp::Node {
public:
    MPCNodeComplete() : Node("mpc_node_complete") {
        initializeParameters();
        initializeControllers();
        initializePublishersSubscribers();
        
        RCLCPP_INFO(this->get_logger(), "Complete 16D MPC Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Mass: %.2f kg", mass_);
        RCLCPP_INFO(this->get_logger(), "  Horizon: %d", horizon_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_freq_);
        RCLCPP_INFO(this->get_logger(), "  Mode: %s", getModeString().c_str());
        RCLCPP_INFO(this->get_logger(), "  Stance (L x W): %.3f x %.3f m",
                    stance_length_, stance_width_);
        RCLCPP_INFO(this->get_logger(), "  Nominal body height: %.3f m", nominal_body_height_);
        RCLCPP_INFO(this->get_logger(), "  CoM offset: [%.3f, %.3f, %.3f] m",
                    com_offset_.x(), com_offset_.y(), com_offset_.z());
    }

private:
    struct LevelAttitudeError {
        double roll_like = 0.0;
        double pitch_like = 0.0;
        double tilt = 0.0;
        double body_up_z = 1.0;
        bool valid = false;
        bool inverted = false;
    };

    static double clampDouble(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    // MPC leg order (lf,lh,rh,rf). Do NOT use Dog2Model::FOOT_NAMES here:
    // that array is in rh,rf,lf,lh order.
    static constexpr const char* kMpcFootFrames[4] = {
        "lf_foot_link",
        "lh_foot_link",
        "rh_foot_link",
        "rf_foot_link"
    };

    Eigen::Vector4d rail_position_lower_;
    Eigen::Vector4d rail_position_upper_;
    Eigen::Matrix3d body_inertia_ = Eigen::Matrix3d::Identity();

    void loadRobotDescriptionDerivedModelInfo() {
        const std::string robot_description =
            this->get_parameter("robot_description").as_string();

        if (robot_description.empty()) {
            throw std::runtime_error(
                "mpc_node_complete requires robot_description parameter to load model data from URDF");
        }

        dog2_model_ = std::make_unique<dog2_dynamics::Dog2Model>(
            dog2_dynamics::Dog2Model::fromUrdfXml(robot_description));
        auto& dog2_model = *dog2_model_;
        rail_position_lower_ = dog2_model.slidingJointLowerLimits();
        rail_position_upper_ = dog2_model.slidingJointUpperLimits();
        mass_ = dog2_model.mass();

        const auto& pin_model = dog2_model.getModel();
        if (!pin_model.existFrame("base_link")) {
            throw std::runtime_error("URDF-derived Dog2Model is missing base_link frame");
        }
        const auto base_frame_id = pin_model.getFrameId("base_link");
        body_inertia_ = pin_model.frames[base_frame_id].inertia.inertia().matrix();

        // Nominal feet must be evaluated at the standing pose, not q=0:
        // at q=0 the legs hang straight (z=-0.465, wrong x), which used to
        // feed the SRBD QP a stance geometry 0.2 m away from reality.
        const std::vector<double> stance_pose =
            this->get_parameter("stance_joint_pose").as_double_array();
        Eigen::VectorXd q_stance = Eigen::VectorXd::Zero(dog2_model.nq());
        if (stance_pose.size() == 4) {
            const auto& model = dog2_model.getModel();
            static constexpr const char* kLegPrefixes[4] = {"lf", "lh", "rh", "rf"};
            static constexpr const char* kJointSuffixes[4] = {
                "rail_joint", "coxa_joint", "femur_joint", "tibia_joint"};
            for (const char* prefix : kLegPrefixes) {
                for (int j = 0; j < 4; ++j) {
                    const std::string joint_name =
                        std::string(prefix) + "_" + kJointSuffixes[j];
                    if (model.existJointName(joint_name)) {
                        const auto joint_id = model.getJointId(joint_name);
                        q_stance(model.idx_qs[joint_id]) = stance_pose[j];
                    }
                }
            }
        }
        base_foot_positions_ = Eigen::MatrixXd::Zero(4, 3);
        for (int i = 0; i < 4; ++i) {
            base_foot_positions_.row(i) =
                dog2_model.footPosition(kMpcFootFrames[i], q_stance).transpose();
        }
        base_foot_positions_.rowwise() += com_offset_.transpose();
        com_stance_ = dog2_model.centerOfMass(q_stance) + com_offset_;
        RCLCPP_INFO(this->get_logger(),
                    "Stance COM (base frame): [%.4f, %.4f, %.4f]",
                    com_stance_(0), com_stance_(1), com_stance_(2));

        RCLCPP_INFO(this->get_logger(),
                    "Loaded rail limits from URDF: lower=[%.3f %.3f %.3f %.3f], upper=[%.3f %.3f %.3f %.3f]",
                    rail_position_lower_(0), rail_position_lower_(1),
                    rail_position_lower_(2), rail_position_lower_(3),
                    rail_position_upper_(0), rail_position_upper_(1),
                    rail_position_upper_(2), rail_position_upper_(3));
        RCLCPP_INFO(this->get_logger(),
                    "Loaded MPC model data from URDF: mass=%.3f kg, base_link inertia diag=[%.5f %.5f %.5f]",
                    mass_, body_inertia_(0, 0), body_inertia_(1, 1), body_inertia_(2, 2));
        RCLCPP_INFO(this->get_logger(),
                    "Loaded URDF base feet (lf,lh,rh,rf) x=[%.3f %.3f %.3f %.3f] y=[%.3f %.3f %.3f %.3f] z=[%.3f %.3f %.3f %.3f]",
                    base_foot_positions_(0, 0), base_foot_positions_(1, 0),
                    base_foot_positions_(2, 0), base_foot_positions_(3, 0),
                    base_foot_positions_(0, 1), base_foot_positions_(1, 1),
                    base_foot_positions_(2, 1), base_foot_positions_(3, 1),
                    base_foot_positions_(0, 2), base_foot_positions_(1, 2),
                    base_foot_positions_(2, 2), base_foot_positions_(3, 2));

        // q-index map for runtime FK (measured joints -> Pinocchio q).
        // MPC leg order lf,lh,rh,rf; per-leg joint order rail,coxa,femur,tibia.
        {
            static constexpr const char* kFkLegPrefixes[4] = {"lf", "lh", "rh", "rf"};
            static constexpr const char* kFkJointSuffixes[4] = {
                "rail_joint", "coxa_joint", "femur_joint", "tibia_joint"};
            runtime_fk_ready_ = true;
            for (int leg = 0; leg < 4; ++leg) {
                for (int j = 0; j < 4; ++j) {
                    const std::string joint_name =
                        std::string(kFkLegPrefixes[leg]) + "_" + kFkJointSuffixes[j];
                    if (!pin_model.existJointName(joint_name)) {
                        fk_q_index_[leg][j] = -1;
                        runtime_fk_ready_ = false;
                        continue;
                    }
                    fk_q_index_[leg][j] =
                        pin_model.idx_qs[pin_model.getJointId(joint_name)];
                }
            }
            if (!runtime_fk_ready_) {
                RCLCPP_WARN(this->get_logger(),
                            "Runtime FK disabled: URDF is missing expected leg joints; "
                            "support shaping will keep using stance geometry.");
            }
        }
        // Sane lever arms until the first joint state arrives.
        for (int i = 0; i < 4; ++i) {
            support_lever_arms_.row(i) =
                base_foot_positions_.row(i) - com_stance_.transpose();
        }
    }

    void initializeParameters() {
        // 声明参数
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter("mass", 11.8);
        this->declare_parameter("horizon", 10);
        this->declare_parameter("dt", 0.05);
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("enable_sliding_constraints", true);
        this->declare_parameter("mode", "hover");  // hover, walking, crossing
        this->declare_parameter("slack_linear_weight", 5e3);
        this->declare_parameter("rail_tracking_error_threshold", 0.020);
        this->declare_parameter("support_polygon_margin_threshold", 0.015);
        this->declare_parameter("crossing_transition_stable_time", 0.15);
        this->declare_parameter("default_stance_length", 0.40);
        this->declare_parameter("default_stance_width", 0.30);
        // [rail, coxa, femur, tibia] applied to all four legs when deriving
        // the nominal stance feet from the URDF.
        this->declare_parameter("stance_joint_pose",
                                std::vector<double>{0.0, 0.0, 1.05, -1.10});
        this->declare_parameter("nominal_body_height", 0.28);
        this->declare_parameter("com_offset_x", 0.0);
        this->declare_parameter("com_offset_y", 0.0);
        this->declare_parameter("com_offset_z", 0.0);
        this->declare_parameter("crossing_window_x_position", 1.55);
        this->declare_parameter("crossing_window_width", 0.48);
        this->declare_parameter("crossing_window_height", 0.62);
        this->declare_parameter("crossing_window_bottom_height", 0.0);
        this->declare_parameter("crossing_window_top_height", 0.62);
        this->declare_parameter("crossing_window_safety_margin", 0.04);
        this->declare_parameter("crossing_activation_distance", 0.25);
        this->declare_parameter("crossing_approach_speed", 0.05);
        this->declare_parameter("vertical_support_enabled", true);
        this->declare_parameter("vertical_support_target_height", 0.0);
        this->declare_parameter("vertical_support_kp", 700.0);
        this->declare_parameter("vertical_support_kd", 90.0);
        this->declare_parameter("vertical_support_min_total_force_multiplier", 1.20);
        this->declare_parameter("vertical_support_max_leg_force", 100.0);
        this->declare_parameter("vertical_support_height_error_limit", 0.25);
        // Max UPWARD per-leg fz rate (N/s) in flat modes; 0 disables.
        this->declare_parameter("vertical_support_load_rate", 500.0);
        // Scheduled-stance feet whose FK world-z lever rides more than this
        // above the lowest foot are treated as airborne (no support force).
        // Must exceed the trot-rock lever spread (~8 cm at 0.3 rad tilt);
        // 0 disables.
        this->declare_parameter("support_foot_height_gate", 0.12);
        this->declare_parameter("hover_force_sanitize_enabled", true);
        this->declare_parameter("hover_force_max_leg_fz", 55.0);
        this->declare_parameter("hover_force_min_leg_fz", 18.0);
        this->declare_parameter("attitude_support_enabled", true);
        this->declare_parameter("attitude_support_hover_enabled", true);
        this->declare_parameter("attitude_support_hover_scale", 0.5);
        this->declare_parameter("attitude_support_roll_target", 0.0);
        this->declare_parameter("attitude_support_pitch_target", 0.0);
        this->declare_parameter("attitude_support_roll_kp", 90.0);
        this->declare_parameter("attitude_support_roll_kd", 12.0);
        this->declare_parameter("attitude_support_pitch_kp", 140.0);
        this->declare_parameter("attitude_support_pitch_kd", 18.0);
        this->declare_parameter("attitude_support_max_leg_delta", 24.0);
        this->declare_parameter("crossing_forward_assist_enabled", false);
        this->declare_parameter("crossing_forward_assist_force_per_leg", 14.0);
        this->declare_parameter("crossing_forward_assist_pre_approach_enabled", false);
        this->declare_parameter("crossing_force_full_support", true);
        this->declare_parameter("flat_force_full_support", false);
        this->declare_parameter("flat_forward_assist_enabled", false);
        this->declare_parameter("flat_forward_assist_force_per_mps", 120.0);
        this->declare_parameter("flat_forward_assist_max_force_per_leg", 18.0);
        this->declare_parameter("flat_forward_assist_min_cmd", 0.02);
        // Walking planar stabilization: closes vx/vy/yaw-rate loops through
        // stance-foot tangential forces. Without it flat walking is open-loop
        // (feed-forward push only) and the body veers/yaws freely.
        this->declare_parameter("walking_stabilization_enabled", true);
        this->declare_parameter("walking_vx_kp", 25.0);
        this->declare_parameter("walking_vy_kp", 25.0);
        this->declare_parameter("walking_wz_kp", 4.0);
        this->declare_parameter("walking_fx_max_per_leg", 30.0);
        this->declare_parameter("walking_fy_max_per_leg", 25.0);
        this->declare_parameter("walking_yaw_moment_max", 8.0);
        // Roll/pitch rate damping via common-mode tangential forces
        // (N*m per rad/s, converted through the COM-to-foot height).
        // Default OFF: the odom angular rate is a 20 Hz finite difference
        // that spikes at every touchdown, so this term saturates its clamp
        // in a noise-driven direction each step (run41/42: +-20/30 N
        // common-mode shoves flipped the trunk within the first stride).
        // Only useful with a clean rate source (IMU) + low-pass.
        this->declare_parameter("walking_rate_damping", 0.0);
        this->declare_parameter("walking_rate_force_max", 20.0);
        this->declare_parameter("walking_vel_filter_tau", 0.35);
        this->declare_parameter("crossing_freeze_rail_targets", false);
        this->declare_parameter("bfs_rail_ramp_enabled", true);
        this->declare_parameter("bfs_rail_ramp_rate", 0.015);
        this->declare_parameter("bfs_rail_ramp_slow_scale", 0.25);
        this->declare_parameter("bfs_attitude_gate_roll", 0.30);
        this->declare_parameter("bfs_attitude_gate_pitch", 0.25);
        this->declare_parameter("bfs_attitude_gate_tilt", 0.40);
        this->declare_parameter("bfs_attitude_gate_up_z", 0.85);
        this->declare_parameter("bfs_attitude_gate_angular_rate", 1.20);
        this->declare_parameter("bfs_min_body_z_for_ramp", 0.080);
        this->declare_parameter("bfs_hard_fail_body_z", 0.055);
        
        
        // 获取参数
        mass_ = this->get_parameter("mass").as_double();
        horizon_ = this->get_parameter("horizon").as_int();
        dt_ = this->get_parameter("dt").as_double();
        control_freq_ = this->get_parameter("control_frequency").as_double();
        stance_length_ = this->get_parameter("default_stance_length").as_double();
        stance_width_ = this->get_parameter("default_stance_width").as_double();
        nominal_body_height_ = this->get_parameter("nominal_body_height").as_double();
        com_offset_ << this->get_parameter("com_offset_x").as_double(),
                       this->get_parameter("com_offset_y").as_double(),
                       this->get_parameter("com_offset_z").as_double();
        crossing_window_.x_position =
            this->get_parameter("crossing_window_x_position").as_double();
        crossing_window_.width =
            this->get_parameter("crossing_window_width").as_double();
        crossing_window_.height =
            this->get_parameter("crossing_window_height").as_double();
        crossing_window_.bottom_height =
            this->get_parameter("crossing_window_bottom_height").as_double();
        crossing_window_.top_height =
            this->get_parameter("crossing_window_top_height").as_double();
        crossing_window_.safety_margin =
            this->get_parameter("crossing_window_safety_margin").as_double();
        crossing_activation_distance_ =
            this->get_parameter("crossing_activation_distance").as_double();
        crossing_approach_speed_ =
            this->get_parameter("crossing_approach_speed").as_double();
        refreshVerticalSupportParameters();
        
        std::string mode_str = this->get_parameter("mode").as_string();
        if (mode_str == "walking") {
            current_mode_ = TrajectoryGenerator::Mode::WALKING;
        } else if (mode_str == "crossing") {
            current_mode_ = TrajectoryGenerator::Mode::CROSSING;
        } else {
            current_mode_ = TrajectoryGenerator::Mode::HOVER;
        }
    }
    
    void initializeControllers() {
        loadRobotDescriptionDerivedModelInfo();
        
        // MPC参数
        MPCController::Parameters mpc_params;
        mpc_params.horizon = horizon_;
        mpc_params.dt = dt_;
        mpc_params.enable_sliding_constraints = 
            this->get_parameter("enable_sliding_constraints").as_bool();
        
        // 16维状态权重
        mpc_params.Q = Eigen::MatrixXd::Identity(16, 16);
        mpc_params.Q.diagonal() << 100, 100, 200,  // 位置
                                  50, 50, 50,       // 姿态
                                  10, 10, 10,       // 线速度
                                  5, 5, 5,          // 角速度
                                  50, 50, 50, 50;   // 滑动副
        
        mpc_params.R = Eigen::MatrixXd::Identity(12, 12) * 0.01;
        mpc_params.u_min = Eigen::VectorXd::Constant(12, -100.0);
        mpc_params.u_max = Eigen::VectorXd::Constant(12, 100.0);
        
        // 创建控制器
        mpc_controller_ = std::make_unique<MPCController>(mass_, body_inertia_, mpc_params);
        mpc_controller_->setSlidingPositionLimits(rail_position_lower_, rail_position_upper_);

        // rail soft bound exact penalty 一次项权重（支持动态调参）
        mpc_controller_->setSlackLinearWeight(
            this->get_parameter("slack_linear_weight").as_double());
        mpc_controller_->setFreezeCrossingRailTargets(
            this->get_parameter("crossing_freeze_rail_targets").as_bool());

        // 设置越障状态机 guard 参数（可通过 ROS2 参数服务器调节）
        const double rail_tracking_error_threshold =
            this->get_parameter("rail_tracking_error_threshold").as_double();
        const double support_polygon_margin_threshold =
            this->get_parameter("support_polygon_margin_threshold").as_double();
        const double crossing_transition_stable_time =
            this->get_parameter("crossing_transition_stable_time").as_double();
        mpc_controller_->setCrossingGuardParams(
            rail_tracking_error_threshold,
            support_polygon_margin_threshold,
            crossing_transition_stable_time);

        TrajectoryGenerator::Parameters trajectory_params;
        trajectory_params.default_height = nominal_body_height_;
        trajectory_generator_ = std::make_unique<TrajectoryGenerator>(trajectory_params);
        contact_detector_ = std::make_unique<ContactDetector>();
        gait_generator_ = std::make_unique<HybridGaitGenerator>();
        
        // 基础足端位置来自 robot_description / Dog2Model，顺序显式为 lf,lh,rh,rf。
        mpc_controller_->setBaseFootPositions(base_foot_positions_);

        RCLCPP_INFO(this->get_logger(),
                    "Base feet x (lf,lh,rh,rf) relative to body: [%.3f, %.3f, %.3f, %.3f]",
                    base_foot_positions_(0, 0),
                    base_foot_positions_(1, 0),
                    base_foot_positions_(2, 0),
                    base_foot_positions_(3, 0));
        
        // 初始化滑动副速度
        Eigen::Vector4d sliding_velocity = Eigen::Vector4d::Zero();
        mpc_controller_->setSlidingVelocity(sliding_velocity);
        
        // 设置轨迹生成器模式
        trajectory_generator_->setMode(current_mode_);
    }
    
    void initializePublishersSubscribers() {
        // 订阅
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dog2/odom", 10,
            std::bind(&MPCNodeComplete::odomCallback, this, std::placeholders::_1));
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MPCNodeComplete::jointCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MPCNodeComplete::cmdVelCallback, this, std::placeholders::_1));
        
        enable_crossing_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/enable_crossing", 10,
            std::bind(&MPCNodeComplete::enableCrossingCallback, this, std::placeholders::_1));

        // Contact schedule comes from the gait scheduler so MPC, WBC and the
        // swing generator share one phase clock. The internal phase clock is
        // only a fallback while this stream is stale.
        contact_phase_sub_ = this->create_subscription<dog2_interfaces::msg::ContactPhase>(
            "/dog2/gait/contact_phase", 10,
            std::bind(&MPCNodeComplete::contactPhaseCallback, this, std::placeholders::_1));
        
        // 发布
        foot_force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/dog2/mpc/foot_forces", 10);
        crossing_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/dog2/mpc/crossing_state", 10);
        
        // 控制定时器
        auto timer_period = std::chrono::duration<double>(1.0 / control_freq_);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&MPCNodeComplete::controlLoop, this));
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 提取SRBD状态
        Eigen::VectorXd srbd_state(12);
        
        srbd_state(0) = msg->pose.pose.position.x;
        srbd_state(1) = msg->pose.pose.position.y;
        srbd_state(2) = msg->pose.pose.position.z;
        
        // 四元数转欧拉角
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        Eigen::Quaterniond q_wb(qw, qx, qy, qz);
        if (std::isfinite(q_wb.w()) &&
            std::isfinite(q_wb.x()) &&
            std::isfinite(q_wb.y()) &&
            std::isfinite(q_wb.z()) &&
            q_wb.norm() > 1e-6) {
            current_body_q_wb_ = q_wb.normalized();
            current_body_q_valid_ = true;
            const LevelAttitudeError level_err =
                computeLevelAttitudeErrorFromQuat(current_body_q_wb_);
            if (level_err.valid) {
                last_level_roll_like_ = level_err.roll_like;
                last_level_pitch_like_ = level_err.pitch_like;
                last_level_tilt_ = level_err.tilt;
                last_body_up_z_ = level_err.body_up_z;
            }
        }
        
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        srbd_state(3) = std::atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2.0 * (qw * qy - qz * qx);
        srbd_state(4) = std::abs(sinp) >= 1 ? 
            std::copysign(M_PI / 2, sinp) : std::asin(sinp);
        
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        srbd_state(5) = std::atan2(siny_cosp, cosy_cosp);
        
        srbd_state(6) = msg->twist.twist.linear.x;
        srbd_state(7) = msg->twist.twist.linear.y;
        srbd_state(8) = msg->twist.twist.linear.z;
        srbd_state(9) = msg->twist.twist.angular.x;
        srbd_state(10) = msg->twist.twist.angular.y;
        srbd_state(11) = msg->twist.twist.angular.z;
        
        current_srbd_state_ = srbd_state;
        odom_received_ = true;
    }
    
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        Eigen::Vector4d sliding_positions = Eigen::Vector4d::Zero();
        Eigen::Vector4d sliding_velocities = current_sliding_velocities_;
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            double vel = 0.0;
            if (i < msg->velocity.size()) {
                vel = msg->velocity[i];
            }

            if (msg->name[i] == "j1" || msg->name[i] == "lf_rail_joint") {
                sliding_positions(0) = msg->position[i];
                sliding_velocities(0) = vel;
            } else if (msg->name[i] == "j2" || msg->name[i] == "lh_rail_joint") {
                sliding_positions(1) = msg->position[i];
                sliding_velocities(1) = vel;
            } else if (msg->name[i] == "j3" || msg->name[i] == "rh_rail_joint") {
                sliding_positions(2) = msg->position[i];
                sliding_velocities(2) = vel;
            } else if (msg->name[i] == "j4" || msg->name[i] == "rf_rail_joint") {
                sliding_positions(3) = msg->position[i];
                sliding_velocities(3) = vel;
            } else if (msg->name[i].find("hip_roll") != std::string::npos ||
                       msg->name[i].find("_haa_joint") != std::string::npos ||
                       msg->name[i].find("_coxa_joint") != std::string::npos) {
                int leg = getLegIdFromName(msg->name[i]);
                if (leg >= 0) current_joint_angles_[leg](0) = msg->position[i];
            } else if (msg->name[i].find("hip_pitch") != std::string::npos ||
                       msg->name[i].find("_hfe_joint") != std::string::npos ||
                       msg->name[i].find("_femur_joint") != std::string::npos) {
                int leg = getLegIdFromName(msg->name[i]);
                if (leg >= 0) current_joint_angles_[leg](1) = msg->position[i];
            } else if (msg->name[i].find("knee") != std::string::npos ||
                       msg->name[i].find("_kfe_joint") != std::string::npos ||
                       msg->name[i].find("_tibia_joint") != std::string::npos) {
                int leg = getLegIdFromName(msg->name[i]);
                if (leg >= 0) {
                    current_joint_angles_[leg](2) = msg->position[i];
                    leg_joint_received_[leg] = true;
                }
            }
        }
        
        current_sliding_positions_ = sliding_positions;
        current_sliding_velocities_ =
            sanitizeSlidingVelocities(sliding_positions, sliding_velocities);
        mpc_controller_->setSlidingVelocity(current_sliding_velocities_);
        measured_leg_configs_valid_ =
            leg_joint_received_[0] &&
            leg_joint_received_[1] &&
            leg_joint_received_[2] &&
            leg_joint_received_[3];
        joint_received_ = true;
    }

    std::array<CrossingStateMachine::LegConfiguration, 4> estimateMeasuredLegConfigurations() const {
        std::array<CrossingStateMachine::LegConfiguration, 4> configs;
        for (int i = 0; i < 4; ++i) {
            if (leg_joint_received_[i]) {
                configs[i] = (current_joint_angles_[i](2) < 0.0)
                    ? CrossingStateMachine::LegConfiguration::ELBOW
                    : CrossingStateMachine::LegConfiguration::KNEE;
            } else {
                configs[i] = CrossingStateMachine::LegConfiguration::ELBOW;
            }
        }
        return configs;
    }

    Eigen::Vector4d sanitizeSlidingVelocities(const Eigen::Vector4d& positions,
                                              const Eigen::Vector4d& velocities) const {
        // The simplified SRBD MPC treats rail velocity as a known exogenous input.
        // In WALKING/PRE_APPROACH, measured rail velocities are often controller
        // transient/noise and can make the hard physical rail bounds infeasible.
        // Real rail motion is only required after the crossing controller is initialized.
        const bool initialized_crossing =
            current_mode_ == TrajectoryGenerator::Mode::CROSSING &&
            mpc_controller_ &&
            mpc_controller_->isCrossingEnabled();

        if (!initialized_crossing) {
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d sanitized = velocities;
        const Eigen::Vector4d& rail_min = rail_position_lower_;
        const Eigen::Vector4d& rail_max = rail_position_upper_;
        constexpr double kLimitMargin = 0.002;
        constexpr double kMaxPredictionVelocity = 0.25;

        for (int i = 0; i < 4; ++i) {
            sanitized(i) = std::max(
                -kMaxPredictionVelocity,
                std::min(sanitized(i), kMaxPredictionVelocity));
            if (positions(i) <= rail_min(i) + kLimitMargin && sanitized(i) < 0.0) {
                sanitized(i) = 0.0;
            }
            if (positions(i) >= rail_max(i) - kLimitMargin && sanitized(i) > 0.0) {
                sanitized(i) = 0.0;
            }
        }

        return sanitized;
    }
    
    static int legIdFromShortName(const std::string& name) {
        // Gait scheduler publishes bare leg names ("lf"), while joint-state
        // parsing expects prefixed names ("lf_..."); accept both.
        if (name == "lf") return 0;
        if (name == "lh") return 1;
        if (name == "rh") return 2;
        if (name == "rf") return 3;
        return -1;
    }

    void contactPhaseCallback(const dog2_interfaces::msg::ContactPhase::SharedPtr msg) {
        const size_t count = std::min(msg->leg_names.size(), msg->phase.size());
        for (size_t i = 0; i < count; ++i) {
            int leg = legIdFromShortName(msg->leg_names[i]);
            if (leg < 0) {
                leg = getLegIdFromName(msg->leg_names[i]);
            }
            if (leg >= 0) {
                gait_contact_mask_[leg] =
                    msg->phase[i] != dog2_interfaces::msg::ContactPhase::SWING;
            }
        }
        gait_mask_stamp_ = this->get_clock()->now();
        gait_mask_received_ = true;
    }

    bool gaitMaskFresh() {
        if (!gait_mask_received_) {
            return false;
        }
        const double age =
            (this->get_clock()->now() - gait_mask_stamp_).seconds();
        return age >= 0.0 && age < 0.3;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        velocity_cmd_target_(0) = msg->linear.x;
        velocity_cmd_target_(1) = msg->linear.y;
        velocity_cmd_target_(2) = msg->angular.z;

        // 如果收到速度命令，切换到行走模式
        if (velocity_cmd_target_.norm() > 0.01 &&
            current_mode_ == TrajectoryGenerator::Mode::HOVER) {
            current_mode_ = TrajectoryGenerator::Mode::WALKING;
            trajectory_generator_->setMode(current_mode_);
            RCLCPP_INFO(this->get_logger(), "Switched to WALKING mode");
        }
    }

    // Slew-limit the velocity command actually served to the trajectory
    // generator and the planar velocity loop. A step to 0.12 m/s at walk
    // onset demands full tracking force on the very first trot pair while
    // the trunk is still settling into the gait; ramping over ~0.5 s lets
    // the robot start stepping in place and accelerate smoothly.
    void slewVelocityCommand() {
        auto slew = [this](double current, double target, double rate) {
            const double max_step = rate * dt_;
            return current + clampDouble(target - current, -max_step, max_step);
        };
        velocity_cmd_(0) =
            slew(velocity_cmd_(0), velocity_cmd_target_(0), cmd_linear_slew_rate_);
        velocity_cmd_(1) =
            slew(velocity_cmd_(1), velocity_cmd_target_(1), cmd_linear_slew_rate_);
        velocity_cmd_(2) =
            slew(velocity_cmd_(2), velocity_cmd_target_(2), cmd_angular_slew_rate_);
    }

    bool hasCrossingState() const {
        return odom_received_ && joint_received_ && current_srbd_state_.size() >= 12;
    }

    void enableCrossingMode() {
        current_mode_ = TrajectoryGenerator::Mode::CROSSING;
        trajectory_generator_->setMode(current_mode_);
        crossing_enabled_ = true;
        publishCrossingState();

        RCLCPP_INFO(this->get_logger(),
                    "Crossing mode ENABLED: window_x=%.3f width=%.3f top=%.3f safety=%.3f activation_distance=%.3f",
                    crossing_window_.x_position,
                    crossing_window_.width,
                    crossing_window_.top_height,
                    crossing_window_.safety_margin,
                    crossing_activation_distance_);
    }
    
    void enableCrossingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || current_mode_ == TrajectoryGenerator::Mode::CROSSING) {
            return;
        }

        if (!hasCrossingState()) {
            pending_crossing_request_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "Crossing request received before state was ready; deferring mode switch");
            return;
        }

        pending_crossing_request_ = false;
        enableCrossingMode();
    }
    
    void controlLoop() {
        if (!odom_received_ || !joint_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Waiting for state...");
            return;
        }

        if (pending_crossing_request_ &&
            current_mode_ != TrajectoryGenerator::Mode::CROSSING) {
            enableCrossingMode();
            pending_crossing_request_ = false;
        }

        // 动态读取 exact penalty 一次项权重
        mpc_controller_->setSlackLinearWeight(
            this->get_parameter("slack_linear_weight").as_double());
        refreshVerticalSupportParameters();
        updateSupportGeometry();
        slewVelocityCommand();
        
        // 构建16维扩展状态
        Eigen::VectorXd extended_state(16);
        extended_state.segment<12>(0) = current_srbd_state_;
        extended_state.segment<4>(12) = current_sliding_positions_;
        
        // 生成参考轨迹
        std::vector<Eigen::VectorXd> x_ref;
        
        bool crossing_pre_approach = false;
        switch (current_mode_) {
            case TrajectoryGenerator::Mode::HOVER:
                x_ref = trajectory_generator_->generateHoverTrajectory(
                    extended_state, horizon_, dt_);
                break;
                
            case TrajectoryGenerator::Mode::WALKING: {
                advanceGaitPhase();
                
                HybridGaitGenerator::GaitState gait_state;
                // GaitState不需要phase成员，直接传递
                
                x_ref = trajectory_generator_->generateWalkingTrajectory(
                    extended_state, velocity_cmd_, gait_state, horizon_, dt_);
                break;
            }
                
            case TrajectoryGenerator::Mode::CROSSING: {
                if (!mpc_controller_->isCrossingEnabled()) {
                    const double activation_x =
                        crossing_window_.x_position - crossing_activation_distance_;
                    if (current_srbd_state_(0) < activation_x) {
                        advanceGaitPhase();
                        HybridGaitGenerator::GaitState gait_state;
                        Eigen::Vector3d approach_velocity(
                            crossing_approach_speed_, 0.0, 0.0);
                        x_ref = trajectory_generator_->generateWalkingTrajectory(
                            extended_state, approach_velocity, gait_state, horizon_, dt_);
                        crossing_pre_approach = true;
                        break;
                    }

                    mpc_controller_->initializeCrossing(buildCrossingRobotState(), crossing_window_);
                    RCLCPP_INFO(this->get_logger(),
                                "Initialized controller crossing at x=%.3f (window_x=%.3f)",
                                current_srbd_state_(0),
                                crossing_window_.x_position);
                }

                // Crossing 主参考由 MPCController 内部状态机维护，这里仅保持中性轨迹。
                x_ref = trajectory_generator_->generateHoverTrajectory(
                    extended_state, horizon_, dt_);
                break;
            }
        }
        
        // 设置MPC参考轨迹
        mpc_controller_->setReference(x_ref);
        updateBfsRailGate();

        const bool use_gait_contact_mask =
            (current_mode_ == TrajectoryGenerator::Mode::WALKING || crossing_pre_approach);
        ContactDetector::ContactState contact_state =
            computeCommandContactState(crossing_pre_approach);
        
        // 注入真实腿部构型（让状态机使用真实测量值而非 stage 期望值）
        {
            auto measured_configs = estimateMeasuredLegConfigurations();
            mpc_controller_->setMeasuredLegConfigurations(measured_configs, measured_leg_configs_valid_);
        }
        {
            auto mc = estimateMeasuredLegConfigurations();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Leg configs: valid=%d lf=%s lh=%s rh=%s rf=%s tibia=[%.3f,%.3f,%.3f,%.3f]",
                       measured_leg_configs_valid_ ? 1 : 0,
                       (mc[0] == CrossingStateMachine::LegConfiguration::ELBOW ? "E" : "K"),
                       (mc[1] == CrossingStateMachine::LegConfiguration::ELBOW ? "E" : "K"),
                       (mc[2] == CrossingStateMachine::LegConfiguration::ELBOW ? "E" : "K"),
                       (mc[3] == CrossingStateMachine::LegConfiguration::ELBOW ? "E" : "K"),
                       current_joint_angles_[0](2),
                       current_joint_angles_[1](2),
                       current_joint_angles_[2](2),
                       current_joint_angles_[3](2));
        }

        // 求解MPC
        Eigen::VectorXd u_optimal;
        bool success = mpc_controller_->solve(extended_state, u_optimal);
        
        if (!success) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "MPC solve failed, publishing vertical support fallback");
            u_optimal = Eigen::VectorXd::Zero(12);
            applyVerticalSupport(u_optimal, contact_state, use_gait_contact_mask,
                                 crossing_pre_approach);
            applyAttitudeSupport(u_optimal, contact_state, use_gait_contact_mask,
                                 crossing_pre_approach);
            applyCrossingForwardAssist(u_optimal, contact_state, use_gait_contact_mask,
                                       crossing_pre_approach);
            applyFlatForwardAssist(u_optimal, contact_state, use_gait_contact_mask,
                                   crossing_pre_approach);
            applyWalkingPlanarStabilization(u_optimal, contact_state,
                                            use_gait_contact_mask,
                                            crossing_pre_approach);
            applyHoverForceSanitizer(u_optimal);
            publishCrossingState();
            publishFootForces(u_optimal);
            return;
        }

        publishCrossingState();
        
        if (use_gait_contact_mask) {
            zeroSwingLegForces(u_optimal, contact_state);
        }
        const Eigen::VectorXd u_qp_snapshot = u_optimal;
        applyVerticalSupport(u_optimal, contact_state, use_gait_contact_mask,
                             crossing_pre_approach);
        applyAttitudeSupport(u_optimal, contact_state, use_gait_contact_mask,
                             crossing_pre_approach);
        applyCrossingForwardAssist(u_optimal, contact_state, use_gait_contact_mask,
                                   crossing_pre_approach);
        applyFlatForwardAssist(u_optimal, contact_state, use_gait_contact_mask,
                               crossing_pre_approach);
        applyWalkingPlanarStabilization(u_optimal, contact_state,
                                        use_gait_contact_mask,
                                        crossing_pre_approach);
        applyHoverForceSanitizer(u_optimal);
        if (current_mode_ == TrajectoryGenerator::Mode::WALKING) {
            // Per-leg planar force breakdown: QP raw vs after shapers, to
            // attribute the walking lurch (QP planar output vs velocity
            // stabilization shares).
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[walk_fx] qp=[%.1f %.1f %.1f %.1f] out=[%.1f %.1f %.1f %.1f] "
                "qp_fy=[%.1f %.1f %.1f %.1f] out_fy=[%.1f %.1f %.1f %.1f] mask=[%d%d%d%d]",
                u_qp_snapshot(0), u_qp_snapshot(3), u_qp_snapshot(6), u_qp_snapshot(9),
                u_optimal(0), u_optimal(3), u_optimal(6), u_optimal(9),
                u_qp_snapshot(1), u_qp_snapshot(4), u_qp_snapshot(7), u_qp_snapshot(10),
                u_optimal(1), u_optimal(4), u_optimal(7), u_optimal(10),
                contact_state.in_contact[0] ? 1 : 0,
                contact_state.in_contact[1] ? 1 : 0,
                contact_state.in_contact[2] ? 1 : 0,
                contact_state.in_contact[3] ? 1 : 0);
        }
        
        // 发布足端力
        publishFootForces(u_optimal);
        
        // 统计
        if (++control_count_ % 20 == 0) {
            const std::string crossing_stage =
                (current_mode_ == TrajectoryGenerator::Mode::CROSSING && mpc_controller_->isCrossingEnabled())
                    ? crossingStateKey(mpc_controller_->getCurrentCrossingState())
                    : "NONE";
            const double total_fz =
                u_optimal(2) + u_optimal(5) + u_optimal(8) + u_optimal(11);
            RCLCPP_INFO(this->get_logger(),
                       "MPC: t=%.2fms, h=%.3fm, vz=%.3fm/s, fz=%.1fN, mode=%s, stage=%s, full_support=%d, freeze_rails=%d, bfs_freeze=%d, rails=[%.3f,%.3f,%.3f,%.3f], contacts=[%d,%d,%d,%d]",
                       mpc_controller_->getSolveTime(),
                       current_srbd_state_(2),
                       current_srbd_state_(8),
                       total_fz,
                       getModeString().c_str(),
                       crossing_stage.c_str(),
                       crossing_force_full_support_ ? 1 : 0,
                       crossing_freeze_rail_targets_ ? 1 : 0,
                       last_bfs_rail_freeze_ ? 1 : 0,
                       current_sliding_positions_(0),
                       current_sliding_positions_(1),
                       current_sliding_positions_(2),
                       current_sliding_positions_(3),
                       contact_state.in_contact[0],
                       contact_state.in_contact[1],
                       contact_state.in_contact[2],
                       contact_state.in_contact[3]);
            RCLCPP_INFO(this->get_logger(),
                       "MPC attitude: euler=[%.3f,%.3f] level=[%.3f,%.3f] tilt=%.3f up_z=%.3f wx=%.3f wy=%.3f att_dz=[%.1f,%.1f,%.1f,%.1f] err=[%.3f,%.3f]",
                       current_srbd_state_(3),
                       current_srbd_state_(4),
                       last_level_roll_like_,
                       last_level_pitch_like_,
                       last_level_tilt_,
                       last_body_up_z_,
                       current_srbd_state_(9),
                       current_srbd_state_(10),
                       last_attitude_support_delta_(0),
                       last_attitude_support_delta_(1),
                       last_attitude_support_delta_(2),
                       last_attitude_support_delta_(3),
                       last_roll_error_,
                       last_pitch_error_);
            logFootForceBreakdown(u_optimal, contact_state, use_gait_contact_mask);
        }
    }

    void refreshVerticalSupportParameters() {
        vertical_support_enabled_ =
            this->get_parameter("vertical_support_enabled").as_bool();
        vertical_support_target_height_ =
            this->get_parameter("vertical_support_target_height").as_double();
        vertical_support_kp_ =
            this->get_parameter("vertical_support_kp").as_double();
        vertical_support_kd_ =
            this->get_parameter("vertical_support_kd").as_double();
        // No 1.0 floor here: values below 1.0 are the whole point -- the
        // height regulator must be able to command less than body weight
        // to decelerate an ascent (see applyVerticalSupport).
        vertical_support_min_total_force_multiplier_ =
            std::max(0.0, this->get_parameter("vertical_support_min_total_force_multiplier").as_double());
        vertical_support_max_leg_force_ =
            std::max(0.0, this->get_parameter("vertical_support_max_leg_force").as_double());
        vertical_support_height_error_limit_ =
            std::max(0.0, this->get_parameter("vertical_support_height_error_limit").as_double());
        vertical_support_load_rate_ =
            std::max(0.0, this->get_parameter("vertical_support_load_rate").as_double());
        support_foot_height_gate_ =
            std::max(0.0, this->get_parameter("support_foot_height_gate").as_double());
        hover_force_sanitize_enabled_ =
            this->get_parameter("hover_force_sanitize_enabled").as_bool();
        hover_force_max_leg_fz_ =
            std::max(0.0, this->get_parameter("hover_force_max_leg_fz").as_double());
        hover_force_min_leg_fz_ =
            clampDouble(
                this->get_parameter("hover_force_min_leg_fz").as_double(),
                0.0,
                hover_force_max_leg_fz_);
        attitude_support_enabled_ =
            this->get_parameter("attitude_support_enabled").as_bool();
        attitude_support_hover_enabled_ =
            this->get_parameter("attitude_support_hover_enabled").as_bool();
        attitude_support_hover_scale_ = clampDouble(
            this->get_parameter("attitude_support_hover_scale").as_double(),
            0.0, 1.0);
        attitude_support_roll_target_ =
            this->get_parameter("attitude_support_roll_target").as_double();
        attitude_support_pitch_target_ =
            this->get_parameter("attitude_support_pitch_target").as_double();
        attitude_support_roll_kp_ =
            std::max(0.0, this->get_parameter("attitude_support_roll_kp").as_double());
        attitude_support_roll_kd_ =
            std::max(0.0, this->get_parameter("attitude_support_roll_kd").as_double());
        attitude_support_pitch_kp_ =
            std::max(0.0, this->get_parameter("attitude_support_pitch_kp").as_double());
        attitude_support_pitch_kd_ =
            std::max(0.0, this->get_parameter("attitude_support_pitch_kd").as_double());
        attitude_support_max_leg_delta_ =
            std::max(0.0, this->get_parameter("attitude_support_max_leg_delta").as_double());
        crossing_forward_assist_enabled_ =
            this->get_parameter("crossing_forward_assist_enabled").as_bool();
        crossing_forward_assist_force_per_leg_ =
            std::max(0.0, this->get_parameter("crossing_forward_assist_force_per_leg").as_double());
        crossing_forward_assist_pre_approach_enabled_ =
            this->get_parameter("crossing_forward_assist_pre_approach_enabled").as_bool();
        crossing_force_full_support_ =
            this->get_parameter("crossing_force_full_support").as_bool();
        flat_force_full_support_ =
            this->get_parameter("flat_force_full_support").as_bool();
        flat_forward_assist_enabled_ =
            this->get_parameter("flat_forward_assist_enabled").as_bool();
        flat_forward_assist_force_per_mps_ =
            std::max(0.0, this->get_parameter("flat_forward_assist_force_per_mps").as_double());
        flat_forward_assist_max_force_per_leg_ =
            std::max(0.0, this->get_parameter("flat_forward_assist_max_force_per_leg").as_double());
        flat_forward_assist_min_cmd_ =
            std::max(0.0, this->get_parameter("flat_forward_assist_min_cmd").as_double());
        walking_stabilization_enabled_ =
            this->get_parameter("walking_stabilization_enabled").as_bool();
        walking_vx_kp_ =
            std::max(0.0, this->get_parameter("walking_vx_kp").as_double());
        walking_vy_kp_ =
            std::max(0.0, this->get_parameter("walking_vy_kp").as_double());
        walking_wz_kp_ =
            std::max(0.0, this->get_parameter("walking_wz_kp").as_double());
        walking_fx_max_per_leg_ =
            std::max(0.0, this->get_parameter("walking_fx_max_per_leg").as_double());
        walking_fy_max_per_leg_ =
            std::max(0.0, this->get_parameter("walking_fy_max_per_leg").as_double());
        walking_yaw_moment_max_ =
            std::max(0.0, this->get_parameter("walking_yaw_moment_max").as_double());
        walking_rate_damping_ =
            std::max(0.0, this->get_parameter("walking_rate_damping").as_double());
        walking_rate_force_max_ =
            std::max(0.0, this->get_parameter("walking_rate_force_max").as_double());
        walking_vel_filter_tau_ =
            std::max(0.0, this->get_parameter("walking_vel_filter_tau").as_double());
        crossing_freeze_rail_targets_ =
            this->get_parameter("crossing_freeze_rail_targets").as_bool();
        bfs_rail_ramp_enabled_ =
            this->get_parameter("bfs_rail_ramp_enabled").as_bool();
        bfs_rail_ramp_rate_ =
            std::max(0.0, this->get_parameter("bfs_rail_ramp_rate").as_double());
        bfs_rail_ramp_slow_scale_ =
            clampDouble(this->get_parameter("bfs_rail_ramp_slow_scale").as_double(), 0.0, 1.0);
        bfs_attitude_gate_roll_ =
            std::max(0.0, this->get_parameter("bfs_attitude_gate_roll").as_double());
        bfs_attitude_gate_pitch_ =
            std::max(0.0, this->get_parameter("bfs_attitude_gate_pitch").as_double());
        bfs_attitude_gate_tilt_ =
            std::max(0.0, this->get_parameter("bfs_attitude_gate_tilt").as_double());
        bfs_attitude_gate_up_z_ =
            clampDouble(this->get_parameter("bfs_attitude_gate_up_z").as_double(), -1.0, 1.0);
        bfs_attitude_gate_angular_rate_ =
            std::max(0.0, this->get_parameter("bfs_attitude_gate_angular_rate").as_double());
        bfs_min_body_z_for_ramp_ =
            std::max(0.0, this->get_parameter("bfs_min_body_z_for_ramp").as_double());
        bfs_hard_fail_body_z_ =
            std::max(0.0, this->get_parameter("bfs_hard_fail_body_z").as_double());
        if (mpc_controller_) {
            mpc_controller_->setFreezeCrossingRailTargets(crossing_freeze_rail_targets_);
            mpc_controller_->setCrossingBfsRailGate(
                bfs_rail_ramp_enabled_,
                last_bfs_rail_freeze_,
                last_bfs_rail_ramp_scale_,
                bfs_rail_ramp_rate_);
        }
    }

    // Lever arms for the support force shapers (vertical split, attitude
    // deltas, yaw-moment fx split): measured-FK feet relative to the
    // measured-FK COM, rotated into world axes.
    //
    // The old code took the constant stance geometry (base_foot_positions_ /
    // com_stance_) as moment arms. During stand-up the measured feet sit
    // >10 cm away from the stance feet (deep crouch), and at tilt ~0.24 rad
    // the base-frame z offset of the feet leaks ~sin(tilt)*0.15 m into the
    // world-horizontal lever. Both errors made the moment balance
    // systematically wrong and kept pumping the settle limit cycle
    // (run20-26). Falls back to stance geometry until joints arrive.
    void updateSupportGeometry() {
        Eigen::Matrix3d R_wb = Eigen::Matrix3d::Identity();
        if (current_body_q_valid_ && current_body_q_wb_.norm() > 1e-6) {
            R_wb = current_body_q_wb_.normalized().toRotationMatrix();
        }

        support_geometry_from_fk_ = false;
        support_foot_airborne_.fill(false);
        if (dog2_model_ && runtime_fk_ready_ &&
            joint_received_ && measured_leg_configs_valid_) {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(dog2_model_->nq());
            for (int leg = 0; leg < 4; ++leg) {
                q(fk_q_index_[leg][0]) = current_sliding_positions_(leg);
                q(fk_q_index_[leg][1]) = current_joint_angles_[leg](0);
                q(fk_q_index_[leg][2]) = current_joint_angles_[leg](1);
                q(fk_q_index_[leg][3]) = current_joint_angles_[leg](2);
            }
            const Eigen::Vector3d com_b = dog2_model_->centerOfMass(q);
            for (int i = 0; i < 4; ++i) {
                const Eigen::Vector3d foot_b =
                    dog2_model_->footPosition(kMpcFootFrames[i], q);
                support_lever_arms_.row(i) =
                    (R_wb * (foot_b - com_b)).transpose();
            }
            support_geometry_from_fk_ = true;

            // Physical contact gate for the clock-driven schedule: feet on
            // the ground share one world plane, so their COM-relative
            // world z levers are equal regardless of body tilt. A
            // scheduled-stance foot riding well above the lowest foot is
            // physically airborne (late touchdown, tip recovery, splayed
            // leg) and must not be budgeted support force: loading it is
            // a free-fall on that corner (run56 turn entry: fz=[67,67,0,0]
            // with one foot 0.6 m out -> roll flip).
            if (support_foot_height_gate_ > 0.0) {
                double lowest = support_lever_arms_(0, 2);
                for (int i = 1; i < 4; ++i) {
                    lowest = std::min(lowest, support_lever_arms_(i, 2));
                }
                for (int i = 0; i < 4; ++i) {
                    support_foot_airborne_[i] =
                        (support_lever_arms_(i, 2) - lowest) >
                        support_foot_height_gate_;
                }
            }
        } else {
            for (int i = 0; i < 4; ++i) {
                const Eigen::Vector3d d_b =
                    base_foot_positions_.row(i).transpose() - com_stance_;
                support_lever_arms_.row(i) = (R_wb * d_b).transpose();
            }
        }

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[support_geom] src=%s lever_x=[%.3f %.3f %.3f %.3f] lever_y=[%.3f %.3f %.3f %.3f]",
            support_geometry_from_fk_ ? "fk" : "stance",
            support_lever_arms_(0, 0), support_lever_arms_(1, 0),
            support_lever_arms_(2, 0), support_lever_arms_(3, 0),
            support_lever_arms_(0, 1), support_lever_arms_(1, 1),
            support_lever_arms_(2, 1), support_lever_arms_(3, 1));
    }

    LevelAttitudeError computeLevelAttitudeErrorFromQuat(
        const Eigen::Quaterniond& q_wb) const {
        LevelAttitudeError out;
        if (q_wb.norm() < 1e-6) {
            return out;
        }

        const Eigen::Quaterniond q = q_wb.normalized();
        const Eigen::Matrix3d R_wb = q.toRotationMatrix();
        const Eigen::Vector3d body_z_w = R_wb.col(2);
        const double up_z = clampDouble(body_z_w.z(), -1.0, 1.0);
        const double lateral_norm =
            std::sqrt(body_z_w.x() * body_z_w.x() +
                      body_z_w.y() * body_z_w.y());

        out.body_up_z = up_z;
        out.tilt = std::atan2(lateral_norm, up_z);
        const double denom = std::max(0.15, std::abs(up_z));
        out.roll_like = std::atan2(-body_z_w.y(), denom);
        out.pitch_like = std::atan2(body_z_w.x(), denom);
        out.inverted = up_z < 0.2;
        out.valid = true;
        return out;
    }

    void updateBfsRailGate() {
        if (!mpc_controller_) {
            return;
        }

        const LevelAttitudeError level_err =
            current_body_q_valid_
                ? computeLevelAttitudeErrorFromQuat(current_body_q_wb_)
                : LevelAttitudeError{};
        if (level_err.valid) {
            last_level_roll_like_ = level_err.roll_like;
            last_level_pitch_like_ = level_err.pitch_like;
            last_level_tilt_ = level_err.tilt;
            last_body_up_z_ = level_err.body_up_z;
            mpc_controller_->setLevelAttitudeState(
                true,
                level_err.roll_like,
                level_err.pitch_like,
                level_err.tilt,
                level_err.body_up_z);
        } else {
            mpc_controller_->setLevelAttitudeState(false, 0.0, 0.0, 0.0, 1.0);
        }

        bool freeze = false;
        double ramp_scale = 1.0;

        if (!bfs_rail_ramp_enabled_ ||
            current_mode_ != TrajectoryGenerator::Mode::CROSSING ||
            !mpc_controller_->isCrossingEnabled()) {
            last_bfs_rail_freeze_ = false;
            last_bfs_rail_ramp_scale_ = 1.0;
            mpc_controller_->setCrossingBfsRailGate(
                bfs_rail_ramp_enabled_, false, 1.0, bfs_rail_ramp_rate_);
            return;
        }

        const auto stage = mpc_controller_->getCurrentCrossingState();
        if (stage != CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT) {
            last_bfs_rail_freeze_ = false;
            last_bfs_rail_ramp_scale_ = 1.0;
            mpc_controller_->setCrossingBfsRailGate(
                bfs_rail_ramp_enabled_, false, 1.0, bfs_rail_ramp_rate_);
            return;
        }

        const double wx = current_srbd_state_.size() > 10 ? current_srbd_state_(9) : 0.0;
        const double wy = current_srbd_state_.size() > 10 ? current_srbd_state_(10) : 0.0;
        const double body_z = current_srbd_state_.size() > 2 ? current_srbd_state_(2) : 0.0;

        if (!level_err.valid || bfs_rail_ramp_rate_ <= 0.0) {
            freeze = true;
            ramp_scale = 0.0;
        } else {
            const bool attitude_bad =
                std::abs(level_err.roll_like) > bfs_attitude_gate_roll_ ||
                std::abs(level_err.pitch_like) > bfs_attitude_gate_pitch_ ||
                level_err.tilt > bfs_attitude_gate_tilt_ ||
                level_err.body_up_z < bfs_attitude_gate_up_z_ ||
                std::abs(wx) > bfs_attitude_gate_angular_rate_ ||
                std::abs(wy) > bfs_attitude_gate_angular_rate_ ||
                level_err.inverted;
            const bool height_bad = body_z < bfs_min_body_z_for_ramp_;
            freeze = attitude_bad || height_bad;
            ramp_scale = freeze ? 0.0 : 1.0;
            if (!freeze && level_err.tilt > 0.25) {
                ramp_scale = bfs_rail_ramp_slow_scale_;
            }
        }

        if (body_z < bfs_hard_fail_body_z_) {
            freeze = true;
            ramp_scale = 0.0;
        }

        last_bfs_rail_freeze_ = freeze;
        last_bfs_rail_ramp_scale_ = ramp_scale;
        mpc_controller_->setCrossingBfsRailGate(
            bfs_rail_ramp_enabled_, freeze, ramp_scale, bfs_rail_ramp_rate_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "[BFS rail_gate] freeze=%d ramp_scale=%.2f roll_like=%.3f pitch_like=%.3f tilt=%.3f up_z=%.3f z=%.3f wx=%.3f wy=%.3f",
            freeze ? 1 : 0,
            ramp_scale,
            level_err.roll_like,
            level_err.pitch_like,
            level_err.tilt,
            level_err.body_up_z,
            body_z,
            wx,
            wy);
    }

    bool isCrossingSupportStabilizationActive(bool crossing_pre_approach) const {
        if (current_mode_ != TrajectoryGenerator::Mode::CROSSING) {
            return false;
        }

        if (crossing_pre_approach) {
            return true;
        }

        if (!mpc_controller_ || !mpc_controller_->isCrossingEnabled()) {
            return false;
        }

        const auto stage = mpc_controller_->getCurrentCrossingState();
        return stage == CrossingStateMachine::CrossingState::APPROACH ||
               stage == CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT;
    }

    // Attitude (roll/pitch) support through fz differentials, for phases
    // where stance forces carry the body: flat walking, crossing staging,
    // and (since the crouch-handoff stand-up) HOVER as well. HOVER used to
    // rely solely on the WBC posture PD, which is joint-space and blind to
    // trunk attitude - a pitched quiet stand got zero corrective action
    // (run15-17: att_dz stayed [0,0,0,0] throughout). HOVER uses a scaled
    // gain (attitude_support_hover_scale) so quiet standing does not grind
    // the feet against friction.
    bool isSupportStabilizationActive(bool crossing_pre_approach) const {
        if (current_mode_ == TrajectoryGenerator::Mode::WALKING) {
            return true;
        }
        if (attitude_support_hover_enabled_ &&
            current_mode_ == TrajectoryGenerator::Mode::HOVER) {
            return true;
        }
        return isCrossingSupportStabilizationActive(crossing_pre_approach);
    }

    double attitudeSupportGainScale() const {
        if (current_mode_ == TrajectoryGenerator::Mode::HOVER) {
            return attitude_support_hover_scale_;
        }
        return 1.0;
    }

    ContactDetector::ContactState computeCommandContactState(bool crossing_pre_approach) {
        ContactDetector::ContactState contact_state;
        if (current_mode_ == TrajectoryGenerator::Mode::WALKING || crossing_pre_approach) {
            if (gaitMaskFresh()) {
                // Same clock as WBC/swing generator: the WBC only realizes
                // stance forces on these legs, so force budgeting must use
                // this mask or the vertical support total comes out short.
                for (int i = 0; i < 4; ++i) {
                    contact_state.in_contact[i] = gait_contact_mask_[i];
                }
            } else {
                std::array<double, 4> gait_phases;
                for (int i = 0; i < 4; ++i) {
                    gait_phases[i] = gait_phase_ + (i % 2) * 0.5;
                    if (gait_phases[i] >= 1.0) gait_phases[i] -= 1.0;
                }
                contact_state = contact_detector_->detectFromGait(gait_phases);
            }

            // Schedule AND measured reality: drop scheduled-stance feet the
            // FK height gate says are airborne, unless that would leave
            // fewer than two supports.
            if (support_geometry_from_fk_) {
                int gated_count = 0;
                for (int i = 0; i < 4; ++i) {
                    if (contact_state.in_contact[i] && !support_foot_airborne_[i]) {
                        ++gated_count;
                    }
                }
                if (gated_count >= 2) {
                    for (int i = 0; i < 4; ++i) {
                        contact_state.in_contact[i] =
                            contact_state.in_contact[i] && !support_foot_airborne_[i];
                    }
                }
            }
        }

        if (crossing_force_full_support_ &&
            isCrossingSupportStabilizationActive(crossing_pre_approach)) {
            contact_state.in_contact.fill(true);
        }
        if (flat_force_full_support_ &&
            current_mode_ == TrajectoryGenerator::Mode::WALKING &&
            !crossing_pre_approach) {
            contact_state.in_contact.fill(true);
        }

        return contact_state;
    }

    void logFootForceBreakdown(const Eigen::VectorXd& foot_forces,
                               const ContactDetector::ContactState& contact_state,
                               bool use_contact_mask) const {
        if (foot_forces.size() < 12) {
            return;
        }

        static constexpr const char* kLegNames[4] = {"lf", "lh", "rh", "rf"};
        std::ostringstream stream;
        stream << "MPC leg_forces:";
        for (int leg = 0; leg < 4; ++leg) {
            const Eigen::Vector3d f_leg = foot_forces.segment<3>(leg * 3);
            const bool support = !use_contact_mask || contact_state.in_contact[leg];
            stream << " " << kLegNames[leg]
                   << "[s=" << (support ? 1 : 0)
                   << " fx=" << f_leg.x()
                   << " fy=" << f_leg.y()
                   << " fz=" << f_leg.z()
                   << " n=" << f_leg.norm() << "]";
        }
        RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());
    }

    void zeroSwingLegForces(Eigen::VectorXd& foot_forces,
                            const ContactDetector::ContactState& contact_state) const {
        if (foot_forces.size() < 12) {
            return;
        }

        for (int i = 0; i < 4; ++i) {
            if (!contact_state.in_contact[i]) {
                foot_forces.segment<3>(i * 3).setZero();
            }
        }
    }

    void applyVerticalSupport(Eigen::VectorXd& foot_forces,
                              const ContactDetector::ContactState& contact_state,
                              bool use_contact_mask,
                              bool crossing_pre_approach) {
        if (!vertical_support_enabled_ ||
            foot_forces.size() < 12 ||
            current_srbd_state_.size() < 12) {
            return;
        }

        std::array<bool, 4> support_legs{};
        int support_count = 0;
        for (int i = 0; i < 4; ++i) {
            support_legs[i] = !use_contact_mask || contact_state.in_contact[i];
            if (support_legs[i]) {
                ++support_count;
            }
        }

        if (support_count < 2) {
            support_legs.fill(true);
            support_count = 4;
        }

        const double target_height =
            vertical_support_target_height_ > 0.0
                ? vertical_support_target_height_
                : nominal_body_height_;
        const double raw_height_error = target_height - current_srbd_state_(2);
        const double height_error = std::max(
            -vertical_support_height_error_limit_,
            std::min(raw_height_error, vertical_support_height_error_limit_));
        const double vertical_velocity = current_srbd_state_(8);
        const double gravity_force = mass_ * 9.81;
        // Catch mode: the trunk is meaningfully below target (3 cm; normal
        // trot bob keeps h_err inside ~+-0.06 so this arms on the deeper
        // half of a dip, before the 0.12 m smoke gate at h_err 0.08) or
        // free-falling (vz < -0.25 m/s pre-arms the catch from a bounce
        // apex while z still reads high). Above target and rising, a
        // transient shortfall is the DESIRED deceleration -- boosting
        // demand there pumps the pogo instead of damping it.
        const bool catch_mode =
            height_error > 0.03 || vertical_velocity < -0.25;
        // Symmetric height regulation. The old one-sided law
        // (mg + kp*max(0,err) + kd*max(0,-vz)) could only ever ADD force:
        // above target and rising it output exactly mg, cancelling gravity,
        // so the trunk coasted up until the feet unloaded, fell back,
        // rang the contacts, and got launched again (run28/29 hop cycle).
        // A regulator must also ease below mg to decelerate ascent; the
        // min-total floor below keeps the feet from unloading completely.
        const double support_force =
            gravity_force +
            vertical_support_kp_ * height_error -
            vertical_support_kd_ * vertical_velocity;
        const double min_total_force =
            gravity_force * vertical_support_min_total_force_multiplier_;
        const double target_total_force =
            std::max(min_total_force, support_force);
        const double stabilization_headroom =
            (attitude_support_enabled_ &&
             isSupportStabilizationActive(crossing_pre_approach))
                ? std::min(attitude_support_max_leg_delta_,
                           vertical_support_max_leg_force_)
                : 0.0;
        // In a walking catch the attitude reservation is released: with the
        // 28 N headroom a trot pair tops out at 2x67=134 N (~1.14 mg), which
        // cannot arrest a deep fall (run72 sank to z=0.05 through a
        // "full-force" recovery). Landing on 0.20 m of trunk beats holding
        // 28 N in reserve for a trim loop; attitude deltas then only
        // subtract from saturated legs, which is the right priority order.
        const bool walking_catch =
            catch_mode && current_mode_ == TrajectoryGenerator::Mode::WALKING;
        const double vertical_force_ceiling =
            std::max(0.0, vertical_support_max_leg_force_ -
                              (walking_catch ? 0.0 : stabilization_headroom));
        const double per_leg_force =
            std::min(vertical_force_ceiling,
                     target_total_force / static_cast<double>(support_count));

        // Static-equilibrium distribution: the COM does not sit at the
        // centroid of the support feet, so an equal split permanently
        // misloads one pair and pitches the trunk. Solve the minimum-
        // deviation force set that keeps the total and zeroes the planar
        // moments about the COM. Lever arms come from updateSupportGeometry
        // (measured FK, world axes), so the balance stays correct in a deep
        // crouch and under body tilt, not just at the nominal stance.
        std::array<double, 4> fz_dist{};
        if (support_count == 2) {
            // Trot pair: two feet cannot zero both planar moments unless the
            // COM sits exactly on the support line, so the exact 3-multiplier
            // KKT below turns near-singular (det ~1e-6) and its clamped
            // "solution" loads one foot to the ceiling and unloads the other
            // completely (run39 walking: fz=[77, 0] against a 118 N demand,
            // a 1.25 Hz corner-drop that careened the trunk and somersaulted
            // the turn stage). Split the total along the single feasible
            // degree of freedom instead: w minimizes the squared planar
            // moment |w*d1 + (1-w)*d2|^2; the irreducible residual is left
            // to the attitude trim.
            const double total =
                std::min(target_total_force,
                         vertical_force_ceiling * support_count);
            int legs[2] = {-1, -1};
            int k = 0;
            for (int i = 0; i < 4; ++i) {
                if (support_legs[i] && k < 2) {
                    legs[k++] = i;
                }
            }
            const double dx1 = support_lever_arms_(legs[0], 0);
            const double dy1 = support_lever_arms_(legs[0], 1);
            const double dx2 = support_lever_arms_(legs[1], 0);
            const double dy2 = support_lever_arms_(legs[1], 1);
            const double ex = dx1 - dx2;
            const double ey = dy1 - dy2;
            const double denom = ex * ex + ey * ey;
            double w = 0.5;
            if (denom > 1e-8) {
                w = -(dx2 * ex + dy2 * ey) / denom;
            }
            // Keep both feet meaningfully loaded: a fully unloaded stance
            // foot loses traction authority and re-rings the contact.
            w = clampDouble(w, 0.15, 0.85);
            // Total-conserving clamp: clamping each foot independently let
            // an uneven split silently shed weight (w=0.85 against a 118 N
            // demand: 67 + 17.7 = 85 N delivered, 28% short -- one sagging
            // stride of the run64-68 height limit cycle). Clamp one foot
            // inside the band where the OTHER foot can still absorb the
            // remainder; total <= 2*ceiling is guaranteed above.
            const double f0 = clampDouble(
                total * w,
                std::max(0.0, total - vertical_force_ceiling),
                std::min(vertical_force_ceiling, total));
            fz_dist[legs[0]] = f0;
            fz_dist[legs[1]] = total - f0;
        } else {
            const double total =
                std::min(target_total_force,
                         vertical_force_ceiling * support_count);
            const double base = total / static_cast<double>(support_count);
            double sxx = 1e-4, sxy = 0.0, syy = 1e-4, sx = 0.0, sy = 0.0;
            for (int i = 0; i < 4; ++i) {
                if (!support_legs[i]) {
                    continue;
                }
                const double dx = support_lever_arms_(i, 0);
                const double dy = support_lever_arms_(i, 1);
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
                sx += dx;
                sy += dy;
            }
            // Minimize sum (fz_i - base)^2 subject to BOTH zero planar
            // moments AND the total force. fz_i = base + l0 + l1*dx + l2*dy
            // needs all three multipliers: with only l1/l2 the total picks
            // up an l1*sx + l2*sy error whenever the foot centroid is not
            // at the COM. At the deep crouch sx = -0.29 m and the solve
            // silently delivered 75-80% of body weight (run31: fz 88 N vs
            // mg 118 N, the trunk sank to belly rest and stayed there).
            const double n = static_cast<double>(support_count);
            Eigen::Matrix3d A;
            A << n,  sx,  sy,
                 sx, sxx, sxy,
                 sy, sxy, syy;
            Eigen::Vector3d b(0.0, -base * sx, -base * sy);
            Eigen::Vector3d lambda = Eigen::Vector3d::Zero();
            if (std::abs(A.determinant()) > 1e-9) {
                lambda = A.ldlt().solve(b);
            }
            double dist_sum = 0.0;
            for (int i = 0; i < 4; ++i) {
                if (!support_legs[i]) {
                    continue;
                }
                const double dx = support_lever_arms_(i, 0);
                const double dy = support_lever_arms_(i, 1);
                fz_dist[i] = clampDouble(
                    base + lambda(0) + lambda(1) * dx + lambda(2) * dy,
                    0.0, vertical_force_ceiling);
                dist_sum += fz_dist[i];
            }
            // Conservation repair after clamping. WALKING only: the HOVER
            // crouch/stand chain was validated with the one-sided
            // proportional rescale below and must keep the moment balance
            // the 3-multiplier solve chose (a waterfill would dump force
            // back onto legs the balance deliberately unloaded).
            // - Overdelivery (dist_sum > total): negative-lambda legs were
            //   clamped to zero, the rest now sum high (run70: 97.5 N against
            //   an 80.6 N target while the trunk was already rising -- +17 N
            //   pumped straight into the height bounce). Scale down.
            // - Shortfall (dist_sum < total): a proportional rescale cannot
            //   raise a zero-clamped leg and saturates the rest at the
            //   ceiling (run72: legs=[1011] target 149.5 N delivered 134 --
            //   the trunk kept sinking through a "full-force" recovery).
            //   Waterfill the deficit into ceiling headroom instead, which
            //   by construction cannot overshoot any leg's ceiling.
            if (current_mode_ == TrajectoryGenerator::Mode::WALKING) {
                if (dist_sum > total + 1e-6) {
                    const double scale = total / dist_sum;
                    for (int i = 0; i < 4; ++i) {
                        if (support_legs[i]) {
                            fz_dist[i] *= scale;
                        }
                    }
                } else if (dist_sum < total - 1e-6) {
                    double headroom_sum = 0.0;
                    std::array<double, 4> headroom{};
                    for (int i = 0; i < 4; ++i) {
                        if (!support_legs[i]) {
                            continue;
                        }
                        headroom[i] =
                            std::max(0.0, vertical_force_ceiling - fz_dist[i]);
                        headroom_sum += headroom[i];
                    }
                    if (headroom_sum > 1e-6) {
                        const double deficit =
                            std::min(total - dist_sum, headroom_sum);
                        for (int i = 0; i < 4; ++i) {
                            if (headroom[i] > 0.0) {
                                fz_dist[i] +=
                                    deficit * headroom[i] / headroom_sum;
                            }
                        }
                    }
                }
            } else if (dist_sum > 1e-6 && dist_sum < total) {
                const double scale =
                    std::min(total / dist_sum,
                             vertical_force_ceiling * n /
                                 std::max(dist_sum, 1e-6));
                for (int i = 0; i < 4; ++i) {
                    if (support_legs[i]) {
                        fz_dist[i] = std::min(fz_dist[i] * scale,
                                              vertical_force_ceiling);
                    }
                }
            }
        }

        for (int i = 0; i < 4; ++i) {
            if (!support_legs[i]) {
                continue;
            }

            const int z_index = i * 3 + 2;
            if (current_mode_ == TrajectoryGenerator::Mode::WALKING ||
                current_mode_ == TrajectoryGenerator::Mode::HOVER) {
                // The height PD owns the z channel in flat modes. Merging the
                // QP output through max() let the saturated QP (u_max) push
                // 1.6x body weight into the ground permanently, which pogo-
                // bounced the trunk instead of regulating height.
                foot_forces(z_index) = fz_dist[i];
            } else {
                foot_forces(z_index) = std::max(foot_forces(z_index), per_leg_force);
            }
        }

        // Asymmetric per-leg load-rate limit (final shaping step). At every
        // trot pair transition the static split re-targets instantly
        // (e.g. a fresh diagonal jumps 0 -> ~47 N while the outgoing pair
        // drops 26 N), which hammers the contacts and re-excites the tip
        // every 0.3 s. Limit only the UPWARD rate: a newly landed foot is
        // loaded over ~100-150 ms (spanning the duty overlap), while
        // unloading stays instant because the leg physically leaves the
        // ground anyway.
        if (vertical_support_load_rate_ > 0.0 &&
            (current_mode_ == TrajectoryGenerator::Mode::WALKING ||
             current_mode_ == TrajectoryGenerator::Mode::HOVER)) {
            const double max_step = vertical_support_load_rate_ * dt_;
            double shortfall = 0.0;
            std::array<bool, 4> ramp_limited{};
            for (int i = 0; i < 4; ++i) {
                if (!support_legs[i]) {
                    vertical_support_last_fz_(i) = 0.0;
                    continue;
                }
                const double allowed =
                    vertical_support_last_fz_(i) + max_step;
                if (fz_dist[i] > allowed) {
                    shortfall += fz_dist[i] - allowed;
                    fz_dist[i] = allowed;
                    ramp_limited[i] = true;
                }
            }
            // Total-conserving backfill (WALKING only; the HOVER stand chain
            // is validated as-is and its ramp almost never binds). The old
            // ramp swallowed the held-back force outright ("~1-2 mm of
            // sink"): in reality every pair transition delivered 18-46%
            // less than body weight for ~0.1 s, the trunk free-fell, and
            // the height loop slammed it back up -- the run64-68
            // 0.08-0.23 m per-stride bob that made smoke a 1/3 phase
            // gamble. Hand the shortfall to support legs with ceiling
            // headroom instead: pass 0 prefers legs not ramp-limited this
            // tick (the outgoing, still-planted pair during the duty
            // overlap -- compliant load transfer survives); pass 1
            // overrides the ramp when nobody else is available (a fresh
            // pair alone must carry the trunk, nothing else physically
            // can -- conservation wins over gentleness).
            const bool backfill_active =
                current_mode_ == TrajectoryGenerator::Mode::WALKING;
            for (int pass = 0;
                 pass < 2 && backfill_active && shortfall > 1e-6; ++pass) {
                double headroom_sum = 0.0;
                std::array<double, 4> headroom{};
                for (int i = 0; i < 4; ++i) {
                    if (!support_legs[i] ||
                        (pass == 0 && ramp_limited[i])) {
                        continue;
                    }
                    headroom[i] =
                        std::max(0.0, vertical_force_ceiling - fz_dist[i]);
                    headroom_sum += headroom[i];
                }
                if (headroom_sum <= 1e-6) {
                    continue;
                }
                const double used = std::min(shortfall, headroom_sum);
                for (int i = 0; i < 4; ++i) {
                    if (headroom[i] > 0.0) {
                        fz_dist[i] += used * headroom[i] / headroom_sum;
                    }
                }
                shortfall -= used;
            }
            for (int i = 0; i < 4; ++i) {
                if (!support_legs[i]) {
                    continue;
                }
                vertical_support_last_fz_(i) = fz_dist[i];
                foot_forces(i * 3 + 2) = fz_dist[i];
            }
        }

        if (current_mode_ == TrajectoryGenerator::Mode::WALKING) {
            // applyVerticalSupport is const; use a local steady clock for
            // throttling instead of the node clock.
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), steady_clock, 200,
                "[vsplit] n=%d legs=[%d%d%d%d] fz=[%.1f %.1f %.1f %.1f] "
                "total=%.1f h_err=%.3f catch=%d lever_x=[%.2f %.2f %.2f %.2f] "
                "lever_y=[%.2f %.2f %.2f %.2f] src=%s",
                support_count,
                support_legs[0], support_legs[1], support_legs[2], support_legs[3],
                fz_dist[0], fz_dist[1], fz_dist[2], fz_dist[3],
                target_total_force, height_error, catch_mode ? 1 : 0,
                support_lever_arms_(0, 0), support_lever_arms_(1, 0),
                support_lever_arms_(2, 0), support_lever_arms_(3, 0),
                support_lever_arms_(0, 1), support_lever_arms_(1, 1),
                support_lever_arms_(2, 1), support_lever_arms_(3, 1),
                support_geometry_from_fk_ ? "fk" : "stance");
        }
    }

    void applyHoverForceSanitizer(Eigen::VectorXd& foot_forces) const {
        if (!hover_force_sanitize_enabled_ ||
            current_mode_ != TrajectoryGenerator::Mode::HOVER ||
            foot_forces.size() < 12) {
            return;
        }

        for (int leg = 0; leg < 4; ++leg) {
            const int base = leg * 3;
            foot_forces(base + 0) = 0.0;
            foot_forces(base + 1) = 0.0;
            foot_forces(base + 2) = clampDouble(
                foot_forces(base + 2),
                hover_force_min_leg_fz_,
                hover_force_max_leg_fz_);
        }
    }

    void applyAttitudeSupport(Eigen::VectorXd& foot_forces,
                              const ContactDetector::ContactState& contact_state,
                              bool use_contact_mask,
                              bool crossing_pre_approach) {
        last_attitude_support_delta_.setZero();
        last_roll_error_ = 0.0;
        last_pitch_error_ = 0.0;

        if (!attitude_support_enabled_ ||
            foot_forces.size() < 12 ||
            current_srbd_state_.size() < 12 ||
            !isSupportStabilizationActive(crossing_pre_approach)) {
            return;
        }

        std::array<bool, 4> support_legs{};
        for (int i = 0; i < 4; ++i) {
            support_legs[i] = !use_contact_mask || contact_state.in_contact[i];
        }

        if (!current_body_q_valid_) {
            return;
        }

        const LevelAttitudeError level_err =
            computeLevelAttitudeErrorFromQuat(current_body_q_wb_);
        if (!level_err.valid) {
            return;
        }

        last_level_roll_like_ = level_err.roll_like;
        last_level_pitch_like_ = level_err.pitch_like;
        last_level_tilt_ = level_err.tilt;
        last_body_up_z_ = level_err.body_up_z;
        if (level_err.inverted) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Attitude support skipped: body appears inverted/up_z=%.3f tilt=%.3f",
                level_err.body_up_z,
                level_err.tilt);
            return;
        }

        const double roll_rate = current_srbd_state_(9);
        const double pitch_rate = current_srbd_state_(10);

        last_roll_error_ = attitude_support_roll_target_ - level_err.roll_like;
        last_pitch_error_ = attitude_support_pitch_target_ - level_err.pitch_like;

        const double gain_scale = attitudeSupportGainScale();
        double tau_roll = gain_scale *
            (attitude_support_roll_kp_ * last_roll_error_ -
             attitude_support_roll_kd_ * roll_rate);
        double tau_pitch = gain_scale *
            (attitude_support_pitch_kp_ * last_pitch_error_ -
             attitude_support_pitch_kd_ * pitch_rate);
        tau_roll = clampDouble(tau_roll, -40.0, 40.0);
        tau_pitch = clampDouble(tau_pitch, -45.0, 45.0);

        // COM-relative, world-axis lever arms from the measured FK
        // (stance-geometry fallback until joints arrive).
        const Eigen::MatrixXd& foot_rel = support_lever_arms_;

        double denom_y = 1e-6;
        double denom_x = 1e-6;
        int support_count = 0;
        for (int i = 0; i < 4; ++i) {
            if (!support_legs[i]) {
                continue;
            }
            denom_y += foot_rel(i, 1) * foot_rel(i, 1);
            denom_x += foot_rel(i, 0) * foot_rel(i, 0);
            ++support_count;
        }
        if (support_count < 2) {
            return;
        }

        std::array<double, 4> dfz{};
        double mean_dfz = 0.0;
        for (int leg = 0; leg < 4; ++leg) {
            if (!support_legs[leg]) {
                continue;
            }
            const double x = foot_rel(leg, 0);
            const double y = foot_rel(leg, 1);
            double d = tau_roll * y / denom_y - tau_pitch * x / denom_x;
            d = clampDouble(
                d,
                -attitude_support_max_leg_delta_,
                attitude_support_max_leg_delta_);
            dfz[leg] = d;
            mean_dfz += d;
        }
        mean_dfz /= static_cast<double>(support_count);

        for (int leg = 0; leg < 4; ++leg) {
            if (!support_legs[leg]) {
                continue;
            }
            const double delta = clampDouble(
                dfz[leg] - mean_dfz,
                -attitude_support_max_leg_delta_,
                attitude_support_max_leg_delta_);
            const int z_index = leg * 3 + 2;
            const double updated_force = std::max(
                0.0,
                std::min(vertical_support_max_leg_force_,
                         foot_forces(z_index) + delta));
            last_attitude_support_delta_(leg) += updated_force - foot_forces(z_index);
            foot_forces(z_index) = updated_force;
        }

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "[attitude_support] roll_like=%.3f pitch_like=%.3f tilt=%.3f up_z=%.3f tau=[%.2f,%.2f] dfz=[%.1f,%.1f,%.1f,%.1f]",
            level_err.roll_like,
            level_err.pitch_like,
            level_err.tilt,
            level_err.body_up_z,
            tau_roll,
            tau_pitch,
            last_attitude_support_delta_(0),
            last_attitude_support_delta_(1),
            last_attitude_support_delta_(2),
            last_attitude_support_delta_(3));
    }

    void applyCrossingForwardAssist(Eigen::VectorXd& foot_forces,
                                    const ContactDetector::ContactState& contact_state,
                                    bool use_contact_mask,
                                    bool crossing_pre_approach) {
        if (!crossing_forward_assist_enabled_ || foot_forces.size() < 12) {
            return;
        }

        if (crossing_pre_approach && !crossing_forward_assist_pre_approach_enabled_) {
            return;
        }

        bool active = false;
        if (current_mode_ == TrajectoryGenerator::Mode::CROSSING) {
            active = crossing_pre_approach;
            if (!active && mpc_controller_ && mpc_controller_->isCrossingEnabled()) {
                const auto stage = mpc_controller_->getCurrentCrossingState();
                active = stage == CrossingStateMachine::CrossingState::APPROACH ||
                         stage == CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT;
            }
        }

        if (!active) {
            return;
        }

        const LevelAttitudeError level_err =
            current_body_q_valid_
                ? computeLevelAttitudeErrorFromQuat(current_body_q_wb_)
                : LevelAttitudeError{};
        if (!level_err.valid ||
            std::abs(level_err.roll_like) > 0.30 ||
            std::abs(level_err.pitch_like) > 0.25 ||
            level_err.tilt > 0.40 ||
            level_err.body_up_z < 0.85 ||
            level_err.inverted) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Crossing forward assist gated by level attitude: valid=%d roll=%.3f pitch=%.3f tilt=%.3f up_z=%.3f",
                                 level_err.valid ? 1 : 0,
                                 level_err.roll_like,
                                 level_err.pitch_like,
                                 level_err.tilt,
                                 level_err.body_up_z);
            return;
        }

        std::array<bool, 4> support_legs{};
        int support_count = 0;
        for (int i = 0; i < 4; ++i) {
            support_legs[i] = !use_contact_mask || contact_state.in_contact[i];
            if (support_legs[i]) {
                ++support_count;
            }
        }
        if (support_count < 2) {
            support_legs.fill(true);
        }

        for (int i = 0; i < 4; ++i) {
            if (!support_legs[i]) {
                continue;
            }
            const int x_index = i * 3;
            foot_forces(x_index) = std::max(foot_forces(x_index),
                                            crossing_forward_assist_force_per_leg_);
        }
    }

    void applyFlatForwardAssist(Eigen::VectorXd& foot_forces,
                                const ContactDetector::ContactState& contact_state,
                                bool use_contact_mask,
                                bool crossing_pre_approach) {
        if (!flat_forward_assist_enabled_ ||
            foot_forces.size() < 12 ||
            current_mode_ != TrajectoryGenerator::Mode::WALKING ||
            crossing_pre_approach ||
            std::abs(velocity_cmd_(0)) < flat_forward_assist_min_cmd_) {
            return;
        }

        const LevelAttitudeError level_err =
            current_body_q_valid_
                ? computeLevelAttitudeErrorFromQuat(current_body_q_wb_)
                : LevelAttitudeError{};
        if (!level_err.valid ||
            std::abs(level_err.roll_like) > 0.35 ||
            std::abs(level_err.pitch_like) > 0.35 ||
            level_err.tilt > 0.48 ||
            level_err.body_up_z < 0.82 ||
            level_err.inverted) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Flat forward assist gated by attitude: valid=%d roll=%.3f pitch=%.3f tilt=%.3f up_z=%.3f",
                level_err.valid ? 1 : 0,
                level_err.roll_like,
                level_err.pitch_like,
                level_err.tilt,
                level_err.body_up_z);
            return;
        }

        std::array<bool, 4> support_legs{};
        int support_count = 0;
        for (int i = 0; i < 4; ++i) {
            support_legs[i] = !use_contact_mask || contact_state.in_contact[i];
            if (support_legs[i]) {
                ++support_count;
            }
        }
        if (support_count < 2) {
            support_legs.fill(true);
        }

        const double assist_force =
            clampDouble(velocity_cmd_(0) * flat_forward_assist_force_per_mps_,
                        -flat_forward_assist_max_force_per_leg_,
                        flat_forward_assist_max_force_per_leg_);
        for (int i = 0; i < 4; ++i) {
            if (!support_legs[i]) {
                continue;
            }
            const int x_index = i * 3;
            if (assist_force >= 0.0) {
                foot_forces(x_index) = std::max(foot_forces(x_index), assist_force);
            } else {
                foot_forces(x_index) = std::min(foot_forces(x_index), assist_force);
            }
        }
    }

    // Close the planar loops (vx, vy, yaw rate) through stance-foot
    // tangential forces. Longitudinal/lateral velocity errors map to equal
    // fx/fy shares; the yaw-rate error maps to a differential fx between the
    // left and right stance feet. Runs in WALKING only; HOVER keeps the
    // conservative sanitizer and CROSSING keeps its dedicated assists.
    void applyWalkingPlanarStabilization(Eigen::VectorXd& foot_forces,
                                         const ContactDetector::ContactState& contact_state,
                                         bool use_contact_mask,
                                         bool crossing_pre_approach) {
        if (!walking_stabilization_enabled_ ||
            foot_forces.size() < 12 ||
            current_srbd_state_.size() < 12 ||
            current_mode_ != TrajectoryGenerator::Mode::WALKING ||
            crossing_pre_approach) {
            return;
        }

        const LevelAttitudeError level_err =
            current_body_q_valid_
                ? computeLevelAttitudeErrorFromQuat(current_body_q_wb_)
                : LevelAttitudeError{};
        if (!level_err.valid || level_err.inverted ||
            level_err.tilt > 0.48 || level_err.body_up_z < 0.82) {
            return;
        }

        // Tangential forces act ~0.2 m below the COM, so every newton of
        // velocity-tracking fx/fy is also ~0.2 N*m of pitch/roll moment --
        // far more than the fz attitude trim can counter (run49: a -70 N
        // braking fx = -14 N*m pitch vs ~6 N*m of trim authority, a
        // positive-feedback tip). Fade the planar loop out as tilt grows.
        // Thresholds sit ABOVE the routine trot oscillation (~0.1-0.3 rad):
        // fading from 0.10 rad choked the only forward actuator and the
        // robot stepped in place (run51 projected 0.000 m). 0.25-0.45 only
        // strips the velocity loop during genuine tip emergencies.
        const double tilt_fade = clampDouble(
            1.0 - (level_err.tilt - 0.25) / (0.45 - 0.25), 0.0, 1.0);

        std::array<bool, 4> support_legs{};
        int support_count = 0;
        for (int i = 0; i < 4; ++i) {
            support_legs[i] = !use_contact_mask || contact_state.in_contact[i];
            if (support_legs[i]) {
                ++support_count;
            }
        }
        if (support_count < 2) {
            support_legs.fill(true);
            support_count = 4;
        }

        // Velocity commands and measurements are in the odom/world frame for
        // vx/vy at small yaw; rotate the world-frame velocity error into the
        // body frame using the current yaw so the force channels stay aligned
        // with the feet when the body has turned.
        const double yaw = current_srbd_state_(5);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const double vx_world = current_srbd_state_(6);
        const double vy_world = current_srbd_state_(7);
        const double vx_raw = cy * vx_world + sy * vy_world;
        const double vy_raw = -sy * vx_world + cy * vy_world;

        // Track the MEAN velocity, not the trot rock. The odom velocity is
        // measured at base_link, so the residual pitch/roll limit cycle
        // shows up as vx = h*wy ~ +-0.12 m/s at gait frequency with ZERO
        // net COM motion (run57: vx oscillating -0.12..+0.12, fx flapping
        // 25 N each way, net displacement 0). A ~0.35 s low-pass knocks the
        // ~1.6 Hz rock down 5x while barely delaying command tracking.
        {
            const double alpha =
                dt_ / (std::max(1e-3, walking_vel_filter_tau_) + dt_);
            vx_body_filt_ += alpha * (vx_raw - vx_body_filt_);
            vy_body_filt_ += alpha * (vy_raw - vy_body_filt_);
            wz_filt_ += alpha * (current_srbd_state_(11) - wz_filt_);
        }
        const double vx_body = vx_body_filt_;
        const double vy_body = vy_body_filt_;

        const double fx_total = tilt_fade * clampDouble(
            walking_vx_kp_ * (velocity_cmd_(0) - vx_body),
            -walking_fx_max_per_leg_ * support_count,
            walking_fx_max_per_leg_ * support_count);
        const double fy_total = tilt_fade * clampDouble(
            walking_vy_kp_ * (velocity_cmd_(1) - vy_body),
            -walking_fy_max_per_leg_ * support_count,
            walking_fy_max_per_leg_ * support_count);

        const double wz = wz_filt_;
        const double yaw_moment = tilt_fade * clampDouble(
            walking_wz_kp_ * (velocity_cmd_(2) - wz),
            -walking_yaw_moment_max_,
            walking_yaw_moment_max_);

        double denom_y = 1e-6;
        double lever_h = 0.0;
        for (int i = 0; i < 4; ++i) {
            if (support_legs[i]) {
                denom_y += support_lever_arms_(i, 1) * support_lever_arms_(i, 1);
                lever_h += -support_lever_arms_(i, 2);
            }
        }
        lever_h = std::max(0.05, lever_h / static_cast<double>(support_count));

        // Roll/pitch RATE damping through tangential forces. The vertical
        // channel is owned by the height loop + static split, and the fz
        // attitude trim has zero authority about the line joining a trot
        // pair -- but the stance feet sit ~0.2 m below the COM, so common-
        // mode fx/fy do create horizontal-axis moments (M = r x f with
        // r_z = -h: Mx = h*sum(fy), My = -h*sum(fx)). Damp the world-frame
        // angular rates with them; pure damping only removes rotational
        // energy, so a sign-safe way to arrest the diagonal tip that the
        // duty-overlap windows then reset.
        const double wx = current_srbd_state_(9);
        const double wy = current_srbd_state_(10);
        const double fx_rate_total = clampDouble(
            walking_rate_damping_ * wy / lever_h,
            -walking_rate_force_max_, walking_rate_force_max_);
        const double fy_rate_total = clampDouble(
            -walking_rate_damping_ * wx / lever_h,
            -walking_rate_force_max_, walking_rate_force_max_);

        // fx/fy_total are BODY-frame corrections (from body-frame velocity
        // errors) but foot_forces is a WORLD-frame vector (the WBC rotates
        // it by R_wb^T). Writing them in unrotated used to be harmless only
        // at yaw~0: once yaw drifted, the "forward" correction pushed in a
        // wrong world direction, exciting more lateral error and more yaw --
        // the careening spiral of run32/33 (forward ended at yaw=-2.98 with
        // projected 0.02-0.56 m out of 1.0-1.6 m planar). Rotate the shares
        // to world axes; the yaw-moment differential already uses world
        // levers and world fx, so it stays as is.
        const double fx_share_b = fx_total / support_count;
        const double fy_share_b = fy_total / support_count;
        const double fx_share_w =
            cy * fx_share_b - sy * fy_share_b + fx_rate_total / support_count;
        const double fy_share_w =
            sy * fx_share_b + cy * fy_share_b + fy_rate_total / support_count;
        for (int i = 0; i < 4; ++i) {
            if (!support_legs[i]) {
                continue;
            }
            const int x_index = i * 3;
            const int y_index = i * 3 + 1;
            // Mz = sum(-y_i * fx_i): +Mz needs +fx on the left (y<0) feet.
            const double dfx_yaw =
                -yaw_moment * support_lever_arms_(i, 1) / denom_y;
            // The planar loop OWNS the stance tangentials (mirror of the fz
            // ownership above): the QP's fx/fy were solved jointly with a
            // fz plan we discard, so during transients they are large,
            // inconsistent shoves (run44 with the loop disabled: lone QP
            // fx=+31 N = +6 N*m of pitch, twice the static tip moment).
            foot_forces(x_index) = clampDouble(
                fx_share_w + dfx_yaw,
                -walking_fx_max_per_leg_, walking_fx_max_per_leg_);
            foot_forces(y_index) = clampDouble(
                fy_share_w,
                -walking_fy_max_per_leg_, walking_fy_max_per_leg_);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "[walk_stab] vx=%.3f/%.3f vy=%.3f/%.3f wz=%.3f/%.3f yaw=%.2f xy=[%.3f %.3f] fx=%.1f fy=%.1f mz=%.2f rate_fx=%.1f rate_fy=%.1f n=%d",
            vx_body, velocity_cmd_(0),
            vy_body, velocity_cmd_(1),
            wz, velocity_cmd_(2),
            yaw,
            current_srbd_state_(0), current_srbd_state_(1),
            fx_total, fy_total, yaw_moment,
            fx_rate_total, fy_rate_total, support_count);
    }

    void publishFootForces(const Eigen::VectorXd& foot_forces) {
        auto force_msg = std_msgs::msg::Float64MultiArray();
        force_msg.data.resize(12, 0.0);
        const int count = std::min<int>(12, foot_forces.size());
        for (int i = 0; i < count; ++i) {
            force_msg.data[i] = foot_forces(i);
        }
        foot_force_pub_->publish(force_msg);
    }

    void advanceGaitPhase() {
        gait_phase_ += dt_ / gait_period_;
        if (gait_phase_ >= 1.0) {
            gait_phase_ -= 1.0;
        }
    }

    CrossingStateMachine::RobotState buildCrossingRobotState() const {
        CrossingStateMachine::RobotState robot_state;
        robot_state.position = current_srbd_state_.segment<3>(0);
        robot_state.velocity = current_srbd_state_.segment<3>(6);
        robot_state.orientation = current_srbd_state_.segment<3>(3);
        robot_state.angular_velocity = current_srbd_state_.segment<3>(9);
        robot_state.sliding_positions = current_sliding_positions_;
        robot_state.sliding_velocities = current_sliding_velocities_;

        for (int i = 0; i < 4; ++i) {
            if (joint_received_) {
                // Infer ELBOW vs KNEE config from true joint state (knee angle)
                // Negative knee angle corresponds to ELBOW configuration
                bool is_elbow = current_joint_angles_[i](2) < 0.0;
                robot_state.leg_configs[i] = is_elbow ? 
                    CrossingStateMachine::LegConfiguration::ELBOW : 
                    CrossingStateMachine::LegConfiguration::KNEE;
                    
                robot_state.foot_contacts[i] = true;
                
                // Keep expectation-based foot positions for MPC SRBD stability
                robot_state.foot_positions[i] = robot_state.position;
                if (base_foot_positions_.rows() == 4 && base_foot_positions_.cols() == 3) {
                    robot_state.foot_positions[i] += base_foot_positions_.row(i).transpose();
                    robot_state.foot_positions[i].x() += robot_state.sliding_positions(i);
                }
            } else {
                robot_state.leg_configs[i] = CrossingStateMachine::LegConfiguration::ELBOW;
                robot_state.foot_contacts[i] = true;
                robot_state.foot_positions[i] = robot_state.position;
                if (base_foot_positions_.rows() == 4 && base_foot_positions_.cols() == 3) {
                    robot_state.foot_positions[i] += base_foot_positions_.row(i).transpose();
                    robot_state.foot_positions[i].x() += robot_state.sliding_positions(i);
                }
            }
        }

        return robot_state;
    }

    std::string crossingStateKey(CrossingStateMachine::CrossingState state) const {
        switch (state) {
            case CrossingStateMachine::CrossingState::APPROACH:
                return "APPROACH";
            case CrossingStateMachine::CrossingState::BODY_FORWARD_SHIFT:
                return "BODY_FORWARD_SHIFT";
            case CrossingStateMachine::CrossingState::FRONT_LEGS_TRANSIT:
                return "FRONT_LEGS_TRANSIT";
            case CrossingStateMachine::CrossingState::HYBRID_GAIT_WALKING:
                return "HYBRID_GAIT_WALKING";
            case CrossingStateMachine::CrossingState::RAIL_ALIGNMENT:
                return "RAIL_ALIGNMENT";
            case CrossingStateMachine::CrossingState::REAR_LEGS_TRANSIT:
                return "REAR_LEGS_TRANSIT";
            case CrossingStateMachine::CrossingState::ALL_KNEE_STATE:
                return "ALL_KNEE_STATE";
            case CrossingStateMachine::CrossingState::RECOVERY:
                return "RECOVERY";
            case CrossingStateMachine::CrossingState::CONTINUE_FORWARD:
                return "CONTINUE_FORWARD";
            case CrossingStateMachine::CrossingState::COMPLETED:
                return "COMPLETED";
            default:
                return "UNKNOWN";
        }
    }

    void publishCrossingState() {
        if (!crossing_state_pub_) {
            return;
        }

        std_msgs::msg::String msg;
        if (current_mode_ == TrajectoryGenerator::Mode::CROSSING &&
            mpc_controller_->isCrossingEnabled()) {
            msg.data = "CROSSING:" + crossingStateKey(mpc_controller_->getCurrentCrossingState());
        } else if (current_mode_ == TrajectoryGenerator::Mode::CROSSING) {
            msg.data = "CROSSING:PRE_APPROACH";
        } else {
            msg.data = getModeString();
        }
        crossing_state_pub_->publish(msg);
    }
    
    std::string getModeString() const {
        switch (current_mode_) {
            case TrajectoryGenerator::Mode::HOVER: return "HOVER";
            case TrajectoryGenerator::Mode::WALKING: return "WALKING";
            case TrajectoryGenerator::Mode::CROSSING: return "CROSSING";
            default: return "UNKNOWN";
        }
    }
    
    int getLegIdFromName(const std::string& name) const {
        if (name.find("lf_") != std::string::npos ||
            name.find("j1") != std::string::npos || name.find("leg1") != std::string::npos) {
            return 0;
        } else if (name.find("lh_") != std::string::npos ||
                   name.find("j2") != std::string::npos || name.find("leg2") != std::string::npos) {
            return 1;
        } else if (name.find("rh_") != std::string::npos ||
                   name.find("j3") != std::string::npos || name.find("leg3") != std::string::npos) {
            return 2;
        } else if (name.find("rf_") != std::string::npos ||
                   name.find("j4") != std::string::npos || name.find("leg4") != std::string::npos) {
            return 3;
        }
        return -1;
    }

    double mass_;
    int horizon_;
    double dt_;
    double control_freq_;
    double stance_length_;
    double stance_width_;
    double nominal_body_height_;
    Eigen::Vector3d com_offset_;
    CrossingStateMachine::WindowObstacle crossing_window_;
    double crossing_activation_distance_;
    double crossing_approach_speed_;
    bool vertical_support_enabled_;
    double vertical_support_target_height_;
    double vertical_support_kp_;
    double vertical_support_kd_;
    double vertical_support_min_total_force_multiplier_;
    double vertical_support_max_leg_force_;
    double vertical_support_height_error_limit_;
    double vertical_support_load_rate_ = 500.0;
    Eigen::Vector4d vertical_support_last_fz_ = Eigen::Vector4d::Zero();
    double support_foot_height_gate_ = 0.12;
    std::array<bool, 4> support_foot_airborne_{};
    bool hover_force_sanitize_enabled_ = true;
    double hover_force_max_leg_fz_ = 55.0;
    double hover_force_min_leg_fz_ = 18.0;
    bool attitude_support_enabled_;
    bool attitude_support_hover_enabled_ = true;
    double attitude_support_hover_scale_ = 0.5;
    double attitude_support_roll_target_;
    double attitude_support_pitch_target_;
    double attitude_support_roll_kp_;
    double attitude_support_roll_kd_;
    double attitude_support_pitch_kp_;
    double attitude_support_pitch_kd_;
    double attitude_support_max_leg_delta_;
    bool crossing_forward_assist_enabled_;
    double crossing_forward_assist_force_per_leg_;
    bool crossing_forward_assist_pre_approach_enabled_ = false;
    bool crossing_force_full_support_ = false;
    bool flat_force_full_support_ = false;
    bool flat_forward_assist_enabled_ = false;
    double flat_forward_assist_force_per_mps_ = 120.0;
    double flat_forward_assist_max_force_per_leg_ = 18.0;
    double flat_forward_assist_min_cmd_ = 0.02;
    bool walking_stabilization_enabled_ = true;
    double walking_vx_kp_ = 25.0;
    double walking_vy_kp_ = 25.0;
    double walking_wz_kp_ = 4.0;
    double walking_fx_max_per_leg_ = 30.0;
    double walking_fy_max_per_leg_ = 25.0;
    double walking_yaw_moment_max_ = 8.0;
    double walking_rate_damping_ = 1.5;
    double walking_rate_force_max_ = 20.0;
    double walking_vel_filter_tau_ = 0.35;
    double vx_body_filt_ = 0.0;
    double vy_body_filt_ = 0.0;
    double wz_filt_ = 0.0;
    bool crossing_freeze_rail_targets_ = false;
    bool bfs_rail_ramp_enabled_ = true;
    double bfs_rail_ramp_rate_ = 0.015;
    double bfs_rail_ramp_slow_scale_ = 0.25;
    double bfs_attitude_gate_roll_ = 0.30;
    double bfs_attitude_gate_pitch_ = 0.25;
    double bfs_attitude_gate_tilt_ = 0.40;
    double bfs_attitude_gate_up_z_ = 0.85;
    double bfs_attitude_gate_angular_rate_ = 1.20;
    double bfs_min_body_z_for_ramp_ = 0.080;
    double bfs_hard_fail_body_z_ = 0.055;
    
    // 控制器
    std::unique_ptr<MPCController> mpc_controller_;
    std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
    std::unique_ptr<ContactDetector> contact_detector_;
    std::unique_ptr<HybridGaitGenerator> gait_generator_;
    
    // ROS接口
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_crossing_sub_;
    rclcpp::Subscription<dog2_interfaces::msg::ContactPhase>::SharedPtr contact_phase_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr foot_force_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr crossing_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 状态
    std::array<Eigen::Vector3d, 4> current_joint_angles_{};
    Eigen::VectorXd current_srbd_state_;
    Eigen::Vector4d current_sliding_positions_;
    Eigen::Vector4d current_sliding_velocities_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d last_attitude_support_delta_ = Eigen::Vector4d::Zero();
    Eigen::Vector3d velocity_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_cmd_target_ = Eigen::Vector3d::Zero();
    double cmd_linear_slew_rate_ = 0.25;   // m/s^2
    double cmd_angular_slew_rate_ = 1.0;   // rad/s^2
    Eigen::MatrixXd base_foot_positions_ = Eigen::MatrixXd::Zero(4, 3);
    Eigen::Vector3d com_stance_ = Eigen::Vector3d::Zero();
    // 实时 FK 支撑几何（见 updateSupportGeometry）
    std::unique_ptr<dog2_dynamics::Dog2Model> dog2_model_;
    std::array<std::array<int, 4>, 4> fk_q_index_{};
    bool runtime_fk_ready_ = false;
    Eigen::MatrixXd support_lever_arms_ = Eigen::MatrixXd::Zero(4, 3);
    bool support_geometry_from_fk_ = false;
    Eigen::Quaterniond current_body_q_wb_{1.0, 0.0, 0.0, 0.0};
    bool odom_received_ = false;
    bool joint_received_ = false;
    std::array<bool, 4> leg_joint_received_{false, false, false, false};
    bool measured_leg_configs_valid_ = false;
    bool current_body_q_valid_ = false;
    int control_count_ = 0;
    double last_roll_error_ = 0.0;
    double last_pitch_error_ = 0.0;
    double last_level_roll_like_ = 0.0;
    double last_level_pitch_like_ = 0.0;
    double last_level_tilt_ = 0.0;
    double last_body_up_z_ = 1.0;
    bool last_bfs_rail_freeze_ = false;
    double last_bfs_rail_ramp_scale_ = 1.0;
    
    // 模式
    TrajectoryGenerator::Mode current_mode_;
    bool crossing_enabled_ = false;
    bool pending_crossing_request_ = false;
    
    // 步态
    double gait_phase_ = 0.0;
    double gait_period_ = 0.8;  // Trot周期0.8秒
    std::array<bool, 4> gait_contact_mask_{true, true, true, true};
    rclcpp::Time gait_mask_stamp_;
    bool gait_mask_received_ = false;
};

} // namespace dog2_mpc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dog2_mpc::MPCNodeComplete>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
