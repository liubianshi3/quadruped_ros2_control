#include "dog2_mpc/mpc_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

namespace dog2_mpc {

/**
 * @brief 16维MPC节点 - 完整闭环控制
 * 
 * 功能：
 * 1. 订阅机器人状态（位置、姿态、速度、滑动副）
 * 2. 运行16维MPC求解最优足端力
 * 3. 发布足端力给WBC
 * 4. 支持滑动副控制
 */
class MPCNode16D : public rclcpp::Node {
public:
    MPCNode16D() : Node("mpc_node_16d") {
        // 声明参数
        this->declare_parameter("mass", 11.8);
        this->declare_parameter("horizon", 10);
        this->declare_parameter("dt", 0.05);
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("enable_sliding_constraints", true);
        this->declare_parameter("enable_boundary_constraints", false);
        this->declare_parameter("rail_tracking_error_threshold", 0.005);
        this->declare_parameter("support_polygon_margin_threshold", 0.015);
        this->declare_parameter("crossing_transition_stable_time", 0.15);
        this->declare_parameter("slack_linear_weight", 1e5);
        
        // 获取参数
        double mass = this->get_parameter("mass").as_double();
        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        double control_freq = this->get_parameter("control_frequency").as_double();
        
        // 惯性张量
        Eigen::Matrix3d inertia;
        inertia << 0.0153, 0.00011, 0.0,
                   0.00011, 0.052, 0.0,
                   0.0, 0.0, 0.044;
        
        // 配置16维MPC参数
        MPCController::Parameters params;
        params.horizon = horizon;
        params.dt = dt;
        params.enable_sliding_constraints = 
            this->get_parameter("enable_sliding_constraints").as_bool();
        params.enable_boundary_constraints = 
            this->get_parameter("enable_boundary_constraints").as_bool();
        
        // 16维状态权重
        params.Q = Eigen::MatrixXd::Identity(16, 16);
        params.Q.diagonal() << 100, 100, 200,  // 位置 [x, y, z]
                              50, 50, 50,       // 姿态 [roll, pitch, yaw]
                              10, 10, 10,       // 线速度 [vx, vy, vz]
                              5, 5, 5,          // 角速度 [wx, wy, wz]
                              50, 50, 50, 50;   // 滑动副 [j1, j2, j3, j4]
        
        // 控制权重
        params.R = Eigen::MatrixXd::Identity(12, 12) * 0.01;
        
        // 控制输入约束
        params.u_min = Eigen::VectorXd::Constant(12, -100.0);
        params.u_max = Eigen::VectorXd::Constant(12, 100.0);
        
        // 创建16维MPC控制器
        mpc_controller_ = std::make_unique<MPCController>(mass, inertia, params);

        // rail soft-bound exact penalty 一次项权重（支持动态调参）
        mpc_controller_->setSlackLinearWeight(
            this->get_parameter("slack_linear_weight").as_double());

        // 设置越障状态机 guard 参数（即使未启用 crossing，保持接口一致）
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
        
        // 初始化16维参考轨迹（悬停）
        std::vector<Eigen::VectorXd> x_ref(horizon, Eigen::VectorXd::Zero(16));
        for (auto& x : x_ref) {
            x(2) = 0.3;  // 期望高度 30cm
            // 滑动副保持为0
        }
        mpc_controller_->setReference(x_ref);
        
        // 初始化基础足端位置
        Eigen::MatrixXd base_foot_positions(4, 3);
        base_foot_positions << -0.2, -0.15, -0.3,  // 腿1
                               0.2, -0.15, -0.3,   // 腿2
                               0.2,  0.15, -0.3,   // 腿3
                              -0.2,  0.15, -0.3;   // 腿4
        mpc_controller_->setBaseFootPositions(base_foot_positions);
        
        // 初始化滑动副速度（零速度）
        Eigen::Vector4d sliding_velocity = Eigen::Vector4d::Zero();
        mpc_controller_->setSlidingVelocity(sliding_velocity);
        
        // 订阅机器人状态
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dog2/odom", 10,
            std::bind(&MPCNode16D::odomCallback, this, std::placeholders::_1));
        
        // 订阅关节状态（获取滑动副位置）
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MPCNode16D::jointCallback, this, std::placeholders::_1));
        
        // 发布足端力（给WBC）
        foot_force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/dog2/mpc/foot_forces", 10);
        
        // 创建控制定时器
        auto timer_period = std::chrono::duration<double>(1.0 / control_freq);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&MPCNode16D::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "16D MPC Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Mass: %.2f kg", mass);
        RCLCPP_INFO(this->get_logger(), "  Horizon: %d", horizon);
        RCLCPP_INFO(this->get_logger(), "  dt: %.3f s", dt);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_freq);
        RCLCPP_INFO(this->get_logger(), "  Sliding constraints: %s", 
                   params.enable_sliding_constraints ? "enabled" : "disabled");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 提取SRBD状态（12维）
        Eigen::VectorXd srbd_state(12);
        
        // 位置
        srbd_state(0) = msg->pose.pose.position.x;
        srbd_state(1) = msg->pose.pose.position.y;
        srbd_state(2) = msg->pose.pose.position.z;
        
        // 姿态（四元数转欧拉角）
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // Roll
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        srbd_state(3) = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            srbd_state(4) = std::copysign(M_PI / 2, sinp);
        else
            srbd_state(4) = std::asin(sinp);
        
        // Yaw
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        srbd_state(5) = std::atan2(siny_cosp, cosy_cosp);
        
        // 线速度
        srbd_state(6) = msg->twist.twist.linear.x;
        srbd_state(7) = msg->twist.twist.linear.y;
        srbd_state(8) = msg->twist.twist.linear.z;
        
        // 角速度
        srbd_state(9) = msg->twist.twist.angular.x;
        srbd_state(10) = msg->twist.twist.angular.y;
        srbd_state(11) = msg->twist.twist.angular.z;
        
        current_srbd_state_ = srbd_state;
        odom_received_ = true;
    }
    
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 提取滑动副位置
        Eigen::Vector4d sliding_positions = Eigen::Vector4d::Zero();
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "j1" || msg->name[i] == "lf_rail_joint") {
                sliding_positions(0) = msg->position[i];
            } else if (msg->name[i] == "j2" || msg->name[i] == "lh_rail_joint") {
                sliding_positions(1) = msg->position[i];
            } else if (msg->name[i] == "j3" || msg->name[i] == "rh_rail_joint") {
                sliding_positions(2) = msg->position[i];
            } else if (msg->name[i] == "j4" || msg->name[i] == "rf_rail_joint") {
                sliding_positions(3) = msg->position[i];
            }
        }
        
        current_sliding_positions_ = sliding_positions;
        joint_received_ = true;
    }
    
    void controlLoop() {
        if (!odom_received_ || !joint_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Waiting for state... (odom: %d, joint: %d)",
                               odom_received_, joint_received_);
            return;
        }

        // 动态读取 exact penalty 一次项权重
        mpc_controller_->setSlackLinearWeight(
            this->get_parameter("slack_linear_weight").as_double());
        
        // 构建16维扩展状态
        Eigen::VectorXd extended_state(16);
        extended_state.segment<12>(0) = current_srbd_state_;
        extended_state.segment<4>(12) = current_sliding_positions_;
        
        // 求解16维MPC
        Eigen::VectorXd u_optimal;
        bool success = mpc_controller_->solve(extended_state, u_optimal);
        
        if (!success) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "MPC solve failed, status: %d",
                                mpc_controller_->getSolveStatus());
            return;
        }
        
        // 发布足端力（12维：4条腿×3维力）
        auto force_msg = std_msgs::msg::Float64MultiArray();
        force_msg.data.resize(12);
        for (int i = 0; i < 12; ++i) {
            force_msg.data[i] = u_optimal(i);
        }
        foot_force_pub_->publish(force_msg);
        
        // 打印统计信息
        if (++control_count_ % 20 == 0) {  // 每秒打印一次（20Hz控制）
            RCLCPP_INFO(this->get_logger(),
                       "MPC: solve_time=%.2fms, status=%d, height=%.3fm, sliding=[%.3f,%.3f,%.3f,%.3f]",
                       mpc_controller_->getSolveTime(),
                       mpc_controller_->getSolveStatus(),
                       current_srbd_state_(2),
                       current_sliding_positions_(0),
                       current_sliding_positions_(1),
                       current_sliding_positions_(2),
                       current_sliding_positions_(3));
        }
    }
    
    std::unique_ptr<MPCController> mpc_controller_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr foot_force_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    Eigen::VectorXd current_srbd_state_;
    Eigen::Vector4d current_sliding_positions_;
    bool odom_received_ = false;
    bool joint_received_ = false;
    int control_count_ = 0;
};

} // namespace dog2_mpc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dog2_mpc::MPCNode16D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
