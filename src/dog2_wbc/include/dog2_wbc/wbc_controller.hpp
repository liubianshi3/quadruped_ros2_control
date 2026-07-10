#ifndef DOG2_WBC_CONTROLLER_HPP
#define DOG2_WBC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>

#include "dog2_dynamics/dog2_model.hpp"

namespace dog2_wbc
{

/**
 * @brief 完整的WBC控制器
 *
 * 功能：
 * 1. 精确的雅可比计算（基于当前关节角度）
 * 2. 考虑滑动副影响
 * 3. 混合构型支持（膝式/肘式）
 * 4. QP优化的力矩分配
 */
class WBCController
{
public:
  enum class LegConfiguration
  {
    ELBOW,      // 肘式（标准）
    KNEE        // 膝式（穿越后）
  };

  struct LegState
  {
    Eigen::Vector3d joint_angles;         // 关节角度 [hip_roll, hip_pitch, knee]
    Eigen::Vector3d joint_velocities = Eigen::Vector3d::Zero();
    Eigen::Vector3d swing_pos_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d swing_vel_des = Eigen::Vector3d::Zero();
    double sliding_position;              // 滑动副位置
    LegConfiguration config;              // 构型
    bool in_contact;                      // 是否接触地面
    bool swing = false;
  };

  struct Parameters
  {
    // 腿部几何参数
    double l1;      // 大腿长度
    double l2;      // 小腿长度
    double hip_offset_x;      // 髋关节x偏移
    double hip_offset_y;      // 髋关节y偏移

    // 力矩限制
    double max_torque;
    double max_sliding_force;
    double foot_force_sign;
    bool gravity_compensation;
    Eigen::Vector3d swing_kp;
    Eigen::Vector3d swing_kd;
    double joint_limit_margin;
    double joint_limit_kp;
    double joint_limit_kd;

    // QP权重
    double w_dynamics;          // 动力学一致性权重
    double w_foot_tracking;     // 足端跟踪权重
    double w_regularization;     // 正则化权重

    Parameters()
    : l1(0.2), l2(0.2),
      hip_offset_x(0.2), hip_offset_y(0.15),
      max_torque(50.0), max_sliding_force(100.0),
      foot_force_sign(1.0),
      gravity_compensation(true),
      swing_kp(300.0, 300.0, 300.0),
      swing_kd(8.0, 8.0, 8.0),
      joint_limit_margin(0.12),
      joint_limit_kp(35.0),
      joint_limit_kd(1.5),
      w_dynamics(1000.0), w_foot_tracking(10.0),
      w_regularization(0.01) {}
  };

  WBCController(const Parameters & params = Parameters());

  void initializeFromRobotDescription(const std::string & robot_description);
  bool hasRobotModel() const {return static_cast<bool>(dog2_model_);}

  /**
   * @brief 设置躯干姿态（世界系 <- base 系旋转矩阵 R_wb）
   *
   * MPC 足端力在世界系表达，而本控制器的雅可比/重力项都在 base 系。
   * 躯干一旦俯仰/横滚，必须用 R_wb^T 把力旋回 base 系，否则竖直支撑力
   * 会按 sin(tilt) 比例泄漏成水平推搡（run15 交接翻倒的动力学来源）。
   * 未设置时默认单位阵（水平躯干，行为与旧版一致）。
   */
  void setBaseOrientation(const Eigen::Matrix3d & R_wb) {R_wb_ = R_wb;}

  /**
   * @brief 计算关节力矩
   * @param foot_forces 期望的足端力 (12维)
   * @param leg_states 四条腿的状态
   * @return 关节力矩 (12维旋转关节 + 4维滑动副)
   */
  Eigen::VectorXd computeTorques(
    const Eigen::VectorXd & foot_forces,
    const std::array<LegState, 4> & leg_states);

  /**
   * @brief 计算单条腿的雅可比矩阵
   * @param leg_id 腿编号 (0-3)
   * @param leg_state 腿状态
   * @return 雅可比矩阵 J (3×4): [∂p/∂q1, ∂p/∂q2, ∂p/∂q3, ∂p/∂d]
   */
  Eigen::MatrixXd computeLegJacobian(
    int leg_id,
    const LegState & leg_state);

  /**
   * @brief 正运动学：关节角度 → 足端位置
   * @param leg_id 腿编号
   * @param leg_state 腿状态
   * @return 足端位置（机体坐标系）
   */
  Eigen::Vector3d forwardKinematics(
    int leg_id,
    const LegState & leg_state);

  /**
   * @brief 更新参数
   */
  void updateParameters(const Parameters & params) {params_ = params;}

  /**
   * @brief 获取上次求解的统计信息
   */
  struct SolveStats
  {
    double solve_time_ms;
    double torque_norm;
    bool success;
  };
  SolveStats getLastSolveStats() const {return last_stats_;}

private:
  /**
   * @brief 计算肘式构型的雅可比
   */
  Eigen::MatrixXd computeElbowJacobian(
    int leg_id,
    const Eigen::Vector3d & joint_angles,
    double sliding_position);

  /**
   * @brief 计算膝式构型的雅可比
   */
  Eigen::MatrixXd computeKneeJacobian(
    int leg_id,
    const Eigen::Vector3d & joint_angles,
    double sliding_position);

  Eigen::MatrixXd computeUrdfJacobian(
    int leg_id,
    const LegState & leg_state);

  Eigen::VectorXd buildPinocchioConfiguration(
    int leg_id,
    const LegState & leg_state) const;

  Eigen::VectorXd buildFullPinocchioConfiguration(
    const std::array<LegState, 4> & leg_states) const;

  int jointVelocityIndex(const std::string & joint_name) const;

  /**
   * @brief 应用力矩限制
   */
  void applyTorqueLimits(Eigen::VectorXd & torques);

  Parameters params_;
  SolveStats last_stats_;
  std::unique_ptr<dog2_dynamics::Dog2Model> dog2_model_;
  Eigen::Matrix3d R_wb_ = Eigen::Matrix3d::Identity();
};

} // namespace dog2_wbc

#endif // DOG2_WBC_CONTROLLER_HPP
