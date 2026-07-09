#ifndef DOG2_DYNAMICS_DOG2_MODEL_HPP
#define DOG2_DYNAMICS_DOG2_MODEL_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>

namespace dog2_dynamics
{

/**
 * @brief Dog2机器人动力学模型
 *
 * 支持混合关节类型（4个滑动副 + 12个旋转关节）
 * 使用Pinocchio库进行高效的动力学计算
 */
class Dog2Model
{
public:
  /**
   * @brief 构造函数
   * @param urdf_path URDF文件路径
   */
  explicit Dog2Model(const std::string & urdf_path);

  static Dog2Model fromUrdfXml(const std::string & urdf_xml);

  // ========== 基本属性 ==========

  /**
   * @brief 获取配置空间维度（关节位置）
   * @return 配置空间维度（当前 Dog2 Pinocchio 模型为16维）
   */
  int nq() const {return model_.nq;}

  /**
   * @brief 获取速度空间维度（关节速度）
   * @return 速度空间维度（Dog2为16维）
   */
  int nv() const {return model_.nv;}

  /**
   * @brief 获取机器人总质量
   * @return 总质量（kg）
   */
  double mass() const;

  // ========== 正向运动学 ==========

  /**
   * @brief 计算正向运动学（位置）
   * @param q 关节位置（nq维）
   */
  void forwardKinematics(const Eigen::VectorXd & q);

  /**
   * @brief 计算正向运动学（位置+速度）
   * @param q 关节位置
   * @param v 关节速度（nv维）
   */
  void forwardKinematics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & v);

  /**
   * @brief 计算正向运动学（位置+速度+加速度）
   * @param q 关节位置
   * @param v 关节速度
   * @param a 关节加速度（nv维）
   */
  void forwardKinematics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & v,
    const Eigen::VectorXd & a);

  // ========== 质心计算 ==========

  /**
   * @brief 计算质心位置
   * @param q 关节位置
   * @return 质心位置（3维向量）
   */
  Eigen::Vector3d centerOfMass(const Eigen::VectorXd & q);

  /**
   * @brief 计算质心速度
   * @param q 关节位置
   * @param v 关节速度
   * @return 质心速度（3维向量）
   */
  Eigen::Vector3d centerOfMassVelocity(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & v);

  // ========== 足端运动学 ==========

  /**
   * @brief 计算单个足端位置
   * @param foot_name 足端名称（"rh_foot_link", "rf_foot_link", "lf_foot_link", "lh_foot_link"）
   * @param q 关节位置
   * @return 足端位置（3维向量）
   */
  Eigen::Vector3d footPosition(
    const std::string & foot_name,
    const Eigen::VectorXd & q);

  /**
   * @brief 计算所有足端位置
   * @param q 关节位置
   * @return 4个足端位置的向量
   */
  std::vector<Eigen::Vector3d> allFootPositions(const Eigen::VectorXd & q);

  /**
   * @brief 计算足端雅可比矩阵
   * @param foot_name 足端名称
   * @param q 关节位置
   * @return 足端雅可比矩阵（6×nv）
   */
  Eigen::MatrixXd footJacobian(
    const std::string & foot_name,
    const Eigen::VectorXd & q);

  /**
   * @brief 计算质心雅可比矩阵
   * @param q 关节位置
   * @return 质心雅可比矩阵（3×nv）
   */
  Eigen::MatrixXd comJacobian(const Eigen::VectorXd & q);

  // ========== 动力学计算 ==========

  /**
   * @brief 计算质量矩阵 M(q)
   * @param q 关节位置
   * @return 质量矩阵（nv×nv）
   */
  Eigen::MatrixXd massMatrix(const Eigen::VectorXd & q);

  /**
   * @brief 计算非线性效应项 C(q,v)·v + g(q)
   * @param q 关节位置
   * @param v 关节速度
   * @return 非线性效应项（nv维）
   */
  Eigen::VectorXd nonlinearEffects(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & v);

  /**
   * @brief 计算重力项 g(q)
   * @param q 关节位置
   * @return 重力项（nv维）
   */
  Eigen::VectorXd gravityVector(const Eigen::VectorXd & q);

  /**
   * @brief 计算指定重力方向下的重力项 g(q)
   *
   * 固定基座模型默认重力沿 base 系 -z。躯干一旦俯仰/横滚，真实重力在
   * base 系中的方向随之旋转，调用方应传入 R_wb^T * (0,0,-9.81)。
   * @param q 关节位置
   * @param gravity_linear base 系下的重力加速度向量
   * @return 重力项（nv维）
   */
  Eigen::VectorXd gravityVector(
    const Eigen::VectorXd & q,
    const Eigen::Vector3d & gravity_linear);

  // ========== 滑动副专用接口（Dog2特有）==========

  /**
   * @brief 滑动副状态结构体
   */
  struct SlidingJointState
  {
    Eigen::Vector4d positions;       ///< 滑动副位置 [j1, j2, j3, j4] (m)
    Eigen::Vector4d velocities;      ///< 滑动副速度 (m/s)
    Eigen::Vector4d forces;          ///< 滑动副驱动力 (N)
  };

  /**
   * @brief 获取滑动副状态
   * @param q 关节位置
   * @param v 关节速度
   * @return 滑动副状态
   */
  SlidingJointState getSlidingJointState(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & v) const;

  /**
   * @brief 获取滑动副下限
   * @return 滑动副位置下限（4维向量）
   */
  Eigen::Vector4d slidingJointLowerLimits() const;

  /**
   * @brief 获取滑动副上限
   * @return 滑动副位置上限（4维向量）
   */
  Eigen::Vector4d slidingJointUpperLimits() const;

  /**
   * @brief 获取滑动副速度限制
   * @return 滑动副最大速度（m/s）
   */
  double slidingJointMaxVelocity() const {return 1.0;}

  // ========== 足端名称常量 ==========

  static constexpr const char * FOOT_NAMES[4] = {
    "rh_foot_link",      // Right Hind
    "rf_foot_link",      // Right Front
    "lf_foot_link",      // Left Front
    "lh_foot_link"       // Left Hind
  };

  static constexpr const char * RAIL_JOINT_NAMES[4] = {
    "lf_rail_joint",
    "lh_rail_joint",
    "rh_rail_joint",
    "rf_rail_joint"
  };

  /**
   * @brief 获取Pinocchio模型（只读）
   */
  const pinocchio::Model & getModel() const {return model_;}

  /**
   * @brief 获取Pinocchio数据（只读）
   */
  const pinocchio::Data & getData() const {return data_;}

private:
  Dog2Model() = default;
  void finishInitialization(const std::string & source_label);

  pinocchio::Model model_;    ///< Pinocchio模型
  pinocchio::Data data_;      ///< Pinocchio数据

  /// 足端frame ID缓存（加速查找）
  std::vector<pinocchio::FrameIndex> foot_frame_ids_;

  /**
   * @brief 缓存足端frame ID
   */
  void cacheFrameIds();
};

} // namespace dog2_dynamics

#endif // DOG2_DYNAMICS_DOG2_MODEL_HPP
