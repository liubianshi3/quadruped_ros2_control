# 🐾 Dog2 四足机器人 ROS 2 控制系统

## 🚀 项目简介

本项目是一个基于 **ROS 2 Humble** 和 **Gazebo Fortress** 的四足（蜘蛛形态）机器人运动控制系统。它旨在为 Dog2 机器人提供从基础运动控制、步态规划到高级模型预测控制 (MPC) 和全身控制 (WBC) 的完整解决方案，并在仿真环境中进行验证。

**核心目标**：
*   在 Gazebo Fortress 仿真环境中实现 Dog2 机器人的稳定、动态运动。
*   开发一套模块化、可扩展的 ROS 2 控制软件栈。
*   集成先进的机器人控制算法，如爬行步态、逆运动学、MPC 和 WBC。
*   通过全面的测试和验证，确保系统的鲁棒性和安全性。

## 🏗️ 系统架构概览

系统采用清晰的分层架构设计，各模块通过 ROS 2 的话题 (Topic)、服务 (Service) 和动作 (Action) 进行高效通信。以下是系统的高层架构图：

![ROS 2 Control Architecture](ros2_control_architecture.png)

### 逻辑分层

| 层次 | 核心功能 | 关键 ROS 2 包 |
| :--- | :--- | :--- |
| **仿真/模型层** | 机器人 URDF 模型、Gazebo 仿真环境配置、`ros2_control` 接口 | `dog2_description` |
| **高层运动控制层** | 步态生成、轨迹规划、逆运动学 (IK) 求解、关节指令下发 | `dog2_motion_control` |
| **优化控制层** | 模型预测控制 (MPC)、全身控制 (WBC) | `dog2_mpc`, `dog2_wbc` |
| **支撑能力层** | 动力学模型、运动学库、状态估计、可视化工具、接口定义 | `dog2_kinematics`, `dog2_dynamics`, `dog2_state_estimation`, `dog2_visualization`, `dog2_interfaces` |

## ✨ 核心功能与技术亮点

### 1. 爬行步态 (Crawl Gait)
*   实现了高度稳定的爬行步态，确保机器人始终有三条腿支撑地面。
*   通过精确的相位控制，实现腿部依次摆动：`Leg 1` → `Leg 3` → `Leg 2` → `Leg 4`。
*   **无状态轨迹生成**：足端轨迹完全由当前步态相位决定，避免了传统方法中可能出现的累积误差和漂移问题。

### 2. 解析逆运动学 (Inverse Kinematics - IK)
*   针对 Dog2 机器人每条腿的 4-DOF 结构（1个导轨关节 + 3个旋转关节），开发了高效的解析 IK 求解器。
*   利用几何法和三角函数（如余弦定理）直接计算关节角度，计算速度快（通常小于 10ms），适用于实时控制。
*   **安全性与鲁棒性**：集成了工作空间检查和关节限位检查，并在 IK 求解失败时具备回退机制。

### 3. 先进控制算法 (MPC & WBC)
*   **模型预测控制 (MPC)**：在 `dog2_mpc` 包中实现，利用单刚体动力学 (SRBD) 模型和 OSQP 求解器，优化未来时间步的控制输入，实现动态平衡和运动规划。
*   **全身控制 (WBC)**：在 `dog2_wbc` 包中实现，用于协调机器人的所有关节，处理多任务优先级，如保持躯干姿态、足端力控制等。

![MPC WBC Hierarchical Control](mpc_wbc_hierarchical_control.png)

### 4. ROS 2 `ros2_control` 集成
*   系统与 `ros2_control` 框架深度集成，通过 `joint_trajectory_controller` 控制旋转关节，并通过 `rail_position_controller` 锁定导轨关节，实现精确的硬件接口控制。
*   所有控制循环均与 Gazebo Fortress 的仿真时钟同步，确保仿真与实际控制逻辑的一致性。

### 5. 启动与安全机制
*   **一键启动脚本**：`start_fortress.sh` 脚本简化了编译、环境配置和仿真启动过程。
*   **分层延迟启动**：精心设计的启动时序确保 Gazebo、机器人模型、`ros2_control` 控制器和主控制器按正确顺序和时间间隔启动，避免时序问题。
*   **平滑起立与紧急下降**：实现了机器人从初始姿态平滑站立，以及在紧急情况下安全下降到稳定姿态的功能。

## 🛠️ 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/e17515135916-cmd/quadruped_ros2_control.git
cd quadruped_ros2_control
```

### 2. 编译与安装依赖

本项目依赖 ROS 2 Humble 和 Gazebo Fortress。请确保您的系统已正确安装这些环境。

```bash
# 假设您在 ROS 2 工作空间 (e.g., ~/aperfect/carbot_ws) 的 src 目录下克隆了本仓库
cd ~/aperfect/carbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动 Gazebo Fortress 仿真

使用提供的一键启动脚本来启动整个仿真环境和机器人控制系统：

```bash
cd ~/aperfect/carbot_ws
bash start_fortress.sh
```

该脚本将自动完成以下步骤：
1.  重新编译 `dog2_motion_control` 包。
2.  Source ROS 2 环境。
3.  检查 Gazebo Fortress 并启动完整的仿真系统。

请耐心等待约 **10-15 秒**，直到所有组件（Gazebo 窗口、机器人模型、控制器、主控制器）依次启动。

### 4. 验证系统运行

在**另一个终端**中，您可以检查 ROS 2 节点、话题和控制器状态：

```bash
ros2 topic list
ros2 node list
ros2 control list_controllers
```

### 5. 测试机器人运动

向 `/cmd_vel` 话题发布速度命令，观察机器人在 Gazebo 中的运动：

```bash
# 发送前进命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}}" --once

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## ✅ 测试与验证

本项目高度重视代码质量和系统鲁棒性，通过了全面的测试：
*   **单元测试**：119 个测试用例，通过率达 **97.5%**。
*   **需求满足度**：8 项主要需求，100% 满足。
*   **正确性属性**：22 项属性测试，100% 通过。
*   **系统完整性**：15 项系统完整性检查，100% 通过。

## 📚 更多文档

*   **项目架构详情**：[`PROJECT_ARCHITECTURE.md`](PROJECT_ARCHITECTURE.md)
*   **快速启动指南**：[`START_HERE.md`](START_HERE.md)
*   **算法开发汇报**：[`ALGORITHM_DEVELOPMENT_PPT.md`](ALGORITHM_DEVELOPMENT_PPT.md)
*   **ROS 2 Control 数据流图**：[`ROS2_CONTROL_DATA_FLOW_DIAGRAM.md`](ROS2_CONTROL_DATA_FLOW_DIAGRAM.md)

## 🤝 贡献与维护

欢迎对本项目提出建议或贡献代码。请遵循以下开发规范：
*   **模块化设计**：保持每个 ROS 2 包和模块的职责单一。
*   **测试驱动**：在开发新功能时，优先编写测试用例。
*   **文档先行**：及时更新相关文档，确保代码与文档同步。

--- 

**作者**: 路志强
**最后更新**: 2026年3月12日
