# Dog2 四足机器人工作区

这是一个基于 ROS 2 的自研四足（蜘蛛式）机器人工作区，包含：
- 机器人描述（URDF/Xacro + ros2_control）
- 运动控制与步态生成
- MPC / WBC / 动力学模块
- 仿真与可视化支持

> 主要目标：在 Gazebo Fortress 中实现低姿态稳定行走与障碍跨越。

---

## 项目结构

```text
carbot_ws/
├── src/
│   ├── dog2_description/        # URDF/Xacro、ros2_control 配置、launch、模型脚本
│   ├── dog2_motion_control/     # 步态生成器、蜘蛛机器人控制器、启动文件
│   ├── dog2_mpc/                # 模型预测控制模块
│   ├── dog2_wbc/                # 全身控制模块
│   ├── dog2_dynamics/           # 机器人动力学模块
│   ├── dog2_state_estimation/   # 状态估计模块
│   ├── dog2_gait_planner/       # 步态规划模块
│   ├── dog2_visualization/      # RViz 与可视化相关工具
│   ├── dog2_interfaces/         # ROS 2 公共接口定义
│   └── dog2_demos/              # 示例节点 / 演示脚本
└── DEVELOPMENT_LOG.md           # 工程开发日志与问题排查记录
```

---

## 环境要求

- Ubuntu 22.04（推荐）
- ROS 2 Humble
- Gazebo Fortress（gz）
- Python 3.10+
- CMake / colcon

---

## 构建

在工作区根目录（`carbot_ws`）执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

仅构建指定包（可选）：

```bash
colcon build --packages-select dog2_description dog2_motion_control --symlink-install
source install/setup.bash
```

---

## 运行仿真

启动完整链路（仿真 + 控制器 + 运动控制）：

```bash
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py
```

若只想查看机器人模型：

```bash
ros2 launch dog2_description display.launch.py
```

---

## 控制器命名约定

当前关节语义命名：
- `rail`
- `coxa`
- `femur`
- `tibia`

腿编号顺序约定：
- `leg1 = LF`（左前）
- `leg2 = LH`（左后）
- `leg3 = RH`（右后）
- `leg4 = RF`（右前）

请保持 URDF、控制器 YAML、运动控制映射三者一致。

---

## 常用检查命令

执行 URDF 偏移边界检查（严格模式）：

```bash
python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict
```

查看控制器状态：

```bash
ros2 control list_controllers
```

---

## 工作区清理建议

请勿提交以下构建产物：
- `build/`
- `install/`
- `log/`
- `**/*.egg-info/`
- `**/__pycache__/`

需要彻底清理并重建时：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 开发日志

详见 [`DEVELOPMENT_LOG.md`](./DEVELOPMENT_LOG.md)，其中记录了：
- 架构决策
- Bug 根因分析
- 参数调整历史
- 后续计划

---

## 许可证

请在此处补充你的许可证信息（如 MIT / Apache-2.0 / 闭源项目）。

---

## 维护者

- 你的姓名 / 团队名
- 联系方式：your-email@example.com
