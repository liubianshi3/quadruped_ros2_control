# Dog2 四足蜘蛛机器人项目说明（read.md）

> 基于 2026-03-18 ~ 2026-03-19 的开发日志与当前仓库状态整理。本文作为“启动、排障、参数一致性”的单页入口。

## 1. 项目目标与当前阶段

本项目围绕 **Gazebo Fortress + ros2_control + 自研步态/IK 控制器** 构建四足蜘蛛形机器人仿真闭环，面向“跨越窗框型障碍”任务。

当前迭代的核心成果：
- 已打通并稳定启动链路（仿真、模型、控制器、上层控制节点）。
- 已完成腿序/命名/几何坐标的同源对齐，减少“错腿驱动”和 IK 假超界问题。
- 已把 URDF 历史偏移问题收敛为“唯一补偿层 + 可执行边界校验”。

---

## 2. 单一真源（必须遵守）

### 2.1 机体与腿序语义
- 机头朝向：`-X`
- 腿序：`1=LF, 2=LH, 3=RH, 4=RF`
- 映射代码定义：`LEG_PREFIX_MAP = {1: 'lf', 2: 'lh', 3: 'rh', 4: 'rf'}`。  
  参考：`src/dog2_motion_control/dog2_motion_control/joint_names.py`

### 2.2 关节语义命名
- 每条腿 4 自由度：`rail / coxa / femur / tibia`
- 旋转关节类型定义为 `['coxa', 'femur', 'tibia']`，并用于统一生成 16 个关节名。  
  参考：`src/dog2_motion_control/dog2_motion_control/joint_names.py`

### 2.3 URDF 偏移边界
- `urdf_shift_*` 只允许在 `base_offset_joint` 使用（一次性补偿）。
- `base_link` 与腿根 origin 保持 base_link-local 常量表达。  
  参考：`src/dog2_description/urdf/dog2.urdf.xacro`

---

## 3. 启动链路（推荐）

主入口：
```bash
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py
```

该 launch 已包含以下关键逻辑：
1. 启动 Gazebo（GUI/headless 都支持，默认 `-r` 自动运行）。
2. xacro 生成 `robot_description` 并启动 `robot_state_publisher`。
3. 显式桥接 `/clock`，降低“sim time not advancing”问题。
4. 生成机器人后按序加载控制器：
   - `joint_state_broadcaster`
   - `joint_trajectory_controller`
   - `rail_position_controller`
5. 最后启动 `spider_robot_controller`。  
   参考：`src/dog2_motion_control/launch/spider_gazebo_complete.launch.py`

---

## 4. 近期关键修复（按问题域）

### 4.1 IK Failure（工作空间越界）
- 现象：持续 `Target out of workspace`，控制器回退站立并循环失败。
- 处理：收敛外展偏移（`y_offset`）使目标落足点回到可达域，保持 `body_height` 稳定，避免引入新干涉。

### 4.2 命名不一致导致的控制器/运行时错误
- 典型问题：
  - 控制器激活超时
  - command interface 不匹配
  - `KeyError: 'coxa'` / `KeyError: 'haa'`
- 修复方向：
  - URDF、`ros2_controllers.yaml`、`joint_names.py`、`leg_parameters.py` 同步到 `rail/coxa/femur/tibia`。
  - 调试信息发布路径同步新键名。  
  参考：
  - `src/dog2_description/config/ros2_controllers.yaml`
  - `src/dog2_motion_control/dog2_motion_control/joint_names.py`
  - `src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`

### 4.3 仿真时钟启动竞态
- 现象：控制器已激活，但上层误判时间未推进，状态机越级。
- 修复：在起立流程中等待 `/clock` 就绪，再执行 standup 轨迹。  
  参考：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`

### 4.4 URDF Shift 双重补偿风险
- 新增边界脚本校验：xacro 展开后检查 `base_link` 与四个 `*_rail_joint` 的期望原点。
- 支持 `--tolerance/--tol` 和 `--strict`。  
  参考：`src/dog2_description/scripts/check_urdf_shift_boundary.py`

---

## 5. 参数与配置现状

### 5.1 步态配置
- 控制频率：`50 Hz`
- `leg_sequence = [leg1, leg2, leg3, leg4]`
- 相位偏移：`0/90/180/270`
- 已按 LF→LH→RH→RF 对齐 base positions/rotations。  
  参考：
  - `src/dog2_motion_control/config/gait_params.yaml`
  - `src/dog2_motion_control/config/gait_params_ros.yaml`

> 注：部分配置项仍保留 `haa/hfe/kfe` 历史字段命名（配置层），而运行时代码已迁移到 `coxa/femur/tibia` 语义。后续建议彻底收口，避免认知混淆。

### 5.2 描述包测试开关
- `DOG2_ENABLE_URDF_SHIFT_BOUNDARY_CHECK` 默认 `ON`。
- 在 `BUILD_TESTING` 下自动注册 `dog2_urdf_shift_boundary_check`。  
  参考：`src/dog2_description/CMakeLists.txt`

---

## 6. 建议验证流程（最小回归）

### 6.1 构建与启动
```bash
colcon build --packages-select dog2_description dog2_motion_control --symlink-install
source install/setup.bash
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py
```

### 6.2 运行中检查
```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

预期：
- 三个控制器均为 active。
- 能读到有效 `joint_states`。

### 6.3 URDF 边界严格校验
```bash
python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict
```

预期输出：
- `[PASS] URDF shift boundary checks passed (mode=strict, tol=1e-06).`

---

## 7. 后续工作建议

1. 做一次全仓“旧命名残留”自动扫描（尤其 `haa/hfe/kfe` 在配置/脚本/文档层的残留）。
2. 将“命名一致性 + 腿序一致性 + 时钟一致性 + manager 目标一致性”固化为启动前自检脚本。
3. 在稳定 IK 解域基础上引入 rail 位移优化，支撑重心迁移（面向窗框越障）。
4. 与 MPC/WBC 接口进一步对齐，复用 `rail/coxa/femur/tibia` 语义输入。

---

## 8. 一句话总结

这轮工作的核心价值不是“单点调参成功”，而是把 **物理拓扑、URDF 描述、控制器命名、上层状态机** 统一到同一语义体系，并通过可执行检查把一致性变成默认约束。
