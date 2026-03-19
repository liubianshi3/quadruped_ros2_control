# DEVELOPMENT_LOG

## 2026-03-18 今日工作概述
完成 Gazebo Fortress + ros2_control 启动链路的验证与修复，确保控制器成功激活并进入 50 Hz 主循环；针对“蜘蛛式外展步态”持续出现的 IK Failure 进行了系统性诊断，确认问题来源于目标落足点在 Y 方向超出可达工作空间，并将步态外展参数下调以逼近可行解。

### 💻 工程实现与参数调整
- **Gazebo / ros2_control 适配**
  - 将 URDF 中 ros2_control 插件切换为 `gz_ros2_control`，确保 Fortress 可正确加载控制器。
  - 纠正 `base_offset_joint` 的 Z 偏移，避免机器人整体下沉导致腿部穿地。
- **步态生成器（GaitGenerator）**
  - 旧版参数基线保持：`body_height=0.16`、`stride_length=0.02`。
  - 外展偏移调整：`y_offset = ±0.07`，使目标 Y 坐标从 0.2025 收敛到 0.1325，降低侧向外展引起的工作空间越界风险。
- **启动流程与控制器**
  - `spider_gazebo_complete.launch.py` 启动链路确认正常：`joint_state_broadcaster`、`joint_trajectory_controller`、`rail_position_controller` 皆激活。
  - 检查并规避重复启动 `spider_robot_controller` 导致的同名节点冲突。

### 🐛 Bug深度剖析与解决
- **错误现象**：控制器进入运行态后，四条腿连续报 `IK Failure`，提示 “Target out of workspace”。
- **运动学根因**：
  - 以旧版几何基座参数（`base_position.y = ±0.0625`）为基准，落足点目标在 Y 方向叠加外展偏移后达到 **0.1325 m**。该目标点在当前 3-DoF 解析 IK 下的可达区域外，导致余弦定理计算的三角不等式失效（等价于进入“工作空间死区/连杆无法闭合”区域）。
  - 当目标点超出可达范围时，IK 直接返回 None，主控制器被迫退回“站立姿态”，产生持续失败循环。
- **解决措施**：
  - 逐步降低 `y_offset`，将落足目标推回到可达空间；同时保持 `body_height` 不变，以避免引入新的垂向干涉或奇异位姿。
  - 通过日志确认 IK 目标点数值与几何模型一致，排除“旧版本未生效”的假象。

### 🚀 课题意义与下一步
本次修复聚焦于“跨越飞机油箱窗型障碍物”任务的基础支撑姿态稳定性：
- 通过收敛外展偏移，保障腿部目标点处于可达空间，从源头上减少 IK 失败，提高低姿态下的可控性。
- 低重心 + 宽支撑面是穿越窗框、抑制侧翻的核心物理条件；当前的参数收敛为后续 **直线导轨重心转移策略** 与 **MPC/WBC 约束优化** 提供了稳定可行的基线。

**下一步计划**：
1. 进一步核对 URDF 中腿基座几何与 `leg_parameters.py` 是否存在微小偏差，避免局部坐标系不一致造成的“假超界”。
2. 基于已稳定的 IK 解域，引入导轨位移作为优化变量，将 4-DoF 冗余度转化为跨越窗框时的重心迁移能力。
3. 与 MPC/WBC 控制层对接，验证低姿态下的稳态行走与侧向抗倾覆性能。

---

## 2026-03-19 今日工作概述
完成“腿序映射 + 参数同步 + URDF 统一平移”链路的梳理与合并，确保仿真模型、步态参数与控制器命名在新的腿编号顺序下保持一致；整理出待排查的残留文件范围，明确后续验证路径。

### 💻 工程实现与参数调整
- **URDF 统一重定位（Path A）**
  - 在 `dog2.urdf.xacro` 中引入 `urdf_shift_x/y/z` 统一平移量。
  - 平移应用于：`base_offset_joint`、`base_link` 的 inertial/visual/collision，以及 `j${leg_num}` 的关节 `origin`。
  - 修复 xacro 数学表达：将 `xyz` 字符串拆为 `xyz_x/xyz_y/xyz_z` 数值参数，避免下标计算报错。
- **腿编号与命名映射重排**
  - 统一腿编号顺序：`j1=LF`、`j2=LH`、`j3=RH`、`j4=RF`。
  - 同步 `joint_names.py`：
    - `LEG_PREFIX_MAP`: 1→lf, 2→lh, 3→rh, 4→rf
    - `PREFIX_TO_LEG_MAP`: lf→1, lh→2, rh→3, rf→4
- **步态参数同步**
  - `gait_params.yaml` / `gait_params_ros.yaml`：
    - `leg_sequence = [leg1, leg2, leg3, leg4]`
    - `phase_offsets` 依序设为 `0/90/180/270`
    - `leg_base_positions` 与 `leg_base_rotations` 按新腿序重排（LF→LH→RH→RF）
- **几何参数对齐**
  - `leg_parameters.py` 中 `leg_num/leg_id` 与 `base_position/base_rotation` 已按新编号顺序对齐。

### 🐛 Bug深度剖析与解决
- **问题本质**：腿编号重排后，URDF、步态参数和控制器映射若存在“旧顺序残留”，会导致目标点与关节命名错位，表现为 IK 无解、关节驱动错腿或力学异常。
- **根因分析**：
  - 多源配置（URDF、YAML、Python 配置）同时使用腿序，任何一处残留旧顺序都会造成坐标系与命名映射错配。
  - Xacro 中 `xyz` 使用字符串形式参与运算导致数学表达式出错，属于编译期静态失败。
- **解决手段**：
  - 统一引入 URDF 平移参数并修正 xacro 表达式，消除几何偏置与构建错误。
  - 将腿编号映射作为“单一真源”同步到 `joint_names.py` 与配置文件中，防止多处失配。

### 🚀 课题意义与下一步
新的腿序与几何一致性是实现“穿越窗框形障碍物”的基础前置条件：
- 只有当 URDF、控制器与步态顺序一致时，才能保证在低姿态外展状态下正确落足与重心分配，从而提升跨越窄窗框时的抗侧翻能力。
- 统一平移参数为后续引入导轨位移优化（重心转移）提供了稳定的坐标基准，支撑 MPC/WBC 的轨迹与约束建模。

**下一步计划**：
1. 对全工程执行“残留顺序扫描”（lf/rf/lh/rh 关键词），排查 MPC/WBC 与控制器配置中的旧顺序残留。
2. 重点核查 ros2_control 控制器参数与关节顺序是否与新腿序匹配。
3. 启动仿真验证新腿序下的站立姿态与 IK 可达域。
