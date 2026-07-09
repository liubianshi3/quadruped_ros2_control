# Dog2 线性导轨四足机器人 WBC+MPC 工作全记录

> 工作区：`/home/dell/aperfect/carbot_ws`  
> 记录日期：2026-07-08  
> 范围：仅 **dog2** 项目；不含中期汇报、Go2 窗框演示、画图等旁支任务。

---

## 1. 项目目标

在 **Gazebo Fortress + ROS 2 Humble** 仿真中，为带 **直线导轨（prismatic rail）** 的四足机器人 dog2 实现 **MPC + WBC** 研究控制栈，当前阶段目标是：

1. **导轨锁定**：平地行走时四条导轨固定在 0 m，漂移 ≤ 5 mm。
2. **稳定站立**：spawn 后能主动站起，机身高度约 0.26 m，COM 在支撑多边形内。
3. **平地行走**：WBC+MPC 闭环下能前进、转向；前进投影位移应明显大于横向/偏航漂移（用户明确要求“走得流畅”，而非仅通过宽松门限）。

远期目标（尚未稳定）：窗型障碍 **越障**（`window_crossing_test`），当前开发边界在 `CROSSING:BODY_FORWARD_SHIFT`。

---

## 2. 机器人与控制架构

### 2.1 机械与 URDF 语义

| 项目 | 说明 |
|------|------|
| 腿结构 | 每腿 **1P + 3R**：`rail`（ prismatic ）+ `coxa` / `femur` / `tibia`（ revolute ） |
| 腿序 | `lf → lh → rh → rf`（逆时针，机头 **−X**） |
| 16 通道顺序 | 每腿 `rail, coxa, femur, tibia`，四腿串联 |
| rail 正方向 | 机身 **+X**（向前伸出为正，但各腿限位符号不同） |
| 控制根 | `base_link`（semantic trunk root，承载 inertial + collision + visual） |

各腿 rail 限位（不可写统一常数）：

- `lf_rail_joint`: `[0.0, 0.111]`
- `lh_rail_joint`: `[-0.111, 0.0]`
- `rh_rail_joint`: `[0.0, 0.111]`
- `rf_rail_joint`: `[-0.111, 0.0]`

### 2.2 研究栈运行链路

```text
ros2 launch dog2_bringup smoke_test.launch.py
  controller_mode:=effort research_stack:=true
    → effort_research_sim.launch.py
        Gazebo（暂停启动 → 控制器就绪后 unpause）
        joint_state_broadcaster
        rail_position_controller   ← 导轨位置锁
        effort_controller          ← 12 旋转关节力矩
    → control_stack.launch.py
        sim_state_estimator_node
        gait_scheduler_node        → /dog2/gait/contact_phase
        swing_target_node          → 摆动足端轨迹
        mpc_node_complete          → /dog2/mpc/foot_forces
        wbc_node_complete          → /dog2/wbc/joint_effort_command
        wbc_effort_mux             → /effort_controller/commands
        rail_lock_commander        → /rail_position_controller/commands (全 0)
```

核心话题：

```text
/joint_states
/dog2/state_estimation/odom
/dog2/gait/contact_phase
/dog2/mpc/foot_forces
/dog2/wbc/joint_effort_command
/effort_controller/commands          (12 通道)
/rail_position_controller/commands   (4 通道，锁定 0)
/cmd_vel
```

### 2.3 控制分工

| 模块 | 职责 |
|------|------|
| **MPC** | SRBD 预测、足端力参考、高度/姿态/平面速度闭环、越障状态机 |
| **WBC** | 由足端力经 \(J^T F + g(q)\) 映射为关节力矩；HOVER 下 posture PD |
| **wbc_effort_mux** | 启动 hold + stand-up PD → 渐变切入 WBC；12/16 通道拼装 |
| **rail_position_controller** | 导轨 **position-only** 锁定 |
| **rail_lock_commander** | 持续发布 `[0,0,0,0]` 导轨目标 |
| **gait_scheduler** | trot 相位，发布 contact mask |
| **swing_target** | Raibert 落足 + Bezier 摆动轨迹 |

---

## 3. 工作阶段与时间线

### 阶段 A：基础设施与导轨锁（2026-07-08 上午，已验证 PASS）

**背景**：方案 A（`rail_position_controller` 位置锁 + 12 通道 `effort_controller`）代码早已存在，但从未真正跑通——导轨 spawn 落地后漂到 ~40 mm 并保持，位置控制器“激活成功”却不起作用。

#### A1. Pinocchio 3.9 → 4.0 迁移（apt 升级断裂）

**现象**：`mpc_node_complete` / `wbc_node_complete` 启动失败，缺 `libpinocchio_parsers.so.3.9.0`。

**修复**：`dog2_dynamics`、`dog2_mpc`、`dog2_wbc` 的 `CMakeLists.txt` 改为链接导入目标 `pinocchio::pinocchio`（含 deprecated 兼容头路径），清理 build/install 后重建。

#### A2. 导轨位置锁根因（gz_ros2_control 0.7.20 + Fortress Physics）

**根因链**：

1. 关节只要 **暴露** effort 命令接口，`initSim` 就会置 EFFORT 控制位，首个 write 周期给 **全部 16 关节**（含 4 导轨）创建 `JointForceCmd(0 N)`。
2. Fortress `Physics.cc` 每步只清零 `JointForceCmd`，**不移除组件**；命令分发为 `if (JointForceCmd) SetForce; else if (JointVelocityCmd) ...`，force **永远优先**。
3. `rail_position_controller` 写 `JointVelocityCmd`（位置伺服），但陈旧 `JointForceCmd` 让物理端永远忽略 → 导轨等效 **0 N 力控 = 完全自由**。

**修复**：

- `dog2_ros2_control.xacro` 新增 `dog2_rail_control_joint` 宏，4 导轨改为 **position-only** 命令接口。
- 附带：`include_rail_in_output=False`，mux 不再向 effort 通道写导轨力矩。

**文件**：`src/dog2_description/urdf/dog2_ros2_control.xacro`

#### A3. 导轨传动参数加固

行走触地冲击沿腿链在导轨轴向峰值力超过旧限 800 N，5 mm 预算内兜不住。

| 参数 | 旧值 | 新值 |
|------|------|------|
| `prismatic_effort` | 800 | **5000** |
| `prismatic_damping` | 0.25 | **8.0** |
| `prismatic_friction` | 0.05 | **1.0** |

**文件**：`dog2_symmetric_properties.xacro`、`dog2_properties.xacro`（两变体同步）

#### A4. 首次冒烟全绿（run / run2 / run6）

```bash
ros2 launch dog2_bringup smoke_test.launch.py \
  controller_mode:=effort research_stack:=true \
  expect_research_stack:=true \
  ros_domain_id:=44 \
  result_file:=/tmp/dog2_smoke_result.txt
```

| 指标 | 结果 | 门限 |
|------|------|------|
| 站立 XY 漂移 | ~0.015 m | ≤ 0.20 m |
| 前进投影位移 | ~0.90 m | ≥ 0.10 m |
| 转向 yaw Δ | ~3.1 rad | ≥ 0.25 rad |
| **max_rail_lock_err** | **0.0020 m** | **≤ 0.0050 m** |

**结论**：导轨锁定问题 **已解决**；但步态质量粗糙——前进阶段横向漂 ~1 m、偏航 −2.1 rad、身高 0.05~0.25 m 波动，门限内通过但不可用。

---

### 阶段 B：站立姿态与启动序列（2026-07-08 下午）

#### B1. 静态不稳定站立姿态

**问题**：旧 `standing_pose`（femur −0.3、tibia +0.6）下足端全在 COM 后方，支撑多边形与 COM 不相交，机器人只能“趴着”，stand 阶段必然漂移。

**方法**：用 Pinocchio FK + 支撑多边形检查搜索新姿态（脚本 `/tmp/find_stance.py`）。

**新站立角**（四腿相同）：

```yaml
hip_pitch_rad:  1.05   # femur
knee_pitch_rad: -1.10   # tibia
rail_m: 0.0
```

**静态结论**（对称模型，rail=0）：

- 足端 x：`[-0.052, +0.224, +0.224, -0.052]` m  
- 足端 z：约 **−0.268** m（相对 base_link）  
- COM x = **+0.108** m，在支撑多边形内，裕度 ≥ 0.11 m  
- 前后腿静力分配约 **42% / 58%**（COM 偏后）

**文件**：`src/dog2_motion_control/config/gait_params.yaml`

#### B2. spawn 即塌地（belly-flop）

**问题**：Gazebo 一 unpause，effort 关节为 0 N（limp），WBC 尚未接管，机器人直接拍地；smoke 在 stand 阶段量到身高 < 0.12 m。

**修复**：

1. **`effort_research_sim.launch.py`**：去掉 `gz_args` 中的 `-r`，改为 **暂停启动**；`effort_controller` 加载后再 `gz service ... pause: false` unpause。
2. **`wbc_effort_mux.py`**：启动 hold 阶段加入 **关节空间 PD stand-up**，从测量姿态渐变到 `startup_standup_pose`，叠加 per-leg 静力前馈力矩。
3. **`control_stack.launch.py`**：配置 hold 时间、PD 增益、12 通道静力前馈。

**启动 hold 静力前馈**（`startup_hold_joint_effort`，顺序 lf/lh/rh/rf × coxa/femur/tibia）：

```python
[0.0, 1.752, -4.568,   # lf
 0.0, 2.041, -6.102,   # lh
 0.0, 2.041, -6.102,   # rh
 0.0, 1.752, -4.568]   # rf
```

**Stand-up PD 参数**（`control_stack.launch.py`）：

| 参数 | 值 |
|------|-----|
| `startup_hold_sec` | 6.0 s |
| `startup_ramp_sec` | 2.0 s |
| `startup_standup_pose` | `[0.0, 1.05, -1.10]` |
| `startup_standup_pose_ramp_sec` | 4.0 s |
| `startup_standup_kp` | `[25, 60, 60]` |
| `startup_standup_kd` | `[1.0, 2.0, 2.0]` |
| `startup_standup_max_torque` | 33.0 N·m |

#### B3. smoke 增加 `wait_settle` 阶段

**问题**：话题/控制器就绪 ≠ 机器人已静止；在 stand-up 中途采样会把“起立 lurch”记成 stand 漂移。

**修复**：`smoke_check.py` 在 `wait_ready` 后增加 `wait_settle`：

- 至少等待 **10 s**（等 mux hold+ramp 结束）
- 连续 **2 s** 满足：身高 ∈ [0.12, 0.40] m、平面速度 < 0.08 m/s、高度变化率 < 0.05 m/s
- 超时 **40 s** 则 FAIL

**文件**：`src/dog2_bringup/config/smoke_test.yaml`（`body_height_min_m: 0.12` 等）

---

### 阶段 C：行走质量改进（WBC+MPC 闭环）

用户反馈：**“前进投影完全不合格，走的还没有漂移的多”**。以下为针对根因的系统性修改。

#### C1. 摆动足端目标错误（stomp & drag）

**根因**：`swing_target_node.py` 的 `NOMINAL_FOOTS` 使用陈旧坐标（z ≈ −0.465，比地面低 0.2 m；x 偏差最大 0.24 m），每步摆动都“跺地拖拽”。

**修复**：更新为 Pinocchio FK 真实足端（femur +1.05、tibia −1.10、rail=0）：

```python
NOMINAL_FOOTS = [
    [-0.052, -0.118, -0.268],  # lf
    [ 0.224, -0.118, -0.268],  # lh
    [ 0.224,  0.118, -0.268],  # rh
    [-0.052,  0.118, -0.268],  # rf
]
```

**文件**：`src/dog2_gait_planner/dog2_gait_planner/swing_target_node.py`

#### C2. 行走无平面速度闭环（开环前馈）

**根因**：MPC 行走仅 feed-forward，无 vx/vy/yaw 误差反馈；`flat_forward_assist` 是开环助力，无法抑制漂移。

**修复**：`mpc_node_complete.cpp` 新增 `applyWalkingPlanarStabilization()`：

- 在 **WALKING** 模式，根据 vx/vy/wz 误差对 **支撑腿** 施加切向 fx/fy
- 偏航误差通过左右足 fx 差形成力矩
- 仅在姿态正常时激活（tilt < 0.48 rad，body_up_z > 0.82）

**参数**（`research_mpc.yaml`）：

| 参数 | 值 | 含义 |
|------|-----|------|
| `walking_stabilization_enabled` | true | 总开关 |
| `walking_vx_kp` / `walking_vy_kp` | 200.0 | N/(m/s) |
| `walking_wz_kp` | 6.0 | 偏航力矩增益 |
| `walking_fx_max_per_leg` | 35.0 N | 单腿 fx 限幅 |
| `walking_fy_max_per_leg` | 30.0 N | 单腿 fy 限幅 |
| `flat_forward_assist_enabled` | **false** | 被平面闭环取代 |

#### C3. 行走无姿态支撑 + 垂直力分配错误

**根因 1**：`isSupportStabilizationActive()` 原先仅在 **CROSSING** 开启，WALKING 无 roll/pitch 支撑。

**修复**：WALKING 模式也启用 attitude support（通过支撑腿 fz 差分）。

**根因 2**：`flat_force_full_support=true` 使 MPC 按 **4 腿** 分配体重，但 trot 仅 **2 腿** 支撑 → 垂直力不足 → “pogo-sticking” 身高振荡。

**修复**：

- `flat_force_full_support: false`
- `applyVerticalSupport()` 改为按 **前后静力分配**（COM 相对足端）直接设定 fz，而非 `max(qp_fz, pd_fz)` 竞争
- `vertical_support_max_leg_force`: 75 → **95** N
- `vertical_support_min_total_force_multiplier`: 1.08 → **1.0**
- `hover_force_max_leg_fz`: 32 → **48** N（站立姿态支撑余量）

#### C4. 步态相位不同步

**根因**：MPC 内部相位与 `gait_scheduler` 不一致，WBC 只在 scheduler 认定的支撑腿施力，MPC 却按另一套 mask 预算力 → 力/接触错配。

**修复**：`mpc_node_complete` 订阅 `/dog2/gait/contact_phase`，新鲜时使用 `gait_contact_mask_`，否则回退内部相位钟。

**依赖**：`dog2_mpc/package.xml` 增加 `dog2_interfaces`。

#### C5. WBC posture PD 与行走对抗

**根因**：`applyPostureBias()` 在 WALKING 仍激活，关节 PD 与步态所需运动对抗。

**修复**：`wbc_node_complete.cpp` 中 posture PD **仅在 HOVER** 启用；行走交给 MPC 力参考 + WBC 映射。

**HOVER posture PD**（`control_stack.launch.py`）：

```yaml
posture_pd_enabled: true
posture_kp_coxa: 30.0
posture_kp_femur: 60.0
posture_kp_tibia: 60.0
posture_kd_coxa: 2.0
posture_kd_femur: 3.0
posture_kd_tibia: 3.0
posture_max_torque: 35.0
```

#### C6. MPC stance 足端参考

**修复**：`base_foot_positions_` 不再用 q=0 的错误 FK，改为从参数 `stance_joint_pose` 计算；增加 `com_stance_` 存储名义 COM。

---

## 4. 修改文件清单

### 4.1 描述与仿真

| 文件 | 改动摘要 |
|------|----------|
| `dog2_description/urdf/dog2_ros2_control.xacro` | 导轨 position-only；旋转关节 position+effort |
| `dog2_description/urdf/dog2_symmetric_properties.xacro` | 导轨 effort/damping/friction 加固 |
| `dog2_description/urdf/dog2_properties.xacro` | 同上（real 变体） |

### 4.2 构建

| 文件 | 改动摘要 |
|------|----------|
| `dog2_dynamics/CMakeLists.txt` | `pinocchio::pinocchio` |
| `dog2_mpc/CMakeLists.txt` | 同上 + `dog2_interfaces` |
| `dog2_wbc/CMakeLists.txt` | `pinocchio::pinocchio` |
| `dog2_mpc/package.xml` | `dog2_interfaces` 依赖 |

### 4.3 Bringup 与验证

| 文件 | 改动摘要 |
|------|----------|
| `dog2_bringup/launch/effort_research_sim.launch.py` | 暂停启动、控制器顺序、unpause |
| `dog2_bringup/launch/control_stack.launch.py` | mux/WBC/MPC 参数 |
| `dog2_bringup/dog2_bringup/wbc_effort_mux.py` | stand-up PD、12 通道输出、hold 逻辑 |
| `dog2_bringup/dog2_bringup/rail_lock_commander.py` | 发布全零导轨目标 |
| `dog2_bringup/dog2_bringup/smoke_check.py` | `wait_settle` 阶段 |
| `dog2_bringup/config/smoke_test.yaml` | 身高/前进/导轨门限 |
| `dog2_bringup/config/research_mpc.yaml` | 垂直支撑、平面闭环、assist 开关 |

### 4.4 控制算法

| 文件 | 改动摘要 |
|------|----------|
| `dog2_mpc/src/mpc_node_complete.cpp` | 平面闭环、垂直支撑、contact 订阅、姿态支撑范围 |
| `dog2_wbc/src/wbc_node_complete.cpp` | posture PD 仅 HOVER；腿名解析 |
| `dog2_gait_planner/.../swing_target_node.py` | `NOMINAL_FOOTS` 校正 |
| `dog2_motion_control/config/gait_params.yaml` | `standing_pose` 静态稳定角 |

---

## 5. 冒烟测试口径

### 5.1 命令

```bash
cd /home/dell/aperfect/carbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select \
  dog2_description dog2_dynamics dog2_mpc dog2_wbc \
  dog2_bringup dog2_gait_planner dog2_motion_control \
  --symlink-install

ros2 launch dog2_bringup smoke_test.launch.py \
  controller_mode:=effort \
  research_stack:=true \
  expect_research_stack:=true \
  ros_domain_id:=<fresh_id> \
  result_file:=/tmp/dog2_smoke_result.txt \
  2>&1 | tee /tmp/dog2_smoke_runN.log
```

### 5.2 阶段流程

```text
wait_ready  →  检查话题/节点/控制器
wait_settle →  等 startup hold+ramp 结束 + 2s 静止
stand       →  3s，XY 漂移、身高
forward     →  4s，vx=0.12，前进投影 ≥ 0.20 m
turn        →  5s，wz=0.35，yaw Δ ≥ 0.25 rad
全程        →  max_rail_drift ≤ 5 mm
```

### 5.3 历史结果摘要

| 运行 | 结果 | 备注 |
|------|------|------|
| run / run2 / run6 | **PASS** | 导轨锁 OK；行走质量差（横漂大） |
| run7–run14 | 混合 | 站立/启动序列迭代中 |
| **run15（最新）** | **FAIL** | `wait_settle` 40 s 超时；tilt ≈ 3 rad，机器人翻倒 |

**run15 失败特征**（`/tmp/dog2_smoke_run15.log`）：

- t ≈ 0–4 s：tilt ≈ 0，stand-up PD 正常
- t ≈ 5 s：pitch ≈ 0.86 rad，开始倾斜
- t ≈ 7 s 起：tilt > 2.5 rad，up_z < 0，**明确翻倒**
- MPC attitude 日志：`att_dz=[0,0,0,0,0]` — 翻倒期间姿态支撑未生效（mux 仍在 startup hold，MPC 未接管力控）

**当前 `/tmp/dog2_smoke_result.txt`**：

```text
FAIL: Robot never settled after startup (40 s): still moving or body height out of band.
```

---

## 6. 问题根因分析（当前阻塞）

### 6.1 已解决

| 问题 | 状态 |
|------|------|
| Pinocchio 库版本不匹配 | ✅ 已修复 |
| 导轨 position 锁不生效（JointForceCmd 冲突） | ✅ 已修复 |
| 静态不稳定 standing_pose | ✅ 已修复 |
| spawn 即 limp 塌地 | ⚠️ 部分缓解（暂停+PD，但不稳定） |
| 摆动目标 stomp/drag | ✅ 已修复 |
| 行走开环、无平面闭环 | ✅ 已实现 |
| trot 垂直力预算错误 | ✅ 已修复 |
| WBC posture 与行走对抗 | ✅ 已修复 |

### 6.2 未解决（当前重点）

**启动 stand-up PD 与静力前馈导致翻倒**

- 现象：run15 在 hold 阶段约 5 s 后 pitch 快速增大直至翻倒（tilt ≈ π）。
- 可能原因：
  1. **PD 增益与 ramp 仍过激**：`kp=[25,60,60]` + 4 s ramp 在 partially collapsed 初态下产生过大 femur/tibia 力矩，把躯干推过稳定域。
  2. **静力前馈与 PD 叠加方向错误**：前馈按站立姿态计算，但初始堆叠姿态不同，叠加后 net torque 可能加剧 pitch。
  3. **hold 期间 MPC/WBC 不施力**：翻倒时 `att_dz` 全零，无 MPC 姿态或 WBC 重力补偿兜底。
  4. **spawn_z=0.308 m 可能偏低**：相对新 standing_pose 的 min_foot_z，裕度仅 0.04 m，PD 起立过程中足端可能穿地或打滑。

**行走质量（在能稳定站立之前无法验证）**

- 早期 PASS 运行虽过门限，但 forward 阶段 **横向漂移 ~1 m >> 前进投影**，用户判定不合格。
- 平面闭环、contact 同步等改动 **尚未在稳定站立基线上回归验证**。

---

## 7. 关键参数速查

### 7.1 MPC（`research_mpc.yaml`）

```yaml
nominal_body_height: 0.26
vertical_support_kp: 250.0
vertical_support_kd: 90.0
vertical_support_max_leg_force: 95.0
flat_force_full_support: false
walking_stabilization_enabled: true
walking_vx_kp: 200.0
walking_vy_kp: 200.0
walking_wz_kp: 6.0
attitude_support_enabled: true
hover_force_max_leg_fz: 48.0
```

### 7.2 站立姿态（`gait_params.yaml`）

```yaml
standing_pose.*.hip_pitch_rad:  1.05
standing_pose.*.knee_pitch_rad: -1.10
standing_pose.*.rail_m: 0.0
```

### 7.3 Smoke 门限（`smoke_test.yaml`）

```yaml
body_height_min_m: 0.12
body_height_max_m: 0.40
forward_min_distance_m: 0.20
max_rail_drift_m: 0.005
stand_max_xy_drift_m: 0.20
```

---

## 8. 下一步建议（按优先级）

1. **稳定启动（阻塞项）**
   - 降低 stand-up PD 增益或延长 ramp（例如 kp 减半、ramp 6–8 s）。
   - 检查 hold 阶段是否应更早引入 MPC 垂直支撑或 WBC 重力补偿（而非纯 mux PD）。
   - 增大 `spawn_z_margin` 做 A/B（0.04 → 0.08 m）。
   - 在 Gazebo GUI 下目视 stand-up 全过程，确认是 pitch 失稳还是足端打滑。

2. **站立稳定后回归行走**
   - 重跑 smoke，对比 run6（旧 PASS）与新栈的 forward 投影 / 横漂 / yaw 比。
   - 微调 `walking_vx_kp`、`vertical_support_kp/kd`、步态 `stride_length` / `body_height`。

3. **工程卫生**
   - 每次 smoke 使用 fresh `ros_domain_id`。
   - 记录 `/tmp/dog2_smoke_runN.log` 与 `grep "MPC attitude\|tilt\|max_rail"` 摘要。
   - 重复性：同一 commit 至少跑 3 次 smoke。

4. **远期：越障**
   - 当前 `window_crossing_test` 仅 `PASS_STAGE_SMOKE` 稳定；`PASS_FULL` 未达成。
   - 开发边界：`CROSSING:BODY_FORWARD_SHIFT`。

---

## 9. 相关文档与日志

| 资源 | 路径 |
|------|------|
| 模块基线 | `docs/MODULE_RESEARCH_BASELINE.md` |
| 开发日志（07-08 导轨锁条目） | `DEVELOPMENT_LOG.md` |
| Bringup 架构 | `src/dog2_bringup/doc/ARCHITECTURE.md` |
| Gazebo 控制说明 | `src/dog2_description/GAZEBO_CONTROL_FIX.md` |
| 最新 smoke 日志 | `/tmp/dog2_smoke_run15.log` |
| 最新 smoke 结果 | `/tmp/dog2_smoke_result.txt` |
| 对话 transcript | `~/.cursor/projects/.../9549326d-8d07-4774-8bbf-6c95f61d0e2b.jsonl` |

---

## 10. 一句话总结

> ⚠️ 本节及 §6–§9 描述的是 2026-07-08 修复前的状态，已被 §11–§13 取代。
> **当前状态请看 §13（run33/34 起立→行走全流程 PASS 后的落成基线）。**

**（历史）** 导轨位置锁和 Pinocchio 迁移已打通，冒烟曾全绿但行走质量不合格；后续加入静态稳定站姿、启动 PD 起立、MPC 平面/垂直闭环与步态同步等一系列改进，逻辑上针对“前进不如漂移”的根因，但最新回归（run15）在启动 settle 阶段翻倒，当前阻塞点是 stand-up 序列稳定性，需先站稳再验证 WBC+MPC 行走流畅度。

**（2026-07-09 现状）** 经层 0→4 逐层根因排查（COM 漏躯干惯量、WBC 力符号反、常量力臂、单边高度环+分配不守恒、posture PD 与高度环对抗五个相互掩盖的根因全部修复），起立链路 run33/34 连续两次全流程 PASS；工作面转入行走质量。详见 §12（排查过程）与 §13（落成基线）。

---

## 11. 2026-07-08 下午复盘更新（run16 / run17，逐日志排查）

### 11.1 更正第 6.2 节的根因假设

run15 逐时间戳复盘推翻了“翻倒发生在 hold 阶段、MPC 未接管”的判断：

- stand-up PD 阶段机器人在**恢复**（tilt 0.86 → 0.41 rad）；
- 翻倒精确发生在 **mux hold→WBC ramp 交接完成的 1 秒内**（t≈65s ramp 结束，t≈66-67s tilt 0.64→2.07）；
- 交接瞬间 WBC 输出 coxa 四腿饱和 ±35、femur 符号翻转（+8 → −40 N·m）、单腿机械功率 −1900 W。

**真实根因链（前两项已修复并验证）：**

1. **hold 计时器被 smoke_check 的订阅提前触发**：mux 用 `get_subscription_count()` 判断 effort_controller
   就绪，但 smoke_check 也订阅 `/effort_controller/commands`，计时器比控制器激活早 2.6 s——pose ramp
   空转 66% 后力矩才接通（造成 lurch），且 6 s hold 窗被吃掉近半。
   **修复**：`wbc_effort_mux.py` 改为按订阅者节点名匹配（`output_controller_node: effort_controller`），
   并在计时器真正启动时重采 `_standup_initial_pose`。
2. **固定 6 s 定时交接，不看机器人状态**：run15 交接时 tilt 仍 ~0.5 rad 且在运动。
   **修复**：交接加 settle 门槛（odom tilt ≤ 0.15 rad + max|q̇| ≤ 0.6 rad/s 持续 1 s，上限 30 s 强制放行）。
   run16/run17 验证：**不再翻倒**，门槛正确拒绝危险交接。
3. **（待修）WBC 固定基座坐标系缺陷**：Pinocchio 模型无浮动基座，`LOCAL_WORLD_ALIGNED` 实际以 base_link
   为世界，MPC 的世界系 fz 在躯干俯仰 0.5 rad 时约有 sin(0.5)≈48% 分量变成水平推搡（~80 N），
   这是交接踹翻的动力学来源。
4. **（待修）HOVER 无姿态兜底**：attitude support 仅 WALKING 生效，hover sanitizer 还把 fx/fy 清零，
   起立/settle 阶段 MPC 层面对躯干倾斜零反馈（全程 att_dz=[0,0,0,0] 的原因）。

### 11.2 排除的两个假警报（重要，避免复查绕路）

- **MPC mass=12.00 kg 是对的**：躯干 6.0 kg 在固定基座模型中挂在 universe 惯量上，
  `pin.computeTotalMass()` 会跳过它只算四腿 6.0 kg。`Dog2Model::mass()`（遍历含 universe 的
  inertias）= 12.0028 kg 正确。HOVER fz≈123 N vs 实际重量 117.7 N，只超 5%。
- **startup 静力前馈方向没有错**：按 12 kg + 42/58 分配用 J^T f + g(q) 重算，与现有配置
  （femur +1.75/+2.04，tibia −4.57/−6.10）方向一致、量级吻合。run15 日志里 tibia +23.6 N·m
  是 posture PD 分量，不能当静力项比对。

### 11.3 当前唯一阻塞项：起立俯仰跑飞 → “搁浅”

run16（修复 1+2 后）：不翻倒，但起立后段俯仰跑飞（t≈67→70s：0.16→0.29→0.60 rad），最终停在
tilt 0.601 rad 的稳定斜姿；门槛正确地永不放行，forward 阶段 0 位移 FAIL。

机理：中间姿态的实际重力矩远大于按站姿算的恒定前馈，欠补偿侧下沉 → COM 向低侧滑移 →
负载更偏 → 正反馈跑飞 → 躯干边缘触地“搁浅”。

**run17 A/B（关键否定证据）**：给 stand-up PD 加俯仰差分调平（低侧伸展 / 高侧折叠，kp 0.5，
限幅 0.3 rad）。关节全部按修正目标到位（后腿 tibia −1.62→−1.14），但躯干俯仰 0.601→0.658
反而略差——搁浅后关节空间怎么调都改变不了姿态。**结论：关节空间 PD + 静态前馈在架构上压不住
这个不稳定，调 kp/ramp/差分都是边际打转（也解释了历史上“kp 45 塌、kp 90 翻”的调参循环）。**
调平机制保留在代码中但默认关闭（`startup_standup_level_enabled: false`）。

### 11.4 下一步（结构性修复，同时也是行走质量的根因）

1. **WBC 力坐标系**：订阅 odom 姿态，把 MPC 世界系足端力旋回 base 系再做 J^T
   （或引入浮动基座模型）；重力补偿同理。
2. **HOVER 姿态支撑**：`isSupportStabilizationActive` 放开 HOVER（可用低增益档），
   hover sanitizer 不再一刀切清零 fx/fy。
3. **提前交接**：上述两项就位后，放宽/提前 settle 门槛，让姿态闭环的 WBC+MPC 完成起立后半程，
   而不是让盲的关节 PD 一路扛到站直。
4. 站稳后回归行走质量（fix 1/2 的 smoke 基线：run16/run17 均到达 stand 阶段，z=0.225/0.286，
   无翻倒）。

### 11.4b run19 里程碑（结构修复三件套首次联测，2026-07-08 傍晚）

**层 0 补充修复**：`Dog2Model::centerOfMass` 漏躯干惯量（COM x +0.108 → 真值 +0.054，
前/后负载 42/58 → 真值 61.6/38.4，方向反了）；起立前馈按蹲姿几何重算。

**结构修复**：① WBC 力旋回 base 系 + 姿态感知重力补偿（`rotate_forces_to_base`）；
② MPC HOVER 姿态支撑（scale 0.5）；③ 起立目标降为蹲姿 [0, 1.05, −1.55]（身高 0.166 m）。

**run19 结果（对比 run16-18 搁浅 0.60-0.66 rad / run18b 强交接后满地乱窜 5 m）**：

- 蹲姿起立成功：静止在 **tilt 0.197 rad**（差门槛 0.15 一点，未自动交接，36 s 强制）
- HOVER 姿态支撑首次实际出力：att_dz=[−12.4,+12.4,+12.4,−12.4]，方向正确（抬尾侧）
- 强制交接后**不再翻倒、不再乱窜**（tilt 0.16~0.51 波动，up_z≥0.87），但起立到站高的
  闭环仍震荡未收敛 → smoke settle 40 s 超时 FAIL
- **失败模式已从"结构性缺陷"降级为"闭环调参"**：剩余工作是交接门槛（0.15→~0.22）、
  WBC posture PD 与 MPC 高度环的节奏配合

**大范围清理（同日）**：删除 28 个孤儿/白名单遗留 launch、4 个死 C++ 可执行
（mpc_node / mpc_node_16d / state_simulator / wbc_node_simple）、dog2_kinematics 死包；
`check_symmetric_usage.py` 豁免白名单退役。活跃链唯一化：
smoke/window_crossing → system → sim_base(effort_research_sim | spider_gazebo_*) +
control_stack(estimator/gait/mpc_complete/wbc_complete/mux) + visualization。

### 11.4c 交接后闭环调参轨迹（run20-22）

| run | 改动 | 结果 |
|-----|------|------|
| 20 | 门槛 0.15→0.22（交接自动化，hold 7 s） | 交接成功；~1 Hz 摆动发散，10 s 后翻倒 |
| 21 | ramp 2→5 s；posture kp 60→45/max 28；attitude scale 0.5→0.35、kd 上调；height kd 90→140 | 不翻倒；但高度环抢 0.26 目标，身高 0.08-0.15 弹跳 |
| 22 | **一阶段站高 0.20**（交接高度 0.166 + 3.4 cm，几乎无阶跃）；posture 目标改 -1.40（与 0.20 一致）、kp 20/max 18（降为零空间整形器） | **不摔、不发散**，tilt 有界 0.03-0.33 极限环，未达 settle 静止度 |

方向已收敛：三环（posture PD / MPC 高度 / MPC 姿态）目标一致化 + 准静态交接是对的。

**run23-26 A/B 矩阵（全部有界不摔，但 settle 未过）：**

| run | 变量 | 观察 |
|-----|------|------|
| 23 | stance_joint_pose/height 全对齐 0.20 站姿 | 极限环仍在（0.02-0.41） |
| 24 | 关 HOVER attitude support（A/B） | 仍振荡 0.03-0.55 → **姿态支撑不是极限环唯一源** |
| 25 | 前馈换回站姿版（对照蹲姿版） | hold 静止 h=0.091/tilt=0.238（比蹲姿版 0.083 略好但同数量级）；ramp 期 tilt 0.238→0.075 说明力栈能拉平 |
| 26 | 门槛 0.26 提前交接 | 交接自动（7 s）；交接后 h 在 0.05-0.17 弹跳不收敛到 0.20 |

**关键新发现**：PD hold 从未真正到达设计蹲姿（应 h=0.166/tilt≈0，实际 h≈0.09/tilt≈0.24，
两种前馈皆然）——起立在更深的姿态就停滞了；交接后力栈保持安全但不收敛。

**下一步两个具体方向（非调参）：**
1. **MPC 垂直力分配用实时 FK 足端**：`applyVerticalSupport` 的力矩配平用的是常量
   `base_foot_positions_`（站姿几何），深蹲实际足端偏离甚远 → 系统性倾覆力矩，
   与观察到的持续 wobble 吻合。MPC 已有 joint states，可改为实时 FK（~20 行）。
2. **可视化诊断**：Gazebo GUI / rosbag 绘图看清 hold 停滞与交接后弹跳的真实形态
   （坐姿？足底打滑？蹦跳？），替代 1 Hz 文本日志盲调。

### 11.5 本轮代码改动清单

| 文件 | 改动 |
|------|------|
| `dog2_bringup/dog2_bringup/wbc_effort_mux.py` | 控制器订阅者识别、settle 门槛交接、初始姿态重采、俯仰调平机制（默认关） |
| `dog2_bringup/launch/control_stack.launch.py` | 新增 handoff/level 参数块 |
| smoke 日志 | `/tmp/dog2_smoke_run16.log`、`/tmp/dog2_smoke_run17.log` |

---

## 12. 2026-07-09 根因大扫除（run27-32：settle 首次 PASS）

从 URDF 层 0 开始逐层对账（Pinocchio 独立核对总质量 12.003 kg、
站姿 COM x=+0.054/前 61.6%、蹲姿前 24.1% 等真值全部确认），随后在
层 2-4 挖出**五个相互掩盖的根因**，全部修复后 settle 门槛 12.1 s 通过
（run15-31 从未通过）。

### 12.1 五个根因（按发现顺序）

1. **MPC 支撑力臂用常量站姿几何**（`applyVerticalSupport` /
   `applyAttitudeSupport` / walking yaw 分配）。深蹲时真实足端偏离站姿
   >10 cm，tilt 0.24 时 base 系 z 偏移还会泄漏 sin(tilt)×0.15≈0.036 m 进
   水平力臂。→ 新增 `updateSupportGeometry()`：实测关节 FK 足端 −
   实测 COM，再旋到世界系（`support_lever_arms_`），四处消费者全部换用。

2. **mux 起立常量前馈只在单一姿态精确**：站姿版在中间深蹲欠补偿，
   蹲姿版是折叠偏置，run19-27 两种都停滞在 h≈0.09（tibia −1.87，比设计
   蹲姿深 0.3 rad，胫骨蹭地）。→ mux 内新增 `QuasiStaticStandupFF`：
   逐周期在测量位形上算 τ_ff(q) = g(q) − Σ J_iᵀ R_wbᵀ [0,0,f_z,i(q)]，
   f_z,i 与 MPC 同款力矩配平。与 `pin.rnea(q,0,0,fext)` 交叉验证误差
   3.6e-15。**效果立竿见影：hold 从 h=0.09/tilt=0.24 停滞变为
   h=0.184/tilt=0.049 静止，7 s 过交接门槛。**

3. **WBC `foot_force_sign` 默认 +1 是反的**（历史最大坑）。MPC 发布的
   fz 是向上的地面反力，静力学正确公式 τ = g − Jᵀf_ext，即 sign 必须
   −1。符号反着时支撑腿全部收到折叠力矩：交接后 tibia slam 到 −2.8
   限位、"h 0.05-0.17 弹跳永不收敛"、行走"身高 0.05-0.25 波动"全是它。
   昨日 11.2 节"前馈方向没错"的复核是循环论证（用同一带符号错的公式
   重算）。→ `wbc_foot_force_sign` 默认改 −1.0（control_stack +
   system 两处）。**run24"关姿态支撑"的 A/B 结论随符号修复全部作废。**

4. **高度环单边 + 分配器不守恒（两个叠加的力学缺陷）**：
   - `applyVerticalSupport` 旧公式 mg + kp·max(0,err) + kd·max(0,−vz)
     只会加力：身高超目标上升时输出恰好 = mg 抵消重力，躯干匀速上漂到
     脚离地 → 摔落 → 接触振铃 → 又被弹起（run28/29 跳跃循环）。
     → 改对称双边 mg + kp·err − kd·vz，multiplier 下限从代码强制 1.0
     放开为 0.0（yaml 配 0.6 保脚不完全卸载）。
   - 最小偏差分配只解两个力矩乘子，**总力从未被约束**：Σfz = total +
     λ1·sx + λ2·sy，而本机 COM 从不在足端形心（站姿 sx≈−0.17，深蹲
     −0.29），分配器从诞生起就少交付 20-40% 体重（run27 hold 要 145 N
     只发 97.9；run31 关掉 posture PD 后彻底暴露：沉到肚皮贴地，需求
     132 N 只交付 77.7）。→ 改三乘子 KKT 精确解（总力+两力矩），钳位后
     再守恒重缩放。

5. **WBC posture PD 与高度环双重调节同一自由度**：四足着地 + 导轨锁 =
   16 自由度被 12 接触约束 + 4 导轨锁完全确定，关节角由身体位姿唯一
   决定；关节空间 posture PD（目标 −1.40 ↔ h=0.205）与任务空间高度环
   （0.20）必然内力对抗、足底蹭地（老注释"蹭地拉锯"预警成真）。run30
   实测：hold 在 h=0.184 完美静止，交接瞬间 posture PD 的 +4.4 Nm/腿把
   躯干踢高 5 cm 引发跳跃循环。→ `posture_pd_enabled: False`（注释：
   没有零空间投影前禁止再开）。

辅助修复：足底接触 kd 20→300（13 Hz 接触共振 ζ 0.04→0.55，20 Hz 控制
环的 Nyquist 只有 10 Hz，振铃必然混叠进 vz 反馈）；高度环增益
kp 250→120 / kd 140→60（符号修复后旧增益等效放大数倍）；
hover sanitizer per-leg [12,64]→[4,72]；HOVER 姿态支撑重新打开
（run24 关它的依据已随符号修复作废）。

### 12.2 run27-32 结果轨迹

| run | 关键变化 | 结果 |
|-----|---------|------|
| 27 | MPC 实时力臂 | 起立仍停滞 h=0.09/tilt=0.237（暴露前馈问题）；交接后 tibia slam −2.8 |
| 28 | + mux 在线前馈 + WBC sign=−1 | **起立治愈**（h=0.184/tilt=0.049，7 s 交接）；交接后跳跃 0.09-0.33 |
| 29 | + 降高度环增益/开姿态支撑 | 仍跳（单边高度环+分配缺口还在） |
| 30 | + 接触阻尼/对称高度环准备 | hold 完美静止；交接瞬间被 posture PD 踢飞 |
| 31 | + 关 posture PD | 不跳了但沉到肚皮 h=0.05（分配缺口全暴露） |
| 32 | + 三乘子守恒分配 | **settle 12.1 s PASS、stand PASS（漂移 3 mm、z=0.194 稳态）**；forward/turn 是下一战场 |

### 12.3 run33：全流程 PASS（2026-07-09 17:43）

`NOMINAL_FOOTS` 对齐 0.20 站高几何后（原值按 h=0.268 站姿，每次落足往
地里跺 6-7 cm，正是 run32 横冲直撞的主因）：

```text
Robot settled after startup (12.1 s)
Stand   PASS  drift=0.003 m  z=0.194 m
Forward PASS  projected=0.560 m (run32: 0.021 m)  planar=1.043 m
Turn    PASS  yaw_delta=3.101 rad  z=0.250 m
PASS: max_rail_lock_err=0.0019 m
```

**这是该项目第一次 settle→stand→forward→turn 全绿。**

run34（同代码重复验证）：再次 PASS——settle 12.2 s、stand 漂移
0.004 m、forward 投影 0.552 m（z=0.198 全程健康）、turn 2.806 rad、
rail 0.0015 m。两连绿，起立链路确认稳定。

剩余质量问题（下一阶段）：forward 期横向漂移仍明显
（projected/planar = 54%）、forward 末段身高 0.138 偏低、
所有行走增益是在 WBC 符号反转年代调的需重新基线、
同 commit 需重复 3 次验证稳定性。

### 12.4 本轮代码改动清单

| 文件 | 改动 |
|------|------|
| `dog2_mpc/src/mpc_node_complete.cpp` | `updateSupportGeometry()` 实时 FK 力臂；对称高度环；三乘子守恒分配；multiplier 下限放开 |
| `dog2_bringup/dog2_bringup/wbc_effort_mux.py` | `QuasiStaticStandupFF` 在线准静态前馈（RNEA 交叉验证）；odom R_wb；rail 位置订阅 |
| `dog2_bringup/launch/control_stack.launch.py` | 在线前馈参数 + robot_description 注入 mux；`wbc_foot_force_sign` −1.0；`posture_pd_enabled` False |
| `dog2_bringup/launch/system.launch.py` | `wbc_foot_force_sign` 默认 −1.0 |
| `dog2_bringup/config/research_mpc.yaml` | 高度环 kp/kd、multiplier 0.6、sanitizer [4,72]、HOVER 姿态支撑开 |
| `dog2_description/urdf/dog2_symmetric_properties.xacro` | 足底接触 kd 20→300 |
| `dog2_gait_planner/dog2_gait_planner/swing_target_node.py` | `NOMINAL_FOOTS` 对齐 0.20 站高几何 |
| 验证脚本 | `/tmp/dog2_layer0_verify.py`（层0 真值）、`/tmp/test_online_ff.py`（前馈交叉验证） |

---

## 13. 落成状态基线（2026-07-09 全流程 PASS 后 · 详细）

> 本节是 run33/34 全流程 PASS 之后的**当前系统 as-built 参照**：约定真值、活跃运行链、
> 起立时序、现行参数、复现手册、里程碑总表、未提交状态、下一阶段计划、已知债务。
> §1–§9 是修复前的历史描述，§11–§12 是逐日排查过程；**遇到"现在是什么状态/怎么跑/
> 参数是多少"一律以本节为准。** 本节的"现行参数速查"取代 §7。

### 13.1 现状一句话

起立链路（趴姿 → 蹲姿 PD → 交接 → WBC+MPC 力栈 → 一阶段站高 0.20 m）已打通并**连续两次
（run33/34）settle→stand→forward→turn 全绿**；导轨锁漂移 ≤ 2 mm。当前工作面从"能不能站起来"
转入"行走质量"（横漂、增益重基线）。全部修复代码**尚未提交**。

### 13.2 约定基线（框架真值，改任何一处都要三方一致）

| 维度 | 真值 | 说明 / 陷阱 |
|------|------|------------|
| 腿序（规范） | `[lf, lh, rh, rf]` = 索引 0/1/2/3，逆时针 | effort_controller 12 通道、rail 4 通道、mux `_LEG_NAMES`、WBC `kLegPrefixes`、MPC `kMpcFootFrames`、状态估计/步态 `LEG_ORDER` 全部按此 |
| 腿内关节 | `rail`(P) → `coxa` → `femur` → `tibia`(R) | 16 通道旧布局每腿 [rail,coxa,femur,tibia] 串联 |
| 机头朝向 | **−X**（用户确认） | 站姿前足 x≈−0.052（头侧），后足 x≈+0.224（尾侧） |
| ⚠️ `Dog2Model::FOOT_NAMES` | `[rh, rf, lf, lh]`（**与规范序不同！**） | 生产路径不用它；`test_mpc_urdf_model_data.cpp` 有守卫防止误改成 lf 序 |
| `/joint_states` 顺序 | Gazebo **乱序** | 所有消费者必须按名字解析，禁止按下标 |
| 总质量 | 12.0028 kg = 躯干 6.0004 + 四腿 6.0024；重 117.75 N | **腿占全重 50%**（一般四足 10–20%），SRBD"腿无质量"假设对本机偏弱 |
| 站姿 COM(base) | x=**+0.054**；前腿承 **61.6%** / 后 38.4% | ⚠️ `pin.centerOfMass` 漏躯干（挂 universe 惯量）→ 曾误得 x=+0.108、42/58（方向反）。已在 `Dog2Model::centerOfMass` 手工合并躯干惯量修复 |
| 力符号 | **`foot_force_sign = −1.0`**；τ = g(q) − Jᵀ f_ext | MPC 发的 fz 是向上地面反力，静力学要求 −1；默认 +1 是历史最大坑（run27 前一直反着） |
| 导轨限位（各腿不同） | lf[0,0.111] lh[−0.111,0] rh[0,0.111] rf[−0.111,0] | 不可写统一常数 |

### 13.3 活跃运行链（唯一，as-built）

```text
smoke_test.launch.py / window_crossing_test.launch.py → crossing_trial.launch.py
└─ system.launch.py（wbc_foot_force_sign 默认 −1.0）
   ├─ sim_base.launch.py
   │   └─[controller_mode:=effort research_stack:=true] effort_research_sim.launch.py
   │        Gazebo(暂停机制走 hold_joints) + spawn(蹲姿关节角, z=0.308)
   │        + joint_state_broadcaster + rail_position_controller(位置锁)
   │        + effort_controller(12 旋转关节) + bridges + gz_pose_to_odom
   │        + robot_description_server + rail_lock_commander(发 [0,0,0,0])
   │   └─[controller_mode:=position] spider_gazebo_* (motion_control 旧栈，保留)
   ├─ control_stack.launch.py
   │   ├─ state_estimation.launch.py → sim_state_estimator_node
   │   ├─ gait_scheduler.launch.py → gait_scheduler_node + swing_target_node
   │   ├─ mpc_node_complete（C++，唯一活跃 MPC）
   │   ├─ wbc_node_complete（C++，唯一活跃 WBC）
   │   ├─ mpc_debug_adapter + wbc_debug_adapter
   │   └─ wbc_effort_mux（起立状态机 + 12 通道拼装）
   └─ visualization.launch.py（rviz 可选）
```

清理后死代码已删（§11.4b/§12）：28 个孤儿 launch、`mpc_node`/`mpc_node_16d`/`state_simulator`/
`wbc_node_simple`、`dog2_kinematics` 包；`check_symmetric_usage.py` 豁免白名单退役。

### 13.4 起立状态机时序（wbc_effort_mux，as-built）

```text
[1] 等 /joint_states → 等 effort_controller 真正订阅 /effort_controller/commands
    （按订阅者节点名匹配 output_controller_node=effort_controller，
     不再被 smoke_check 的订阅骗启动；启动瞬间重采 _standup_initial_pose）
[2] hold + 在线准静态前馈起立：目标蹲姿 [0,1.05,−1.55]，pose ramp 4 s
    每周期 τ = kp·(q_ramp−q) − kd·q̇ + τ_ff(q)，
    τ_ff 逐周期在测量位形上算 g(q) − Σ Jᵢᵀ R_wbᵀ[0,0,f_z,i(q)]（RNEA 交叉验证 3.6e-15）
[3] settle 门槛交接：tilt ≤ 0.26 rad 且 max|q̇| ≤ 0.6 rad/s 持续 1 s（上限 30 s 强制）
[4] ramp 5 s 权限渐入 → 纯 WBC 力栈输出
[5] WBC+MPC 力栈把身高从 ~0.166 拉到一阶段站高 0.20，姿态由 MPC 支撑闭环拉平
```

关键：起立后半程交给**姿态感知的力栈**（WBC 力旋回 base 系 + MPC 高度/姿态闭环），
而非让关节盲 PD 一路扛到站直（后者会搁浅在 0.6 rad 斜姿，见 §11.3）。

### 13.5 现行关键参数速查（取代 §7）

MPC `research_mpc.yaml`：
```yaml
nominal_body_height: 0.20            # 一阶段站高（0.26 全站姿是后续独立调参）
stance_joint_pose: [0.0,0.0,1.05,-1.40]   # 力臂/分配/姿态目标同一位形
vertical_support_kp: 120.0           # 符号修复后从 250 降（旧增益等效放大数倍）
vertical_support_kd: 60.0
vertical_support_min_total_force_multiplier: 0.6   # 双边高度环安全下限（1.0=单边会漂飞）
vertical_support_max_leg_force: 95.0
hover_force_max_leg_fz: 72.0 / min: 4.0
attitude_support_enabled: true
attitude_support_hover_enabled: true / hover_scale: 0.35
attitude_support_pitch_kp: 140 / kd: 28 ; roll_kp: 90 / kd: 20 ; max_leg_delta: 18
walking_vx_kp: 100 / vy_kp: 80 / wz_kp: 12   # 符号修复后重基线（原 200/200/6）
```

WBC / mux（`control_stack.launch.py`）：
```text
wbc_foot_force_sign: -1.0            # ← 最关键；system.launch.py 默认也已 -1.0
rotate_forces_to_base: True          # 力旋回 base 系 + 姿态感知重力补偿
posture_pd_enabled: False            # 与高度环重复调节同一自由度，暂禁（待零空间投影再开）
startup_online_ff_enabled: True      # 在线准静态前馈
startup_standup_pose: [0.0,1.05,-1.55]（蹲姿）
startup_ramp_sec: 5.0 ; startup_handoff_tilt_max_rad: 0.26
```

URDF：`dog2_symmetric_properties.xacro` 足底接触 kd 20→**300**（13 Hz 接触共振 ζ 0.04→0.55，
避免振铃混叠进 20 Hz 控制环）。

### 13.6 复现手册（PASS）

```bash
cd /home/dell/aperfect/carbot_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select \
  dog2_dynamics dog2_mpc dog2_wbc dog2_bringup dog2_gait_planner \
  dog2_motion_control dog2_description --symlink-install
source install/setup.bash

ros2 launch dog2_bringup smoke_test.launch.py \
  controller_mode:=effort research_stack:=true expect_research_stack:=true \
  ros_domain_id:=<每次换新号> \
  result_file:=/tmp/dog2_smoke_result.txt 2>&1 | tee /tmp/dog2_smoke_runN.log
```

期望（run33/34）：
```text
Robot settled after startup (~12 s)
Stand   PASS  drift≈0.003 m  z≈0.194 m
Forward PASS  projected≈0.55 m  planar≈1.0 m
Turn    PASS  yaw_delta≈3.0 rad
PASS: max_rail_lock_err≈0.0019 m
```
注意：清理删了 build/install，首次必须全量 `colcon build`；每次 smoke 用**全新
`ros_domain_id`**（避免残留节点串扰）。

### 13.7 里程碑总表（run15 → run34）

| run | 阶段 | 结果 |
|-----|------|------|
| 15 | 定时盲交接 | 交接瞬间翻倒（tilt≈π） |
| 16–17 | 计时器就绪判断 + settle 门槛 | 不再翻；搁浅 0.60 rad 斜姿 |
| 18–18b | 层0 COM 修复 + 前馈换向 | 到 stand 阶段；强交接后满地乱窜 |
| 19 | 结构三件套联测 | 蹲姿起立、姿态支撑首次出力；未收敛 |
| 20–26 | 交接/增益/几何一致化调参 | 有界极限环，settle 均未过 |
| 27 | MPC 实时力臂 | 暴露前馈停滞 h≈0.09 |
| 28 | 在线前馈 + **WBC sign=−1** | 起立治愈 h=0.184；交接后跳 |
| 29–31 | 高度环/接触阻尼/关 posture PD | 逐个暴露分配缺口 |
| 32 | 三乘子守恒分配 | **settle+stand 首次 PASS** |
| 33 | `NOMINAL_FOOTS` 对齐 0.20 | **全流程首次 PASS** |
| 34 | 同代码复验 | **再次 PASS**（两连绿） |

### 13.8 未提交状态与提交建议

`git status`：**59 改 / 35 删 / 32 新增**，全部未提交（3 个清理 commit 已落：
`9947574`/`4588ec3`/`747e229`）。风险：一整天的根因修复 + PASS 都在工作区里没进版本库。
建议分主题提交（顺序）：

1. `fix(dynamics): centerOfMass 合并躯干惯量` — 层0 真值
2. `fix(wbc): 力旋回 base 系 + 重力姿态补偿 + foot_force_sign=-1` — 力学根因
3. `fix(mpc): 实时 FK 力臂 + 对称高度环 + 三乘子守恒分配 + HOVER 姿态支撑`
4. `feat(bringup): mux 在线准静态前馈 + settle 门槛交接 + 起立状态机`
5. `chore: 删除 35 个死 launch/可执行/dog2_kinematics`（已暂存）
6. `tune(gait/urdf): NOMINAL_FOOTS 对齐 0.20 + 接触 kd 300 + 行走增益重基线`

### 13.9 下一阶段：行走质量（当前工作面）

1. **横向漂移**：forward 期 projected/planar≈54%，横漂仍显著 → 复看 `applyWalkingPlanarStabilization`
   的 vy/yaw 闭环（vx100/vy80/wz12 是符号修复后初调，未系统整定）
2. **forward 末段身高偏低**（0.138）→ 行走时高度环/垂直分配需复核
3. **增益全面重基线**：所有行走增益都是 WBC 符号反转年代调的，需在正确力学下重调
4. **可重复性**：同 commit 至少跑 3 次 smoke（目前 2 连绿）
5. **二阶段站高**：0.20 → 0.26 全站姿的独立抬升（stance_joint_pose/posture/height 三者同步）

### 13.10 已知债务 / 陷阱清单（防止重新踩坑）

- **SRBD 假设腿无质量**，但本机腿占 50% 全重 → 行走调不动时优先怀疑此处
- **posture PD 已禁用**：没有零空间投影前重新打开必然和高度环内力对抗（run30 踢飞）
- **`pin.centerOfMass`/`computeTotalMass` 跳过 universe 躯干惯量** → 任何新算 COM/质量的脚本都要手工合并
- **`FOOT_NAMES` 是 [rh,rf,lf,lh]**，与规范腿序不同 → 新代码勿直接套用
- **力符号 −1 是静力学真值**，勿被历史注释/循环论证带偏（§11.2 曾误判）
- 行走增益、attitude/height 增益均为**符号修复前所调**，参考价值有限
- 清理后 build/install 已删，换机/重开必须全量 build
