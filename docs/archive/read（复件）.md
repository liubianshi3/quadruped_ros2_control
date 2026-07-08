## 先说明审阅边界

我没有成功读取到你 fork 里的完整源码：公开 GitHub 页面和 raw 文件打开都失败，`mpc_node_complete.cpp / trajectory_generator.cpp / crossing_state_machine.cpp` 这几个 raw 路径也没有取到内容。
所以下面不是“逐行 diff 级审阅”，而是基于你给出的**运行结果、文件职责、阶段名、诊断结论和控制链结构**做的专家级架构审阅。结论仍然足够用于下一轮开发。

---

# 一句话技术判断

当前不应该继续只调 `attitude_support_*` 参数。真正要改的是：**把 crossing 初段从“walking reference + 被动姿态补偿”改成“准静态 contact sequence + 稳定性门控 + 力矩重分配”。**

换句话说：

> 现在系统能到 `BODY_FORWARD_SHIFT`，说明 rail / WBC / femur-tibia 力矩不是主障碍；卡住后续阶段的根因更像是 **PRE_APPROACH / APPROACH 参考过激 + 姿态误差处理不鲁棒 + 状态机缺少稳定性条件**。🧩

---

# 1. 根因排序

## P0：`roll ≈ 3.14 rad` 必须先判定是真翻车还是姿态表示问题

`roll` 峰值到 `3.14 rad` 非常危险。这里有两种可能：

| 可能性                          | 含义      | 后果                                                  |
| ---------------------------- | ------- | --------------------------------------------------- |
| 真翻滚 / 机身接近倒扣                 | 物理稳定性已崩 | 后续任何状态机调参都没意义                                       |
| Euler roll wrap / frame 变换异常 | 姿态表示跳变  | `applyAttitudeSupport()` 可能被错误 roll error 激励，反而打爆姿态 |

我会把这条列为最高优先级。因为四足越障控制里，`roll = π` 不一定总是“物理翻车”；如果来自 Euler 角解算，可能是四元数到 RPY 的分支切换，尤其在 yaw/pitch 扰动大时更容易出现跳变。

### 修改建议

在 `mpc_node_complete.cpp` 或状态估计链路中，不要直接用 Euler `roll/pitch` 作为姿态控制误差的唯一来源。改成：

```cpp
// R_des: desired body rotation
// R: measured body rotation
Eigen::Matrix3d R_err = R_des.transpose() * R;
Eigen::Vector3d e_R;
e_R << R_err(2,1) - R_err(1,2),
       R_err(0,2) - R_err(2,0),
       R_err(1,0) - R_err(0,1);
e_R *= 0.5;
```

然后只用 `e_R.x()` 和 `e_R.y()` 做 roll/pitch 稳定，yaw 单独锁定。

同时加日志：

```text
[att_dbg]
stage=
roll_rpy=
pitch_rpy=
yaw_rpy=
quat_norm=
gravity_body=
eR_x=
eR_y=
omega_body=
body_z=
```

目标是确认：`roll≈3.14` 时，`gravity_body.z` 是否真的接近 `-1`。如果不是，那就是姿态表示/坐标系问题，不应继续按“真实大 roll”调力。

---

# 2. 文件级审阅与修改方向

## 2.1 `trajectory_generator.cpp`：最可疑的问题源

你已经指出“可能还保留了 walking 化的初段推进参考”。这个判断很合理。

### 当前问题判断

`PRE_APPROACH / APPROACH` 不应该像普通平地 gait 那样持续给前进速度。越窗前段更像“接触规划问题”，不是普通行走问题。

现在的现象是：

```text
能进 crossing
能到 BODY_FORWARD_SHIFT
但后续腿跨越阶段进不去
roll 初段扰动极大
```

这通常说明：

1. 初段 body reference 推得太快；
2. 横向 / yaw 没被强锁；
3. 机身 COM 没有被放进足端支撑多边形的安全区域；
4. 状态机按 `x / time / stage` 推进，而不是按稳定性推进；
5. 前后腿 transit 前，机体姿态已经不满足可跨腿条件。

### 应改成准静态参考

建议把 crossing 初段改成这种节奏：

```text
PRE_APPROACH:
  hold settle
  very slow body forward
  hold settle again

APPROACH:
  front feet staging
  body remains level
  no aggressive gait

BODY_FORWARD_SHIFT:
  body COM shift forward inside support polygon
  wait until attitude stable

FRONT_LEGS_TRANSIT:
  one front leg at a time or synchronized only if support margin足够

REAR_LEGS_TRANSIT:
  rear legs follow

RECOVERY:
  return to normal gait
```

### 关键参数建议

先保守，不追求速度。

```yaml
crossing:
  quasi_static_enabled: true

  pre_approach:
    vx_max: 0.025          # m/s
    ax_max: 0.04           # m/s^2
    jerk_max: 0.10         # m/s^3
    settle_time: 0.60      # s
    yaw_lock_kp: 1.0
    lateral_lock_kp: 1.0
    max_roll_for_progress: 0.25
    max_pitch_for_progress: 0.25

  approach:
    vx_max: 0.020
    ax_max: 0.035
    settle_time: 0.80
    body_z_ref: 0.155
    y_ref: 0.0
    yaw_ref: 0.0

  body_forward_shift:
    vx_max: 0.015
    ax_max: 0.025
    required_stable_time: 0.50
    support_margin_min: 0.025
    foot_slip_max: 0.025
```

重点不是数值本身，而是控制哲学：

> crossing 不是“走过去”，而是“停—移—稳—跨—稳—跨—恢复”。

---

## 2.2 `crossing_state_machine.cpp`：状态推进条件太可能偏弱

现在状态机能到：

```text
CROSSING:BODY_FORWARD_SHIFT
```

但没有稳定进入：

```text
FRONT_LEGS_TRANSIT
REAR_LEGS_TRANSIT
RECOVERY
```

这说明状态机推进条件可能只看了：

* x 位置；
* 时间；
* rail delta；
* crossing mode；
* 简单 stage flag。

但它应该看的是：

* 姿态是否稳定；
* 角速度是否收敛；
* 支撑足是否可信；
* foot slip 是否小；
* body COM 是否还在支撑多边形内；
* WBC 是否有足够 force / torque margin。

### 建议新增 `StageStabilityMetrics`

建议在 MPC 或 state machine 层定义：

```cpp
struct StageStabilityMetrics {
  double roll_abs;
  double pitch_abs;
  double yaw_rate_abs;
  double body_z;
  double body_vx;
  double body_vy;
  double body_wx;
  double body_wy;
  double support_margin;
  double max_foot_slip;
  double min_fz_margin;
  double max_rail_delta;
  bool all_required_contacts_valid;
};
```

然后加一个稳定性判定：

```cpp
bool isStableForTransit(const StageStabilityMetrics& m) {
  return m.roll_abs < params.max_roll_for_transit &&
         m.pitch_abs < params.max_pitch_for_transit &&
         m.yaw_rate_abs < params.max_yaw_rate_for_transit &&
         m.support_margin > params.support_margin_min &&
         m.max_foot_slip < params.foot_slip_max &&
         m.min_fz_margin > params.fz_margin_min &&
         m.all_required_contacts_valid;
}
```

再加持续时间门控：

```cpp
if (isStableForTransit(metrics)) {
  stable_timer_ += dt;
} else {
  stable_timer_ = 0.0;
}

if (stable_timer_ > params.required_stable_time) {
  advanceStage();
}
```

### 不要直接推进到腿跨越

从 `BODY_FORWARD_SHIFT` 进入 `FRONT_LEGS_TRANSIT` 前，至少要满足：

```text
abs(roll) < 0.25~0.35 rad
abs(pitch) < 0.25~0.35 rad
abs(yaw_rate) < 0.3~0.5 rad/s
body_z in valid range
support_margin > 2~3 cm
foot_slip < 2~3 cm/s
持续 0.4~0.8 s
```

否则前腿一抬，支撑多边形立刻变小，姿态会更差。

---

## 2.3 `mpc_node_complete.cpp`：`applyAttitudeSupport()` 应从“补 fz”升级成“受限力矩重分配”

你现在已经改了 `vertical_support` 给 `fz` headroom。这是对的，但还不够。

### 当前补偿的潜在问题

如果姿态补偿只是简单增加某些腿的 `fz`，或者简单降低 `vertical_support` 留余量，会有两个风险：

1. **总竖直力被破坏**
   机身高度会被影响，`min_z=0.043` 这种值就很危险。

2. **没有保证目标 roll/pitch moment 真能实现**
   WBC 到关节力矩时可能因为几何、摩擦、接触状态被削掉。

### 建议改成“零和 vertical force redistribution”

目标不是“多给总 fz”，而是：

```text
保持 ΣΔfz = 0
通过左右 / 前后腿之间的 fz 差值产生 roll / pitch stabilizing moment
```

对 stance legs 建立：

```cpp
// 对每条支撑腿 i：foot position relative to body COM
// r_i = [x_i, y_i, z_i]
// 只用 vertical force delta: Δf_i = [0, 0, Δfz_i]
//
// torque contribution:
// τx += y_i * Δfz_i
// τy += -x_i * Δfz_i
//
// 同时要求:
// Σ Δfz_i = 0
```

矩阵形式：

```cpp
// A is 3 x N
// row0: 1       1       1       1       -> preserve total vertical force
// row1: y0      y1      y2      y3      -> roll torque
// row2: -x0     -x1     -x2     -x3     -> pitch torque

Eigen::MatrixXd A(3, n_stance);
Eigen::Vector3d b;
b << 0.0, tau_roll_cmd, tau_pitch_cmd;

Eigen::VectorXd dfz =
    A.transpose() * (A * A.transpose() + lambda * I).inverse() * b;
```

之后做：

```cpp
dfz_i = clamp(dfz_i, -fz_headroom_down_i, fz_headroom_up_i);
```

再把这些 `dfz_i` 加到 stance leg 的 nominal fz 上。

### 姿态支持建议链路

建议 `applyAttitudeSupport()` 变成下面结构：

```cpp
AttitudeSupportOutput applyAttitudeSupport(
    const RobotState& state,
    const CrossingStage stage,
    const std::array<ContactState, 4>& contacts,
    const std::array<Eigen::Vector3d, 4>& foot_pos_body,
    const ForcePlan& nominal_forces) {

  // 1. 用 SO(3) 或 gravity vector 计算 roll/pitch error
  Eigen::Vector2d e_rp = computeRobustRollPitchError(state.orientation);

  // 2. 低通 + deadband，防止 Euler wrap / contact noise
  e_rp = lowPass(e_rp);
  e_rp = applyDeadband(e_rp, params.deadband);

  // 3. PD 生成 desired body torque
  double tau_roll  = -kp_roll  * e_rp.x() - kd_roll  * state.omega_body.x();
  double tau_pitch = -kp_pitch * e_rp.y() - kd_pitch * state.omega_body.y();

  // 4. stage-specific clamp
  tau_roll  = clamp(tau_roll,  -tau_roll_max,  tau_roll_max);
  tau_pitch = clamp(tau_pitch, -tau_pitch_max, tau_pitch_max);

  // 5. 根据 stance set 计算 Δfz，要求 sum Δfz = 0
  auto dfz = solveVerticalForceRedistribution(
      foot_pos_body, contacts, tau_roll, tau_pitch);

  // 6. 加回 nominal forces，并记录 saturation / residual
  return applyAndLog(nominal_forces, dfz);
}
```

### 这里的重点

不要再把问题理解成“竖直力不够”。你已经证明：

```text
femur/tibia sat_count=0
fz_cap_min > 单腿 fz≈100N
```

所以正确方向是：

> 不是腿力不够，而是 stance geometry + force distribution + reference timing 不够保守。

---

## 2.4 `wbc_node_complete.cpp`：当前诊断方向对，但还缺“期望力 vs 实际力”

你现在加的：

```text
fu/tu/sat/jzf/jzt/fz_cap/fz_margin
```

很有价值，已经帮你排除了 femur/tibia 饱和和雅可比垂向容量不足。

下一步建议加这些：

```text
leg_id
stage
contact_cmd
contact_est
foot_pos_body
foot_vel_world
desired_fz
achieved_fz_est
desired_tau_hip
desired_tau_femur
desired_tau_tibia
commanded_tau_*
clamped_tau_*
fz_cap
fz_margin
slip_speed
rail_contact_flag
rail_delta
```

关键是要看到：

```text
MPC 想要的 force
WBC 实际能映射出的 torque
Gazebo 里实际接触是否符合预期
```

否则你只能知道“没有饱和”，但不知道“力是否真的打到了想要的方向”。

---

## 2.5 `crossing_check.py`：现在的 PASS 语义应该拆成两层

当前结果：

```text
PASS: stage=CROSSING:BODY_FORWARD_SHIFT
```

这对阶段开发有帮助，但不能再叫最终 crossing pass。

建议拆成：

| 测试名                           | 通过条件                     | 用途   |
| ----------------------------- | ------------------------ | ---- |
| `window_crossing_stage_smoke` | 能进入 `BODY_FORWARD_SHIFT` | 防止退化 |
| `window_crossing_full_test`   | 必须到 `RECOVERY` 或 `DONE`  | 最终验收 |
| `flat_smoke_test`             | 平地不退化                    | 回归保护 |

### 最终 crossing pass 建议条件

```text
reached_stage >= RECOVERY
max_x > window_exit_x
body_z_min > 0.08
body_z_max < 0.35
abs(roll)_max < 0.55 rad
abs(pitch)_max < 0.55 rad
max_rail_delta < configured_limit
no torque saturation burst
no estimator fatal
no controller crash
```

另加输出：

```text
stage_timeline:
  PRE_APPROACH enter/exit time
  APPROACH enter/exit time
  BODY_FORWARD_SHIFT enter/exit time
  FRONT_LEGS_TRANSIT enter/exit time
  REAR_LEGS_TRANSIT enter/exit time
  RECOVERY enter/exit time
```

这样下一轮不会只看最后一行 PASS/FAIL，而是能知道卡在哪个 transition。

---

# 3. 下一步最该改的 3 个文件

## 优先级 1：`trajectory_generator.cpp`

目标：把 crossing 初段改成 quasi-static。

必须做：

```text
PRE_APPROACH / APPROACH 禁止 aggressive walking reference
添加 hold-settle 段
限制 vx / ax / jerk
锁 y_ref / yaw_ref
roll/pitch 超阈值时冻结 x_ref
BODY_FORWARD_SHIFT 前必须生成稳定 body COM reference
```

---

## 优先级 2：`crossing_state_machine.cpp`

目标：状态机从“进度驱动”改成“稳定性驱动”。

必须做：

```text
添加 StageStabilityMetrics
添加 sustained stability timer
BODY_FORWARD_SHIFT -> FRONT_LEGS_TRANSIT 加稳定门控
FRONT_LEGS_TRANSIT / REAR_LEGS_TRANSIT 加 timeout + retry/fallback
日志输出每次不推进的原因
```

例如：

```text
[stage_gate] hold BODY_FORWARD_SHIFT:
  reason=roll_unstable
  roll=0.62 limit=0.30
  stable_time=0.00 required=0.50
```

这比只看“没有进入 FRONT_LEGS_TRANSIT”有用得多。

---

## 优先级 3：`mpc_node_complete.cpp`

目标：把姿态补偿改成鲁棒 attitude torque + bounded force redistribution。

必须做：

```text
不要直接依赖 Euler roll
用 SO(3) 或 gravity vector 姿态误差
roll/pitch torque 做低通、限幅、限速
用 stance foot geometry 解 Δfz
保持 ΣΔfz = 0
记录 desired_tau / achieved_dfz / residual / clamp
```

---

# 4. 不建议现在做的事

## 暂时不要继续大幅加 `attitude_support_kp`

原因：

```text
roll 大扰动可能不是单纯增益不足
如果姿态误差来自 Euler wrap，增益越大越坏
如果参考过激，增益只是在追一个错误轨迹
如果支撑几何不稳，增益会把力打到错误腿上
```

## 暂时不要先改 femur/tibia torque limit

你已经验证：

```text
sat_count=0
fz_cap_min 仍高于单腿 fz≈100N
```

所以 torque limit 不是主因。继续改它只会掩盖真实问题。

## 暂时不要把 `crossing_check.py` 的判定放宽

现在应该更严格，而不是更松。`BODY_FORWARD_SHIFT` pass 可以保留为阶段 smoke，但最终 pass 必须看 `RECOVERY`。

---

# 5. 可直接交给代码助手的详细 Prompt

下面这些 prompt 可以直接丢给 Cursor / Claude Code / ChatGPT Code Interpreter 类工具使用。

---

## Prompt A：总任务 Prompt

```text
你是资深四足机器人控制工程师。请审阅并修改这个 ROS2/Gazebo 四足机器人 crossing 控制栈，目标是让 window crossing 从当前只能稳定到 CROSSING:BODY_FORWARD_SHIFT，继续稳定推进到 FRONT_LEGS_TRANSIT、REAR_LEGS_TRANSIT 和 RECOVERY。

当前已知事实：
1. window_crossing_test 当前有效结果：
   PASS: stage=CROSSING:BODY_FORWARD_SHIFT; max_x=1.659; min_z=0.043; max_z=0.399; max_rail_delta=0.112
2. flat smoke_test 通过：
   PASS: turn_yaw_delta=2.822 z=0.156
3. rails 在 crossing 过程中真实参与，不是装饰状态。
4. femur/tibia torque saturation 不是主因：
   sat_count=0
5. 雅可比垂向容量不是主因：
   fz_cap_min 仍高于单腿 fz≈100N
6. 当前最大问题是 PRE_APPROACH / APPROACH / BODY_FORWARD_SHIFT 初段姿态扰动大，尤其 roll 峰值日志到过约 3.14 rad。
7. 需要优先判断 roll=3.14 是真实翻滚还是 Euler angle wrap / frame 变换问题。

重点文件：
- src/dog2_mpc/src/mpc_node_complete.cpp
- src/dog2_mpc/src/crossing_state_machine.cpp
- src/dog2_mpc/src/trajectory_generator.cpp
- src/dog2_wbc/src/wbc_node_complete.cpp
- src/dog2_bringup/config/research_mpc.yaml
- src/dog2_bringup/dog2_bringup/crossing_check.py
- src/dog2_bringup/launch/window_crossing_test.launch.py

总体修改目标：
1. 将 crossing 初段从 walking-style reference 改成 quasi-static contact sequence。
2. PRE_APPROACH / APPROACH / BODY_FORWARD_SHIFT 必须限速、限加速度、限 jerk，并加入 settle/hold。
3. 状态机不能只按 x/time 推进，必须加入稳定性门控：
   - abs(roll)
   - abs(pitch)
   - yaw rate
   - body z
   - support polygon margin
   - foot slip
   - min fz margin
   - contact validity
4. mpc_node_complete.cpp 中的 applyAttitudeSupport() 不要直接依赖 Euler roll/pitch；改成 SO(3) 或 gravity-vector roll/pitch error。
5. 姿态补偿不要简单增加总 fz，而要做 bounded vertical force redistribution：
   - Σ Δfz_i = 0
   - τx = Σ y_i Δfz_i
   - τy = -Σ x_i Δfz_i
   - 按 stance legs 和 fz headroom 解最小范数 Δfz
6. WBC 诊断要加入 desired vs achieved force/torque，以及 foot slip / contact / rail delta。
7. crossing_check.py 要拆分阶段 smoke 与最终 full crossing 判定：
   - BODY_FORWARD_SHIFT 只能算 stage smoke
   - RECOVERY / DONE 才能算 full pass

不要做：
- 不要简单放大 attitude_support_kp。
- 不要优先改 femur/tibia torque limit。
- 不要放宽 crossing_check 的最终通过条件。
- 不要破坏 flat smoke_test。

验收标准：
1. colcon build 通过。
2. flat smoke_test 仍通过。
3. window_crossing_stage_smoke 至少仍能到 BODY_FORWARD_SHIFT。
4. window_crossing_full_test 能稳定推进到 FRONT_LEGS_TRANSIT，目标继续到 REAR_LEGS_TRANSIT / RECOVERY。
5. 日志必须能解释每次 stage 不推进的原因，例如：
   [stage_gate] hold BODY_FORWARD_SHIFT reason=roll_unstable roll=0.62 limit=0.30 stable_time=0.00 required=0.50
6. 如果 roll 出现接近 pi，日志必须显示 quaternion、gravity_body、SO3 error，用于判断是否为 Euler wrap。
```

---

## Prompt B：专改 `trajectory_generator.cpp`

```text
请重点修改 src/dog2_mpc/src/trajectory_generator.cpp，让 crossing 初段参考从 walking-style 改为 quasi-static reference。

背景：
当前 window crossing 可以进入 CROSSING:BODY_FORWARD_SHIFT，但不能稳定推进到 FRONT_LEGS_TRANSIT / REAR_LEGS_TRANSIT / RECOVERY。初段 PRE_APPROACH / APPROACH 姿态扰动很大，roll 峰值曾到约 3.14 rad。已确认 femur/tibia torque saturation 不是主因，fz capacity 也不是主因。

目标：
1. PRE_APPROACH / APPROACH 不再输出普通 walking/gait 风格的连续快速前进参考。
2. 改成 hold-settle-move-settle 的准静态参考：
   - 进入 PRE_APPROACH 后先 hold 0.5~0.8s。
   - body x_ref 以 very low vx 推进。
   - vx_max 默认 0.02~0.03 m/s。
   - ax_max 默认 0.03~0.05 m/s^2。
   - jerk 尽量平滑，可用 smoothstep / quintic interpolation。
   - y_ref 固定在 crossing centerline。
   - yaw_ref 固定为 0 或 window normal direction。
   - body_z_ref 不要剧烈变化。
3. 如果外部可获得姿态状态，则当 abs(roll) 或 abs(pitch) 超过阈值时冻结 x_ref，不继续推进。
4. BODY_FORWARD_SHIFT 阶段不要像普通行走那样推 velocity，而是生成 body COM shift reference：
   - 速度更低，vx_max 约 0.01~0.02 m/s。
   - 必须预留 settle time。
   - 保持 yaw/lateral lock。
5. 保留原有接口，除非确实需要增加最小参数结构。
6. 所有新增参数接入 src/dog2_bringup/config/research_mpc.yaml，提供保守默认值。
7. 添加日志：
   [traj_crossing] stage=... x_ref=... y_ref=... yaw_ref=... vx_ref=... hold=... frozen=... reason=...

实现建议：
- 添加 CrossingQuasiStaticParams。
- 添加 smoothStep5 或 quintic interpolation：
  s = clamp(t/T, 0, 1)
  q = 10s^3 - 15s^4 + 6s^5
- 对每个 stage 维护 stage_start_time 和 stage_start_pose。
- 不要在 PRE_APPROACH / APPROACH 输出 lateral velocity。
- 不要在初段输出 yaw velocity。
- 如果原代码有普通 gait velocity blending，请在 crossing quasi_static_enabled 时绕开它。

验收：
- 平地 smoke_test 不变。
- window_crossing_test 至少仍到 BODY_FORWARD_SHIFT。
- 日志显示 PRE_APPROACH / APPROACH 的 vx_ref 明显小于普通 walking。
- BODY_FORWARD_SHIFT 期间 roll/pitch 超阈值时 x_ref 会冻结。
```

---

## Prompt C：专改 `crossing_state_machine.cpp`

```text
请修改 src/dog2_mpc/src/crossing_state_machine.cpp，把 crossing 状态推进从 progress-only 改成 stability-gated。

当前问题：
系统能稳定进入 CROSSING:BODY_FORWARD_SHIFT，但不能稳定推进到 FRONT_LEGS_TRANSIT / REAR_LEGS_TRANSIT / RECOVERY。初段姿态扰动大，roll 峰值到过约 3.14 rad。需要防止状态机在姿态未稳定时进入腿跨越阶段。

目标：
1. 新增 StageStabilityMetrics，至少包含：
   - roll_abs
   - pitch_abs
   - yaw_rate_abs
   - body_z
   - body_vx
   - body_vy
   - support_margin
   - max_foot_slip
   - min_fz_margin
   - max_rail_delta
   - all_required_contacts_valid
2. 新增 isStableForTransit(metrics, stage)。
3. 从 BODY_FORWARD_SHIFT -> FRONT_LEGS_TRANSIT 必须满足稳定性条件持续 required_stable_time。
4. FRONT_LEGS_TRANSIT -> REAR_LEGS_TRANSIT 和 REAR_LEGS_TRANSIT -> RECOVERY 也要有类似稳定性门控。
5. 如果条件不满足，不要静默卡住；必须打印明确原因：
   [stage_gate] hold BODY_FORWARD_SHIFT reason=roll_unstable roll=0.62 limit=0.30 stable_time=0.00 required=0.50
6. 增加 hysteresis：
   - enter threshold 更严格
   - remain threshold 稍宽
   防止边界附近来回抖动。
7. 增加 timeout + retry/fallback：
   - 如果某阶段超过 timeout 仍不稳定，进入 SAFE_HOLD 或回退到更保守 reference，不要盲目推进。
8. 不要让 BODY_FORWARD_SHIFT 的 stage smoke pass 误认为最终 crossing pass。

建议默认阈值：
- max_roll_for_transit: 0.30 rad
- max_pitch_for_transit: 0.30 rad
- max_yaw_rate_for_transit: 0.40 rad/s
- support_margin_min: 0.025 m
- foot_slip_max: 0.025 m/s
- required_stable_time: 0.50 s
- body_z_min: 0.10 m
- body_z_max: 0.30 m

注意：
如果某些 metrics 当前拿不到，请先用 available fields 实现框架，并在日志中标明 unavailable，不要虚构数据。优先保证状态机具备稳定性门控结构。

验收：
- 日志能显示每个 stage 进入、退出、hold 原因。
- BODY_FORWARD_SHIFT 不再只靠 x/time 推进。
- 不满足稳定性时不会进入 FRONT_LEGS_TRANSIT。
- 满足稳定性持续 required_stable_time 后才推进。
```

---

## Prompt D：专改 `mpc_node_complete.cpp`

```text
请修改 src/dog2_mpc/src/mpc_node_complete.cpp 中 crossing 初段姿态支持逻辑，重点审阅并重构 applyAttitudeSupport()。

当前事实：
1. window crossing 已能进入 BODY_FORWARD_SHIFT。
2. flat smoke_test 通过。
3. femur/tibia torque saturation 不是主因：sat_count=0。
4. fz capacity 不是主因：fz_cap_min 仍高于单腿 fz≈100N。
5. 当前阻塞是 PRE_APPROACH / APPROACH 初段姿态扰动过大，roll 峰值曾接近 3.14 rad。
6. 现在已有 attitude_support_* 参数和 vertical_support headroom 修改，但还不足以稳定推进到 FRONT_LEGS_TRANSIT。

目标：
1. 不要直接依赖 Euler roll/pitch 作为唯一姿态误差。
2. 新增 robust attitude error：
   - 优先用 SO(3) error：
     e_R = 0.5 * vee(R_des^T R - R^T R_des)
   - 或用 gravity vector error 计算 roll/pitch leveling error。
3. yaw 单独处理，不要让 yaw wrap 污染 roll/pitch。
4. 对 roll/pitch error 做：
   - low-pass
   - deadband
   - rate limit
   - clamp
5. 姿态支持输出 desired body torque：
   tau_roll = -kp_roll * e_roll - kd_roll * omega_x
   tau_pitch = -kp_pitch * e_pitch - kd_pitch * omega_y
6. 不要简单增加总 vertical force。
   改成 stance-leg vertical force redistribution：
   - Σ Δfz_i = 0
   - τx = Σ y_i Δfz_i
   - τy = -Σ x_i Δfz_i
   - 对 stance legs 解最小范数 Δfz
   - 按每条腿 fz headroom clamp
7. 记录 force redistribution residual：
   - desired_tau_roll
   - achieved_tau_roll
   - desired_tau_pitch
   - achieved_tau_pitch
   - dfz_i
   - clamp_i
   - residual norm
8. 如果 roll 接近 pi，必须打印：
   - quaternion
   - quaternion norm
   - rpy
   - gravity_body
   - SO3 error
   以判断是真翻车还是 Euler wrap。

建议实现函数：
- computeSO3RollPitchError(...)
- lowPassAttitudeError(...)
- solveVerticalForceRedistribution(...)
- applyFzHeadroomClamp(...)
- logAttitudeSupportDebug(...)

vertical force redistribution 公式：
给 stance leg i 的 foot_pos_body = [x_i, y_i, z_i]。
只求 Δfz_i。
构造 A:
row 0: 1
row 1: y_i
row 2: -x_i
b = [0, tau_roll, tau_pitch]^T
dfz = A^T * inv(A*A^T + lambda*I) * b
然后 clamp。

注意：
- 如果 stance leg 少于 3 条，降低补偿强度或直接进入 safe hold。
- 如果 A*A^T 条件数太差，降低 torque command，避免数值爆炸。
- 不要破坏 flat smoke_test。
- 参数全部放进 research_mpc.yaml，默认保守。

验收：
- crossing 日志中能看到 attitude support 的 desired/achieved torque。
- roll/pitch error 不会因为 Euler wrap 突然跳到错误方向。
- BODY_FORWARD_SHIFT 阶段如果姿态扰动，系统优先稳住，不继续推 x_ref。
```

---

## Prompt E：专改 `wbc_node_complete.cpp` 诊断

```text
请增强 src/dog2_wbc/src/wbc_node_complete.cpp 的 crossing 诊断日志，不改变核心控制输出，除非发现明显 bug。

当前已有诊断：
fu/tu/sat/jzf/jzt/fz_cap/fz_margin

这些诊断已经证明 femur/tibia torque saturation 不是主因，fz capacity 也不是主因。下一步需要确认 MPC 期望力、WBC 映射力矩、Gazebo 接触之间是否一致。

请新增每条腿的 crossing debug：
- stage
- leg_id
- contact_cmd
- contact_est
- foot_pos_body
- foot_pos_world
- foot_vel_world
- desired_force_world
- desired_fz
- estimated_or_commanded_fz
- desired_joint_torque
- commanded_joint_torque
- torque_clamp_flag
- fz_cap
- fz_margin
- slip_speed
- rail_contact_flag, 如果有
- rail_delta, 如果有

日志格式建议：
[wbc_leg_dbg] stage=BODY_FORWARD_SHIFT leg=FL contact_cmd=1 contact_est=1 fz_des=... fz_cap=... fz_margin=... tau=[...] sat=0 slip=... rail_delta=...

额外要求：
1. 不要刷爆日志，支持 throttle，例如每 0.1s 或每 N tick。
2. crossing 模式下默认打开，普通 flat smoke 可关闭或低频。
3. 如果 desired_fz 和实际/可实现 fz 差距大，打印 residual。
4. 如果 foot slip 大，明确打印 slip warning。

验收：
- 一次 window_crossing_test 后，可以从日志判断：
  a. 哪条腿支撑不可信；
  b. MPC 是否要求了 WBC 无法实现的 force；
  c. 是否存在 foot slip；
  d. rail 接触是否导致 lateral/roll 扰动。
```

---

## Prompt F：专改 `crossing_check.py`

```text
请修改 src/dog2_bringup/dog2_bringup/crossing_check.py，把 window crossing 验收拆成 stage smoke 和 full crossing 两类。

当前问题：
最近一次结果为：
PASS: stage=CROSSING:BODY_FORWARD_SHIFT; max_x=1.659; min_z=0.043; max_z=0.399; max_rail_delta=0.112

这个 PASS 对阶段 smoke 有意义，但不能代表完整越窗成功。完整越窗必须至少进入 RECOVERY 或 DONE。

目标：
1. 增加 --mode 参数：
   --mode stage_smoke
   --mode full
2. stage_smoke 允许 BODY_FORWARD_SHIFT 作为 pass。
3. full 模式必须达到：
   - FRONT_LEGS_TRANSIT
   - REAR_LEGS_TRANSIT
   - RECOVERY 或 DONE
   其中最终 pass 必须达到 RECOVERY/DONE。
4. 输出 stage timeline：
   - 每个 stage 首次进入时间
   - 每个 stage 持续时间
   - 最终 stage
5. 增加姿态/高度安全阈值：
   - body_z_min
   - body_z_max
   - max_abs_roll
   - max_abs_pitch
   - max_rail_delta
6. 如果 roll 接近 pi，输出 warning：
   POSSIBLE_EULER_WRAP_OR_FLIP
7. 保持 flat smoke_test 不受影响。

建议最终 full pass 条件：
- final_stage in [RECOVERY, DONE]
- max_x > configured_window_exit_x
- min_z > 0.08
- max_z < 0.35
- max_abs_roll < 0.55
- max_abs_pitch < 0.55
- max_rail_delta < configured_limit
- no fatal controller/estimator exception

输出格式：
PASS_FULL: final_stage=RECOVERY max_x=... min_z=... max_roll=... timeline=...
PASS_STAGE_SMOKE: final_stage=BODY_FORWARD_SHIFT ...
FAIL: reason=never_reached_front_legs_transit final_stage=BODY_FORWARD_SHIFT stable_gate_reason=...

验收：
- 原来的 BODY_FORWARD_SHIFT 结果在 stage_smoke 下 pass。
- 同样结果在 full 下 fail，并明确说明还没达到完整越窗。
```

---

# 6. 推荐开发顺序

按这个顺序做，风险最低：

```text
1. crossing_check.py
   先把验收拆清楚，避免假 pass。

2. wbc_node_complete.cpp
   扩诊断，不改控制，先拿证据。

3. mpc_node_complete.cpp
   修姿态误差表示，确认 roll=3.14 是否为 Euler wrap。

4. trajectory_generator.cpp
   改 quasi-static reference。

5. crossing_state_machine.cpp
   加稳定性门控和 stage hold reason。

6. 再回头调 research_mpc.yaml 参数。
```

不建议一上来同时大改全部控制链。最安全的节奏是：

```text
诊断增强 -> 姿态误差修正 -> 轨迹变保守 -> 状态机门控 -> 参数收敛
```

---

# 7. 最核心的代码设计目标

最终 crossing 控制应该变成这种结构：

```text
trajectory_generator:
  生成保守、准静态、可冻结的 reference

crossing_state_machine:
  只在稳定性满足时推进 stage

mpc_node:
  计算鲁棒姿态误差
  生成 bounded body torque
  做 stance-leg force redistribution

wbc_node:
  执行力/力矩映射
  输出 desired vs achieved diagnostics

crossing_check:
  区分 stage smoke 和 full crossing pass
```

当前最关键的一句话修改目标：

> **不要再让机器人“边姿态发散边继续向前走”；应该让 crossing 初段具备“姿态不稳就冻结 reference，稳定后才推进 stage”的机制。**

