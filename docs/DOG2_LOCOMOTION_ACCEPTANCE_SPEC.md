# Dog2 无头行走验收规范

版本：LAV1-R2 / result schema v2（2026-07-10）

## 1. 目的与边界

本规范定义 Dog2 在 Gazebo Fortress + ROS 2 Humble 中的无头平地行走验收。
它用于回答“机器人是否在给定协议下安全、可重复地完成基础运动任务”，而不是
替代控制器调试日志，也不以旧 `smoke_check.py` 的宽松运动门作为依据。

验收分为三层：

1. `stack_smoke`：节点、话题、控制器和数据流是否存活；
2. `locomotion_acceptance`：固定任务是否完成，且全程没有安全硬失败；
3. `mobility_benchmark`：坡面、台阶、低摩擦和窗框等任务能力，后续单独建立。

LAV1 只实现第 2 层。旧 smoke 的 PASS 不能替代 LAV1。

## 2. 为什么参考这些来源

### 2.1 ROS 2 `launch_testing`

来源：

- ROS 2 官方文档，Writing Basic Integration Tests with `launch_testing`：
  <https://docs.ros.org/en/kilted/Tutorials/Intermediate/Testing/Integration.html>

采用理由：

- 它解决的是无头工程测试的运行方式，而不是四足运动学本身；
- 官方做法包含独立 `ROS_DOMAIN_ID`、启动期与退出后断言、超时以及 JUnit 输出；
- 这些机制可避免残留节点串扰和“进程没退出但结果文件写了 PASS”的假成功。

Dog2 的采用方式：

- 每次试验使用独立 ROS domain id 和 Gazebo transport partition；
- checker 以进程退出码和结构化结果共同给出结果；
- 每次运行输出 JSON、CSV 和 JUnit XML，便于本地与 CI 使用；
- 所有等待都有明确超时；
- 每轮绑定 `run_uuid`，CSV/JUnit 完成后最后原子提交 JSON；batch 校验
  UUID、trial ID、mtime、schema 和 artifact SHA-256，拒绝旧报告。

### 2.2 NVIDIA Isaac Lab

来源：

- 官方四足速度任务配置：
  <https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py>
- Isaac Lab Termination Manager：
  <https://isaac-sim.github.io/IsaacLab/v2.3.0/source/api/lab/isaaclab.managers.html>

采用理由：

- Isaac Lab 的 ANYmal、Unitree 等任务在每个仿真步连续读取 root state、关节状态和
  contact sensor，而不是只在阶段末抽样；
- 基础速度任务把 base 非法接触作为 episode termination，并把 timeout 与真实失败分开；
- 其评价维度包含线速度/角速度跟踪、重力投影姿态、垂向速度、关节限位、力矩、
  action rate 和足端接触。

Dog2 的采用方式：

- base/tibia 非法接触、翻倒、URDF 硬限位越界和超时属于连续硬门；
- 姿态使用 body +Z 在世界系的投影 `up_z` 与 tilt，避免 Euler wrap 假象；
- 速度跟踪、垂向运动和接触行为按完整时序统计。

限制：

- Isaac Lab 的 reward 权重用于训练，不是行业 PASS 阈值；
- LAV1 只借用“测什么、何时终止”的方法，不复制 reward 权重作为门限。

### 2.3 ETH `legged_gym`

来源：

- 官方仓库：
  <https://github.com/leggedrobotics/legged_gym>
- 基础环境：
  <https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot.py>

采用理由：

- `legged_gym` 是广泛复用的四足批量无头仿真基线；
- 它用仿真 contact-force tensor 检测指定 link 的非法接触并立即 reset；
- 它连续计算线/角速度跟踪、projected gravity、关节位置/速度/力矩软限位、
  足端 air time、stumble 和接触力；
- 它明确指出 mesh contact 数据可能不可靠，足端专用传感器更适合腿式机器人。

Dog2 的采用方式：

- 使用 Gazebo 专用 base、tibia、foot contact sensor，不用
  `/dog2/state_estimation/contact_state` 的 `all_true` 假接触；
- 非法接触以“存在 contact event”直接锁存为硬失败；Gazebo Fortress 未给 wrench
  时不得伪造 1 N，force 只作为可选证据；
- 足端滑移和摆动净空来自世界系足端轨迹与真实接触，不用机身高度冒充。

限制同 Isaac Lab：训练 reward/soft limit 不是通用验收阈值。

### 2.4 Eurobench 与 ANYmal 可复现基准

来源：

- Eurobench Performance Indicator Specification：
  <https://eurobench.github.io/sofware_documentation/latest/pi_spec.html>
- Eurobench Data Format：
  <https://eurobench.github.io/sofware_documentation/latest/data_format.html>
- “A Reproducible Benchmarking Methodology to Assess Robotic Locomotion
  Over Irregular Terrains: A Practical and Scalable Approach”，IEEE Robotics
  & Automation Magazine，2025：
  <https://doi.org/10.1109/mra.2025.3631570>

采用理由：

- 这是“固定协议 + 每次运行 PI + 跨运行聚合”的正式 benchmark 思路；
- 论文中的 ANYmal-C 从稳定站立开始，沿固定直线到终点、停止、再返回起点；
- 指标覆盖平均速度、路径偏差、足端滑移、机械 Cost of Transport 和成功率；
- 协议要求同一条件重复完整试验并报告跨运行统计，而不是挑最好的一次。

Dog2 的采用方式：

- 基础路线为按机身长度归一化的直线往返，长度、速度和 corridor 写入配置；
- task success 必须到达终点、停稳并返回起点；
- 路径偏差按机身宽度归一化，避免只看 `hypot(dx,dy)`；
- 单次报告原始 PI，批量入口默认执行十次并聚合
  mean/std/min/p95/max/final success/provisional rate。

说明：

- Eurobench 的 3 m 是其试验台协议，不是所有机器人必须复制的绝对距离；
- Dog2 使用机身尺度归一化路线，并在报告中同时给出米制原值和无量纲值。

### 2.5 NIST/ASTM 与 Barkour

来源：

- NIST Ground Robot Tests：
  <https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots/ground-robot-tests>
- ASTM E2826/E2826M、E2827/E2827M 等可重复坡面/地形测试；
- Barkour：
  <https://arxiv.org/abs/2305.14654>

采用理由：

- 它们强调固定装置、固定程序、重复运行和任务成功率；
- 适合以后定义坡面、台阶、复杂地形与窗框任务。

LAV1 不把 Barkour 时间分数或 NIST 响应机器人场地直接套在平地基础行走上。

## 3. 参考映射总表

下表说明“借什么、不借什么、在 Dog2 里落在哪”。这是 LAV1 的设计依据，不是
行业统一 PASS 表。

| 来源 | 借鉴内容 | Dog2 实现 | 阈值分类 | 明确不借 |
|------|----------|-----------|----------|----------|
| ROS 2 `launch_testing` | 独立 domain、超时、JUnit、进程退出码 | `locomotion_acceptance_test.launch.py`、`locomotion_acceptance_batch.py` | `TEST_INFRASTRUCTURE` | 不借 turtlesim 示例里的业务判据 |
| Isaac Lab velocity task | 每步 illegal base contact；timeout 与 terminated 分离 | Gazebo base contact event 锁存 | `REFERENCE_HARD` | 不借 reward weight（如 `track_lin_vel_xy_exp=1.0`） |
| Isaac Lab rewards/terminations | 线/角速度跟踪、projected gravity、垂向速度、joint limits | odom RMSE、`up_z`/tilt、URDF hard limit | 硬门 + `REPORT_ONLY` | 不借 `feet_air_time` 训练阈值作 PASS |
| `legged_gym` | contact-force tensor 非法接触；足端 contact 优先于 mesh | foot/tibia/base 专用 contact；stance slip 积分 | `REFERENCE_HARD` + `REPORT_ONLY` | 不借 soft limit 百分比作硬门 |
| Eurobench PI 框架 | 固定 protocol、每 run PI、跨 run 聚合 | 直线往返 + batch 默认 10 次 + aggregate JSON/CSV | `TASK_PROTOCOL` + 统计 | 不直接复制 ANYmal 3 m 绝对距离 |
| ANYmal 不规则地形基准 | 平均速度、Deviation Index、Slippage、CoT、Success rate | 归一化 lateral error、stance slip/path、commanded CoT | `REPORT_ONLY`（初版） | 不借 0.3/0.8 m/s 作为 Dog2 初版命令 |
| ETH ANYmal 论文 | 速度跟踪 RMSE、成功率、重复 trial | batch mean/std、首个失败码分布 | 报告 | 不借 1.0 m/s flying trot 作为 LAV1 目标 |
| Scientific Reports DRL 基准 | Fall rate、50 episodes、slippage ratio | 单次 trial 硬失败即 fall；batch success_rate | 硬门 + 聚合 | 不借 episode return 作 Dog2 PASS |
| NIST/ASTM / Barkour | 固定装置、重复运行、任务成功率 | 仅作为 LAV2+ 坡面/窗框扩展参考 | 未来 | LAV1 不套响应机器人场地 |

## 4. 阈值来源分类

每个 gate 必须标注以下来源之一：

- `REFERENCE_HARD`：外部框架明确用作 termination 的条件；
- `PHYSICAL_HARD`：URDF 或执行器给出的物理硬限位；
- `MISSION_REQUIREMENT`：Dog2 任务明确要求，例如 rail 锁定 5 mm；
- `PROJECT_SAFETY`：为阻止已观察到的危险行为而设，必须注明不是行业通用值；
- `BASELINE_DERIVED`：由已通过的标准拓扑或历史数据统计后确定；
- `TASK_PROTOCOL`：固定路线、站立、停止和净转角的任务要求；
- `TEST_INFRASTRUCTURE`：数据链、时钟、报告身份和运行隔离要求；
- `CALIBRATED_QUALITY`：由带正/负标签的数据集拟合并审查后的质量门；
- `REPORT_ONLY`：文献建议观测，但当前没有公认 PASS 阈值。

禁止把 `PROJECT_SAFETY`、`BASELINE_DERIVED` 或 reward 权重写成“行业标准”。

质量阈值的校准输入为 `report_json,label` 两列 CSV，`label` 只允许
`accept/reject`。工具只接受完整 schema v2 报告，并记录每份输入的 SHA-256：

```bash
ros2 run dog2_bringup locomotion_acceptance_calibrate \
  labels.csv \
  --minimum-per-class 10 \
  --output calibration_candidate.json
```

每项指标的 accept/reject 样本必须可分离；重叠或样本不足时工具输出
`insufficient_data`。即使全部可分离也只输出 `profile_status=candidate`，不会自动把
配置改成 calibrated。必须经过人工审查和独立 holdout 复核后，才能填写候选阈值、
工具生成的 `candidate_profile_id` 和 `calibration_status=calibrated`；缺少有效
profile ID 的 calibrated 配置属于基础设施失败。

## 5. LAV1 固定协议

一次 trial 的状态机：

1. `WAIT_READY`：等待 ground-truth、contact publisher、joint、controller command、
   simulation clock 和 URDF 限位；
2. `WAIT_SETTLE`：零速度等待稳定站立，并要求四个足端都实际观察到接触；
   此阶段不计入 benchmark；
3. `STAND`：零命令保持，连续检查漂移、平面速度、垂向速度和三轴角速度；
4. `OUTBOUND`：沿初始机身轴和命令符号行至固定终点；
5. `OUTBOUND_STOP`：终点停稳；
6. `RETURN`：反向返回起点；
7. `RETURN_STOP`：起点停稳；
8. `TURN`：原地执行固定 yaw 目标；
9. `FINAL_STOP`：停止后按最终净 yaw 检查目标误差，禁止只凭历史峰值通过。

Dog2 物理机头当前为 `base_link -X`，因此默认直行命令为负 `linear.x`。
验收器不假设“世界 X 就是前进”，而是冻结 trial 起始 yaw，并按命令符号构造
路线单位向量。

状态迁移只允许上述固定顺序。协议持续时间、阶段计时、速度差分和指标积分统一使用
仿真时间；wall time 只用于数据 freshness、仿真停滞和总看门狗。

默认批量协议为同配置十次 trial；正式稳定性结论建议至少 30 次并报告置信区间。

## 6. 数据源

验收只使用以下数据：

- base pose：Gazebo ground-truth `/odom`；验收速度由连续 stamped pose 差分得到，
  不信任上游 `twist` 的坐标系约定；
- foot 世界位姿：优先使用 Gazebo `/dog2/dynamic_pose_tf` 或完整 world TF；
  如果 headless SDF 的 fixed-joint reduction 未发布 `foot_link`，则由 Gazebo
  ground-truth `/odom`、Gazebo `/joint_states` 和运行时展开 URDF 通过
  Pinocchio 正运动学重建，并在 JSON 中记录
  `foot_world_position_source=gazebo_odom+joint_state_pinocchio`；
- foot/base/tibia contact：`ros_gz_interfaces/Contacts`；
- joint position/velocity：`/joint_states`；
- 最终旋转关节力矩命令：`/effort_controller/commands`；
- planned contact：`/dog2/gait/contact_phase`；
- joint hard limits、总质量、base collision 尺寸和 foot collision radius：
  运行时同一份展开 `robot_description`。因此 symmetric 变体实际使用 0.020 m
  足半径，而不是配置中的旧 0.012 m。

contact sensor 必须显式选择生成 SDF 中同一 link 上真实存在的 collision，并使用
`/dog2/gz_contact/*` 自有 Gazebo topic。`WAIT_SETTLE` 的四足正接触观测是运行时
数据链自检；仅存在 ROS publisher 不算健康。

以下数据禁止作为 ground-truth：

- `/dog2/state_estimation/contact_state`（当前为 `all_true`）；
- MPC 足端力命令（不是实测 GRF）；
- WBC rail effort（平地研究栈未实际执行）；
- 旧 `gait_quality` 中的硬编码 0。

## 7. 硬门与任务门

### 7.1 连续硬门

- 任意 base contact event：`REFERENCE_HARD`，回调立即锁存，不能因短脉冲过期漏检；
- 任意 tibia contact event：`PROJECT_SAFETY`，用于拒绝拖胫/腿杆撑地；
- tilt、`up_z` 或机身高度越界：`PROJECT_SAFETY`，用于拒绝 GUI 已观察到的
  翻滚假 PASS；
- joint position/velocity 超出 URDF hard limit：`PHYSICAL_HARD`；
- 最终 effort command 超出 URDF effort limit：`PHYSICAL_HARD`；
- rail 锁零误差超过 5 mm：`MISSION_REQUIREMENT`；
- 预期 joint position/velocity 或 12 路最终 effort command 缺失：测试基础设施失败；
- 必需 ground-truth/contact 过期、simulation clock 停滞、wall watchdog、报告身份或
  artifact 校验失败：测试基础设施失败；
- simulation-time 协议或阶段未完成：任务失败，不与基础设施超时混为一类。

任一硬门失败立即结束，并记录首个失败时间、阶段、测量值、限值和来源类别。

### 7.2 任务门

- outbound 到达沿路线投影终点；
- 全程横向偏差不越过配置 corridor；
- return 回到起点容差内；
- stand 漂移不越界且全程满足平面/垂向/三轴角速度稳定门；
- turn 的停止后净 yaw（unwrapped）误差和平移均不越界；
- 各 stop 阶段同时满足平面速度、垂向速度和三轴角速度稳定条件。

任务门采用 Eurobench 的固定路线/到达终点思想；Dog2 的尺度、速度和 corridor
属于显式 `PROJECT_SAFETY` 配置，不宣称为通用标准。

## 8. 统计指标与公式

### 8.1 与文献对应的公式

**Isaac Lab / `legged_gym` 速度跟踪**

- 线速度 RMSE（机体系）：
  \[
  e_v = \sqrt{\frac{1}{N}\sum_t \left[(v_x - v_x^{cmd})^2 + v_y^2\right]}
  \]
  LAV1 用 Gazebo pose 差分得到机体系速度，对 `vx_body - cmd_vx` 与 `vy_body`
  在线统计。
- 角速度 RMSE：
  \[
  e_\omega = \sqrt{\frac{1}{N}\sum_t (\omega_z - \omega_z^{cmd})^2}
  \]

**Isaac Lab projected gravity / `legged_gym` orientation**

- body +Z 在世界系竖直分量：`up_z = R_{wb}[2,2]`（实现中用四元数直接算）
- tilt：`tilt = atan2(||R_{wb}[0:2,2]||, up_z)`
- 硬门：`tilt ≤ max_tilt_rad` 且 `up_z ≥ cos(max_tilt_rad)`

**Eurobench / ANYmal Deviation Index 思想**

- 沿初始路线单位向量 \(\hat{s}\) 的投影与横向偏差：
  \[
  s = \Delta p \cdot \hat{s},\quad w = -\Delta p \cdot \hat{s}_\perp
  \]
- 归一化横向误差：`normalized_lateral_error = |w| / body_width`
- 任务 corridor 硬门：`|w| ≤ corridor_half_width`

**Eurobench / 文献 Slippage 思想**

- 支撑相足端水平位移积分：
  \[
  d_{slip} = \sum_{\text{stance ticks}} \sqrt{\Delta x^2 + \Delta y^2}
  \]
- 报告：`stance_slip_per_base_path = d_{slip} / base_path_length`
- LAV1 初版仅 `REPORT_ONLY`，待标准拓扑基线后再设 `BASELINE_DERIVED` 上限

**Scientific Reports foot slippage 比例**

- 文献定义：滑移距离 / 尝试位移。Dog2 实现为 stance slip 与 base path 之比，
  并在 JSON 中保留分子分母，避免只报一个无量纲数。

**Eurobench / ANYmal Cost of Transport 思想**

- 报告用命令机械能近似：
  \[
  E_{cmd} = \sum_t \sum_i |\tau_{i,cmd}| | \dot q_i | \Delta t
  \]
  \[
  CoT_{cmd} = \frac{E_{cmd}}{m g d_{path}}
  \]
- 必须保留 `commanded_` 前缀；这不是实测电机能耗。

**Contact mismatch（Isaac Lab contact schedule 思想的验收版）**

- 计划支撑来自 `/dog2/gait/contact_phase`；
- 实测接触来自 foot contact sensor；
- 报告：`mismatch_ratio = mismatch_duration / observed_duration`

### 8.2 每次 trial 至少输出的 PI

LAV1 每次 trial 至少输出：

- `linear_velocity_rmse_mps`：机体系命令与实测线速度 RMSE；
- `yaw_rate_rmse_radps`：命令与实测 yaw rate RMSE；
- `tilt_rms/max_rad`、`min_up_z`；
- `body_height_mean/std/min/max_m`、`vertical_velocity_rms_mps`；
- `lateral_error_mean/max_m`；
- `normalized_lateral_error_mean/max`：横向误差除以机身宽度；
- `base_path_length_m`、终点/返程投影；
- 每足 `stance_slip_distance_m` 与总滑移；
- 每次 stance episode 的滑移、最大滑移与支撑足速度 P95；
- 每个计划 swing 的 peak clearance；
- planned/actual contact mismatch ratio、最长连续错配和计划摆动误触时间；
- 直行 heading error、三轴 angular speed 和停止后 turn final error；
- joint hard-limit 最小裕度、速度/effort 峰值；
- rail 最大锁定误差；
- `commanded_mechanical_energy_j` 与
  `commanded_cot = Σ|τ_cmd qdot|dt / (m g distance)`。

`commanded_cot` 不是实测电能或真实关节力矩 CoT，报告必须保留
`commanded_` 前缀。

滑移、净空、contact mismatch、heading、能耗和速度 RMSE 默认是
`REPORT_ONLY`。在 `calibration_status=provisional` 时，即使安全门和任务门全部通过，
也只能输出 `PASS_SAFETY_ROUTE_PROVISIONAL` 且 `passed=false`。只有带标签数据集完成
校准、所有质量阈值非负且数据完整后，才允许升级为 `CALIBRATED_QUALITY` 硬门。

## 9. 旧 smoke 与 LAV1 的差异

| 项目 | 旧 `smoke_check.py` | LAV1 `locomotion_acceptance` |
|------|---------------------|------------------------------|
| 数据源 | 状态估计 odom | Gazebo stamped pose 差分 + contact |
| forward PASS | `hypot(dx,dy) ≥ 0.2 m` | 沿初始机头投影到达固定终点 |
| turn PASS | 历史峰值 yaw delta | 停止后 unwrapped 净 yaw 误差 + 平移上限 |
| 姿态 | 不检查 | 连续 tilt / up_z / 身高 |
| 非法接触 | 不检查 | base/tibia contact 连续硬门 |
| 足端行为 | `gait_quality` 占位 0 | slip / swing clearance / mismatch |
| 导轨 | 仅 max drift | 连续硬门 5 mm |
| 重复性 | 单次 | batch 默认 10 次 + final/provisional rate |
| 结果 | 一行 PASS/FAIL | schema v2 JSON + CSV + JUnit + SHA-256 aggregate |

## 10. 结果语义

单次结果只能是：

- `PASS_LOCOMOTION_BASELINE`：安全门、任务门和已审查 calibrated quality gate 全通过；
- `PASS_SAFETY_ROUTE_PROVISIONAL`：安全门和任务门通过，但质量门尚未校准；
  此状态 `passed=false`，JUnit 为 skipped，不得宣传成正式行走 PASS；
- `FAIL_LOCOMOTION`：机器人或任务失败；
- `FAIL_INFRASTRUCTURE`：必需数据、服务或仿真基础设施失败。

PASS 不代表越障、粗糙地形、抗扰或真实硬件通过。

每次 trial 输出：

- `<trial>.json`：完整元数据、gate、阶段和聚合指标；
- `<trial>_samples.csv`：时序样本；
- `<trial>.junit.xml`：CI 结果。

批量运行每次创建新的 UUID 子目录；输出 aggregate JSON/CSV，至少包含正式成功率、
provisional rate、失败原因分布和各 PI 的 mean/std/min/p95/max。

## 11. 代码入口与运行方式

实现文件：

- 规范：本文件
- 验收节点：`src/dog2_bringup/dog2_bringup/locomotion_acceptance.py`
- 纯判定内核：`src/dog2_bringup/dog2_bringup/acceptance_oracle.py`
- 指标纯函数：`src/dog2_bringup/dog2_bringup/acceptance_metrics.py`
- 批量聚合：`src/dog2_bringup/dog2_bringup/locomotion_acceptance_batch.py`
- 配置：`src/dog2_bringup/config/locomotion_acceptance.yaml`
- Launch：`src/dog2_bringup/launch/locomotion_acceptance_test.launch.py`
- 机械拓扑回归：`src/dog2_description/test/test_acceptance_instrumentation_topology.py`

单次无头 trial：

```bash
cd /home/dell/aperfect/carbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select dog2_description dog2_bringup --symlink-install
source install/setup.bash

ros2 launch dog2_bringup locomotion_acceptance_test.launch.py \
  ros_domain_id:=64 \
  run_uuid:=00000000-0000-0000-0000-000000000001 \
  transport_partition:=dog2_lav1_manual_001 \
  trial_id:=trial_001 \
  result_json:=/tmp/dog2_locomotion_acceptance/trial_001.json
```

默认十次 trial 与聚合：

```bash
ros2 run dog2_bringup locomotion_acceptance_batch \
  --trials 10 \
  --domain-start 64 \
  --result-dir /tmp/dog2_locomotion_acceptance_batch
```

正式稳定性基线建议 `--trials 30`，并在 aggregate 中报告 mean ± std、P95、
final/provisional rate 与 failure code 分布；不要只展示最好的一次。

## 12. 机械拓扑冻结保证

验收仪器只允许增加 Gazebo `<sensor type="contact">`，不得修改 Dog2 机械模型。

自动回归必须验证 `enable_acceptance_contact_sensors=false/true` 两次展开的：

- movable joint 数量均为 16；
- 4 个 prismatic rail 与 12 个 revolute joint 名称一致；
- 每个 movable joint 的 type、parent、child、axis、origin 和 limit 完全一致；
- link inertial、collision 和 visual 不因仪器开关发生改变。
- 生成 SDF 的每个 contact sensor selector 均指向同一 link 上存在的 collision；
- 不允许出现 `__default__` collision/topic，也不允许用 `preserveFixedJoint`
  为验收改变 fixed-joint lumping。

任一差异都视为验收功能自身回归，禁止继续运行。
