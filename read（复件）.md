现在的进度可以概括成一句话：**window crossing 已经重新稳定推进到 `CROSSING:BODY_FORWARD_SHIFT`，平地 smoke 回归通过，但还没有稳定走完 `FRONT_LEGS_TRANSIT / REAR_LEGS_TRANSIT / RECOVERY`，也还没有达到“按最终形态完整越窗”的终态。**

**现在到哪一步了**
- 研究栈 `MPC + WBC + rail` 在 Gazebo 里已经不是只会进 `crossing` 模式，而是能在窗型 world 下推进到 `BODY_FORWARD_SHIFT`。
- `window_crossing_test` 最近一次有效结果是：
  - `PASS: stage=CROSSING:BODY_FORWARD_SHIFT; max_x=1.659; min_z=0.043; max_z=0.399; max_rail_delta=0.112`
- 平地回归没坏：
  - `smoke_test` 结果是 `PASS: turn_yaw_delta=2.822 z=0.156`
- 已经确认 rails 在 crossing 过程中真实参与，不是装饰状态。

**目前遇到的核心问题**
- crossing 还**没有稳定推进到**：
  - `FRONT_LEGS_TRANSIT`
  - `REAR_LEGS_TRANSIT`
  - `RECOVERY`
- 当前主阻塞不是 rail，也不是 `femur/tibia` 力矩不够，而是：
  - PRE_APPROACH / APPROACH 阶段机身姿态扰动太大；
  - 尤其 `roll` 扰动很夸张，日志里峰值到过约 `3.14 rad`；
  - 这会把初段支撑稳定性打散，导致状态机很难稳稳进入后续跨柱阶段。
- 现在已经通过诊断排除了两条常见误判：
  - 不是 `femur/tibia` 饱和：`sat_count=0`
  - 不是雅可比垂向容量不够：`fz_cap_min` 仍高于单腿 `fz≈100N`

**实现的核心文件**
- [src/dog2_mpc/src/mpc_node_complete.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/mpc_node_complete.cpp:83)
  - 这是这轮最核心的文件。
  - 加了 crossing 初段的 `attitude_support_*` 参数。
  - 在控制链里加入了 `applyAttitudeSupport()`。
  - 在 [这里](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/mpc_node_complete.cpp:708) 改了 `vertical_support`，给姿态补偿预留 `fz` headroom。
- [src/dog2_wbc/src/wbc_node_complete.cpp](/home/dell/aperfect/carbot_ws/src/dog2_wbc/src/wbc_node_complete.cpp:275)
  - 加了分腿诊断日志：`fu/tu/sat/jzf/jzt/fz_cap/fz_margin`
  - 用来确认是不是 3R 力矩饱和或雅可比容量不足。
- [src/dog2_bringup/config/research_mpc.yaml](/home/dell/aperfect/carbot_ws/src/dog2_bringup/config/research_mpc.yaml:23)
  - 研究栈默认参数入口。
  - 现在把姿态补偿参数接进来了。
- [WORK_REPORT.md](/home/dell/aperfect/carbot_ws/WORK_REPORT.md:1368)
  - 已持续记录这轮目标、方法、失败、修复、验证和下一步。

**与 crossing 主流程强相关、后续还要继续收敛的实现文件**
- [src/dog2_mpc/src/crossing_state_machine.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/crossing_state_machine.cpp:1)
  - crossing 阶段推进逻辑本体。
- [src/dog2_mpc/src/trajectory_generator.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/trajectory_generator.cpp:1)
  - 下一步很可能要重点改这里，把 PRE_APPROACH / APPROACH 参考改得更 quasi-static。
- [src/dog2_bringup/launch/window_crossing_test.launch.py](/home/dell/aperfect/carbot_ws/src/dog2_bringup/launch/window_crossing_test.launch.py:1)
  - 当前主要验证入口。
- [src/dog2_bringup/dog2_bringup/crossing_check.py](/home/dell/aperfect/carbot_ws/src/dog2_bringup/dog2_bringup/crossing_check.py:1)
  - 当前 PASS/FAIL 的判定入口。

**遇到问题的文件**
- [src/dog2_mpc/src/mpc_node_complete.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/mpc_node_complete.cpp:730)
  - 这里不是“代码报错”，而是当前初段姿态补偿能力还不够，虽然已把系统重新推到 `BODY_FORWARD_SHIFT`，但还没压住大姿态扰动。
- [src/dog2_mpc/src/trajectory_generator.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/trajectory_generator.cpp:1)
  - 当前很可能还保留了过于“walking 化”的初段推进参考，这是下一步最可疑的问题源之一。
- [src/dog2_mpc/src/crossing_state_machine.cpp](/home/dell/aperfect/carbot_ws/src/dog2_mpc/src/crossing_state_machine.cpp:1)
  - 状态机还没有稳定跨过 `BODY_FORWARD_SHIFT` 进入更后面的腿跨越阶段。
- [src/dog2_state_estimation/.../sim_state_estimator_node.py](/home/dell/aperfect/carbot_ws/src/dog2_state_estimation)
  - 在 launch 结束时偶尔有退出异常痕迹，但它目前不是 crossing 行为主阻塞，更像收尾阶段噪声。
- [src/dog2_motion_control/dog2_motion_control/gz_pose_to_odom.py](/home/dell/aperfect/carbot_ws/src/dog2_motion_control/dog2_motion_control/gz_pose_to_odom.py:1)
  - 结束时也有 `ExternalShutdownException` 类日志，当前更多是 shutdown 噪声，不是本轮主要算法阻塞。

**一句话判断**
- 已经从“能进 crossing”推进到“能在窗型 world 下稳定到 `BODY_FORWARD_SHIFT`”。
- 当前真正卡住的是：**crossing 初段参考和姿态稳定还不够保守，导致机身姿态先炸，后续前后腿跨柱阶段进不去。**

如果你要，我下一条可以直接把“下一步最该改的 3 个文件 + 每个文件具体该怎么改”列成非常具体的开发清单。
