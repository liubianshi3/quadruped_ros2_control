# Dog2 Gazebo Research Stack Work Report

## 2026-04-24 22:44 CST

### 现在想实现什么
- 在不修改 URDF / xacro / mesh 的前提下，把 Dog2 的 `MPC + WBC` Gazebo 研究栈从“flat ground smoke test 可通过”推进到“更稳定、可复现、可进入复杂场景”的状态。

### 准备用什么方法实现
- 先检查远端 `~/aperfect/carbot_ws` 当前仓库状态和研究栈相关 launch / bringup 代码。
- 优先收敛启动链路和验证链路中的脆弱点，包括：
  - `gz_gain_setter` 退出噪声；
  - 启动顺序竞争；
  - 旧进程残留导致的验证污染；
  - cold start 场景下的 topic / controller / stream 准备状态。
- 然后增加一个不依赖 URDF 改动的复杂场景入口，优先考虑小障碍 / 台阶 / crossing 工程入口。

### 为什么选这个方法，而不是其他方法
- 当前研究栈已经能通过 flat-ground smoke test，说明主链路不是“从零不可用”，而是“离工程稳定还有距离”。
- 此时继续大改算法或机器人模型风险很高，也不符合最小可行修改原则。
- 先把 launch、验证和复杂场景入口工程化，能更快把系统推进到可复现状态，并为后续 crossing 算法收敛提供稳定基座。

### 当前具体做了哪些操作
- 通过 `ssh dell` 确认远端可直接免交互连接。
- 定位工作区为 `~/aperfect/carbot_ws`。
- 检查远端 `git status` 与 `dog2_bringup` 目录结构。
- 确认 `WORK_REPORT.md` 尚不存在，开始创建本文件。

### 过程中遇到了什么问题
- 仓库当前为明显脏工作区，包含大量未提交改动，且 `src/dog2_bringup/` 整体处于未跟踪状态。

### 问题的原因判断是什么
- 当前仓库很可能处于一轮大规模本地开发中，`dog2_bringup`、`state_estimation`、`gait_planner` 等集成层文件尚未纳入版本控制或尚未整理提交。
- 这意味着后续修改必须非常聚焦，避免覆盖已有本地实验成果。

### 做了哪些修改和尝试
- 当前仅创建工作汇报文件，尚未对业务代码作新的功能修改。

### 当前结果如何，是否成功
- 已完成远端环境和工作区状态确认。
- 已开始建立本轮工作的过程记录。

### 下一步准备做什么
- 检查 `system.launch.py`、`sim_base.launch.py`、`control_stack.launch.py`、`effort_research_sim.launch.py` 的当前状态。
- 重点分析 `gz_gain_setter` 生命周期、cold-start 验证入口以及复杂场景 world / crossing 入口。

## 2026-04-24 22:46 CST

### 现在想实现什么
- 在不动机器人模型的前提下，先完成三件事：
  1. 消除 `gz_gain_setter` 在 launch 收尾阶段的退出噪声；
  2. 让 smoke test 真正校验研究栈 cold-start 关键数据流的新鲜度；
  3. 增加一个可执行的小障碍 / crossing 研究入口。

### 准备用什么方法实现
- 直接修改 `dog2_bringup` 和 `dog2_motion_control` 的胶水层，而不碰 MPC/WBC 主算法：
  - `gz_gain_setter.py`：改为节点内自收敛，不在回调里直接触发全局 `rclpy.shutdown()`；
  - `smoke_check.py`：新增对 `/dog2/gait/contact_phase` 和 `/effort_controller/commands` 的 freshness 订阅与判断；
  - `smoke_test.launch.py`：加入 `ros_domain_id` 和 world 相关参数，默认用隔离 DDS 域跑 cold-start 验证；
  - 新增 `step_block.sdf` 与 `crossing_trial.launch.py`，给复杂场景提供固定入口；
  - 新增 one-shot crossing trigger 节点，用延时 Bool 消息触发 `/enable_crossing`。

### 为什么选这个方法，而不是其他方法
- 这些问题都属于集成层问题，改 launch / bringup / helper node 风险最低。
- `smoke_check` 之前对 topic 存在性覆盖够，但对“确实正在出流”覆盖还不够，容易出现假阳性。
- crossing 目前没有统一触发入口，先补一个 world + trigger + launch 组合，比直接深入改越障算法更稳。

### 当前具体做了哪些操作
- 读取并分析了：
  - `system.launch.py`
  - `sim_base.launch.py`
  - `control_stack.launch.py`
  - `effort_research_sim.launch.py`
  - `gz_gain_setter.py`
  - `smoke_check.py`
  - `smoke_test.launch.py`
  - `flat_ground.sdf`
  - `dog2_bringup` README 和 troubleshooting 文档
- 已在本地临时副本中完成以下补丁：
  - `gz_gain_setter.py`
  - `smoke_check.py`
  - `smoke_test.launch.py`
  - `setup.py`
  - `README.md`
  - `TROUBLESHOOTING.md`
  - 新增 `crossing_trigger.py`
  - 新增 `crossing_trial.launch.py`
  - 新增 `step_block.sdf`
  - 新增测试文件

### 过程中遇到了什么问题
- 文件同步阶段 `scp` 有一条组合命令表现得很慢，存在看起来像阻塞的情况。
- 新增测试文件最开始放到了错误目录。

### 问题的原因判断是什么
- 组合式 `scp` 命令在远端响应较慢时不够透明，不利于定位是哪一段慢。
- 新增测试文件时，临时路径和远端真实 `test/` 目录结构不一致，属于本地临时编辑路径错误。

### 做了哪些修改和尝试
- 旁路检查本地临时目录确认文件已复制成功，避免在可疑 `scp` 会话上空等。
- 立即修正了测试文件的路径，改到 `test/` 目录，并重新检查相对路径。
- 用 `python3 -m py_compile` 对新增/修改的 Python 文件做了本地语法检查，当前通过。

### 当前结果如何，是否成功
- 第一轮代码修改已完成，语法检查通过。
- 还没有开始远端构建和端到端验证。

### 下一步准备做什么
- 把补丁同步回远端。
- 在远端 `colcon build --packages-select dog2_bringup dog2_motion_control`。
- 跑一次隔离域 cold-start smoke test。
- 再跑一次新的复杂场景入口 `crossing_trial.launch.py` 做可执行性验证。

## 2026-04-24 22:51 CST

### 现在想实现什么
- 在保持第一轮修复成立的基础上，把复杂场景入口修到“不会因为 crossing 触发时机过早而直接崩溃”。

### 准备用什么方法实现
- 不仅调整 trigger 延时，而是直接修 `mpc_node_complete` 的 crossing 触发逻辑：
  - crossing 请求到来时，如果状态尚未 ready，则先挂起请求；
  - 等 `odom + joint state` ready 后，在控制循环中再真正切换到 crossing 模式。

### 为什么选这个方法，而不是其他方法
- 单纯增大 `crossing_delay_sec` 只是绕开问题，不是修复问题。
- 当前暴露出来的是一个典型的集成时序 bug：one-shot trigger 可能早于控制节点内部状态初始化。
- 在 `mpc_node_complete` 内部做防御式处理，才能让 crossing 入口对不同启动速度更稳。

### 当前具体做了哪些操作
- 远端完成了以下验证：
  - `colcon build --packages-select dog2_motion_control dog2_bringup --symlink-install`
  - `colcon test --packages-select dog2_bringup`
  - `pytest -q src/dog2_bringup/test/test_bringup_files.py src/dog2_bringup/test/test_research_stack_files.py`
  - `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true`
  - `ros2 launch dog2_bringup crossing_trial.launch.py use_gui:=false rviz:=false teleop:=false crossing_delay_sec:=3.0`

### 过程中遇到了什么问题
- cold-start smoke test 已通过，且 `gz_gain_setter` 现在能正常成功退出。
- 但在 `crossing_trial.launch.py` 中，`crossing_trigger` 发布 `/enable_crossing` 后，`mpc_node_complete` 触发了 Eigen 断言并崩溃。

### 问题的原因判断是什么
- `mpc_node_complete::enableCrossingCallback()` 直接访问 `current_srbd_state_.segment<3>(0)`，但此时状态向量尚未完成有效初始化。
- crossing trigger 是 one-shot，且当前默认按时间触发，没有和控制栈 ready 状态绑定，因此早触发会直接踩到这个时序问题。

### 做了哪些修改和尝试
- 目前已确定下一步修复策略：给 `mpc_node_complete` 增加 crossing request defer / pending 机制，而不是只调大 delay。

### 当前结果如何，是否成功
- 稳定性和 cold-start 验证链路已成功。
- 复杂场景入口“已能启动 world + 触发 crossing”，但 crossing 切换阶段仍存在崩溃，尚未完成。

### 下一步准备做什么
- 修改 `src/dog2_mpc/src/mpc_node_complete.cpp`：
  - crossing 请求早到时先缓存；
  - 状态 ready 后再切换 crossing 模式；
  - 防止 one-shot trigger 造成空状态崩溃。
- 重新构建 `dog2_mpc` 并复测 `crossing_trial.launch.py`。

## 2026-04-24 22:54 CST

### 现在想实现什么
- 让 crossing 模式触发对启动时序鲁棒，不再因为 one-shot `/enable_crossing` 早于状态初始化而把 `mpc_node_complete` 直接打崩。

### 准备用什么方法实现
- 在 `src/dog2_mpc/src/mpc_node_complete.cpp` 内增加一个最小防御性机制：
  - 新增 `hasCrossingState()` 判定状态是否 ready；
  - 新增 `enableCrossingMode()` 把原 crossing 初始化逻辑收拢到一个安全入口；
  - crossing callback 若收到请求但状态未 ready，则只记录 `pending_crossing_request_`；
  - 在 `controlLoop()` 中，当 `odom + joint` 都 ready 后再实际切进 crossing。

### 为什么选这个方法，而不是其他方法
- 这是直接修根因的办法，能够覆盖不同机器负载和不同冷启动速度。
- 如果只调大 `crossing_delay_sec`，以后启动更慢时还会再次出现同类问题。
- 该改动局限在 `dog2_mpc` 节点内部，不影响现有 launch 接口和上层调用方式。

### 当前具体做了哪些操作
- 已在本地临时副本中完成 `mpc_node_complete.cpp` 补丁：
  - 提取 crossing 初始化逻辑为 `enableCrossingMode()`
  - 新增 `hasCrossingState()`
  - 新增 `pending_crossing_request_`
  - 修改 `enableCrossingCallback()`
  - 修改 `controlLoop()`
- 正在把补丁同步回远端工作区。

### 过程中遇到了什么问题
- crossing 逻辑的风险点不在 launch，而在控制节点内部直接读取状态向量的时机。

### 问题的原因判断是什么
- 之前的实现默认假设 crossing 请求一定晚于 `odom` 和 `joint_states` 初始化完成，这个假设在冷启动集成环境里不成立。

### 做了哪些修改和尝试
- 这次没有继续调大 delay，而是改成内部延迟激活。

### 当前结果如何，是否成功
- 代码层补丁已完成。
- 远端构建和障碍场景复测还在进行前，结果待确认。

### 下一步准备做什么
- 同步补丁到远端；
- 构建 `dog2_mpc`；
- 复测 `crossing_trial.launch.py`，确认不再崩溃并观察 crossing 是否真正进入。

## 2026-04-24 22:57 CST

### 现在想实现什么
- 收口本轮改动，确认：
  - crossing 入口已从“会崩”变成“可冷启动进入”；
  - 平地研究栈 smoke 没被回归破坏；
  - `gz_gain_setter` 的日志噪声进一步下降。

### 准备用什么方法实现
- 对 `dog2_motion_control`、`dog2_mpc`、`dog2_bringup` 做一次联合构建；
- 用落盘日志方式分别复测：
  - `crossing_trial.launch.py`
  - `smoke_test.launch.py`
- 通过 grep/result file 提取关键结果，而不是再人工滚长日志。

### 为什么选这个方法，而不是其他方法
- 这能同时验证“功能进入”“不崩溃”“回归没坏”三件事。
- 用日志落盘能避免被 `mpc_node_complete` 的高频调试打印干扰判断。

### 当前具体做了哪些操作
- 远端执行：
  - `colcon build --packages-select dog2_motion_control dog2_mpc dog2_bringup --symlink-install`
  - `timeout 12 ros2 launch dog2_bringup crossing_trial.launch.py use_gui:=false rviz:=false teleop:=false crossing_delay_sec:=3.0`
  - `timeout 40 ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true ros_domain_id:=46 result_file:=/tmp/dog2_smoke_result_after_mpc.txt`
- 同时对 `gz_gain_setter.py` 做了第二轮小修：
  - 把“参数未声明”从反复 WARN 改成静默重试/低噪声等待；
  - 保留最终成功/失败的明确结论。

### 过程中遇到了什么问题
- `crossing_trial` 虽然已不再崩溃，但 `gz_ros2_control` 在参数尚未声明时仍会从服务端打一条底层 WARN，这一条不在 `gz_gain_setter` 节点内部。
- `mpc_node_complete` 自身调试打印非常高频，导致原始 launch 日志噪声很大。

### 问题的原因判断是什么
- `gz_gain_setter` 只能控制自身日志级别，无法完全消除远端参数服务端在“过早请求”时打印的底层告警。
- MPC 节点源码里保留了大量 QP 调试输出，当前不适合在这一轮顺手大改。

### 做了哪些修改和尝试
- 已验证 crossing 请求现在会先 defer，再在状态 ready 后切入 crossing：
  - `Crossing request received before state was ready; deferring mode switch`
  - `Crossing mode ENABLED`
  - 后续持续出现 `mode=CROSSING`
- 已验证平地 smoke 回归通过，结果文件如下：
  - `STAGE: stage=stand x=0.015 y=0.010 z=0.162 yaw=-0.026`
  - `STAGE: stage=forward x=0.531 y=-0.017 z=0.085 yaw=2.628`
  - `STAGE: stage=turn x=0.564 y=0.381 z=0.156 yaw=-2.001`
  - `PASS: turn_yaw_delta=3.122 z=0.103`

### 当前结果如何，是否成功
- 成功。
- 本轮目标中的关键工程入口已具备：
  - 研究栈 cold-start smoke 可通过；
  - crossing 复杂场景 world 可启动；
  - one-shot crossing trigger 不再导致 `mpc_node_complete` 崩溃；
  - 节点能实际进入 `mode=CROSSING`；
  - `gz_gain_setter` 已去掉之前的退出异常噪声，并减少自身重试告警。

### 下一步准备做什么
- 如果继续推进，下一轮应优先做：
  1. 给 `crossing_trial` 增加明确成功/失败判据，而不只是“进入模式”；
  2. 抽出复杂场景 smoke/check 脚本，形成真正可回归测试；
  3. 视需要降低 `dog2_mpc` 的高频调试日志，只保留阶段性摘要。

## 本轮完整总结

### 最终实现了什么
- 把 `MPC + WBC` Gazebo 研究栈从“平地能跑”推进到“具备复杂场景入口且更稳”：
  - 平地 cold-start smoke 仍可通过；
  - 新增了小障碍 world 和 crossing 启动入口；
  - 修掉了 crossing 过早触发导致 `mpc_node_complete` 直接崩溃的问题；
  - 修掉了 `gz_gain_setter` 结束时的异常退出噪声，并降低了其自身启动期重试噪声。

### 关键改动有哪些
- `src/dog2_bringup/launch/smoke_test.launch.py`
  - 增加 domain/world/result_file 等可配置入口，提升复现性。
- `src/dog2_bringup/dog2_bringup/smoke_check.py`
  - 增加研究栈关键 topic 检查，尤其是 `contact_phase` 与 `/effort_controller/commands`。
- `src/dog2_bringup/dog2_bringup/crossing_trigger.py`
  - 新增 one-shot crossing 触发节点。
- `src/dog2_bringup/launch/crossing_trial.launch.py`
  - 新增复杂场景 launch 入口。
- `src/dog2_bringup/worlds/step_block.sdf`
  - 新增小障碍/台阶 world。
- `src/dog2_bringup/test/test_research_stack_files.py`
  - 新增复杂场景入口相关文件测试。
- `src/dog2_motion_control/dog2_motion_control/gz_gain_setter.py`
  - 修复退出路径；
  - 降低参数尚未声明时的自身告警噪声。
- `src/dog2_mpc/src/mpc_node_complete.cpp`
  - 增加 crossing request defer / pending 机制，等待状态 ready 后再切换 crossing。
- `src/dog2_bringup/README.md`
- `src/dog2_bringup/doc/TROUBLESHOOTING.md`
  - 更新复现命令和排障说明。

### 验证怎么做的
- 构建：
  - `colcon build --packages-select dog2_motion_control dog2_mpc dog2_bringup --symlink-install`
- 平地 cold-start：
  - `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true ros_domain_id:=46 result_file:=/tmp/dog2_smoke_result_after_mpc.txt`
- crossing 入口：
  - `ros2 launch dog2_bringup crossing_trial.launch.py use_gui:=false rviz:=false teleop:=false crossing_delay_sec:=3.0`
- 结果：
  - smoke 通过；
  - crossing 不再崩溃，并进入 `mode=CROSSING`。

### 还剩什么问题
- `gz_ros2_control` 在参数尚未声明的极早期，仍可能从服务端打印一条底层 WARN；这不是 `gz_gain_setter` 节点自身的退出异常了，但还不算完全无噪声。
- `crossing_trial` 当前验证的是“能进入 crossing 模式且稳定运行”，还不是“完成障碍跨越并自动判定成功”。
- `dog2_mpc` 调试日志过于密集，不利于长期回归测试观察。

### 建议的下一步是什么
- 给 crossing 入口补一套结构化 check：
  - 触发成功；
  - 进入 crossing；
  - 保持一定时长不崩；
  - 关键 topic 持续新鲜；
  - 若可行，再增加几何/位姿级成功判据。

## 2026-04-24 23:47:47 CST

### 现在想实现什么
- 把当前 crossing 从“能进入模式”推进到“真正走 active controller crossing 路径，并且对窗型障碍具备 PASS/FAIL 自动验证”。
- 同时把 rails 在 crossing 里的参与从隐含状态，变成可观测、可验证的实际控制输出。

### 准备用什么方法实现
- 不再让 `mpc_node_complete.cpp` 继续维护一套节点侧简化 crossing reference，而是直接初始化并启用 `mpc_controller.cpp` 里已经存在的 crossing state machine / rail-window constraints / stage target 逻辑。
- 通过 `/dog2/mpc/crossing_state` 发布 crossing stage，让 `wbc_node_complete.cpp` 根据 stage 切换 `lf/lh/rh/rf` 的 `ELBOW/KNEE` 构型。
- 在 bringup 下新增真实窗型 world、`crossing_check.py` 和 `window_crossing_test.launch.py`，形成单命令 PASS/FAIL 回归入口。

### 为什么用这个方法
- 这是最小改动路径，因为 richer crossing 逻辑本来就已经在 `mpc_controller.cpp` 里，只是 active node 没把它接上。
- 如果继续沿节点侧 `trajectory_generator.cpp` 那条粗 placeholder 路径堆补丁，会变成两套 crossing 逻辑并存，后续更难维护。
- 通过 stage topic 驱动 WBC 切腿型，能在不改 URDF 的前提下，让 `lf_rail_joint/lh_rail_joint/rh_rail_joint/rf_rail_joint` 和 3R 关节的协同更接近最终机构形态。

### 当前具体做了哪些操作
- 审计了 active path，确认研究栈实际运行链路是：
  - `mpc_node_complete -> wbc_node_complete -> wbc_effort_mux -> /effort_controller/commands`
- 对比了 `mpc_node_complete.cpp` 和 `mpc_controller.cpp`：
  - 确认 controller 内已经有 crossing state machine、stage-specific sliding constraints、rail-window constraints；
  - 确认 node 侧此前没有调用 `initializeCrossing(...)`，导致 richer crossing path 未被实际启用。
- 已修改本地补丁副本：
  - `src/dog2_mpc/src/mpc_node_complete.cpp`
  - `src/dog2_mpc/src/mpc_controller.cpp`
  - `src/dog2_wbc/src/wbc_node_complete.cpp`
  - `src/dog2_bringup/launch/control_stack.launch.py`
  - `src/dog2_bringup/launch/system.launch.py`
  - `src/dog2_bringup/launch/crossing_trial.launch.py`
  - `src/dog2_bringup/setup.py`
  - `src/dog2_bringup/test/test_research_stack_files.py`
- 已新增：
  - `src/dog2_bringup/worlds/window_frame.sdf`
  - `src/dog2_bringup/launch/window_crossing_test.launch.py`
  - `src/dog2_bringup/dog2_bringup/crossing_check.py`

### 过程中遇到了什么问题
- 发现 `mpc_controller.cpp` 里的 `addRailWindowConstraints(...)` 使用了错误的优化变量偏移：
  - 用的是 `k * nx`
  - 但优化向量实际布局是 interleaved 的 `[x_k, u_k]`
- 发现本机直接可写路径和远端 `ssh dell` 看到的绝对路径不一致：
  - 远端是 `~/aperfect/carbot_ws`
  - 本地可直接访问的是 `/home/dell/carbot_ws`

### 问题的原因判断是什么
- `addRailWindowConstraints(...)` 之前没暴露，是因为 richer crossing path 一直没有真正启用；一旦接通 controller crossing，这个索引错误就会把 rail/window 约束绑错变量。
- 路径差异应该是当前环境对工作区做了本地映射，不能直接假设 `/home/dell/aperfect/carbot_ws` 在本地 shell 下存在。

### 做了哪些修改和尝试
- 在 `mpc_node_complete.cpp` 中：
  - 新增 window 参数；
  - crossing 启用时改为调用 `mpc_controller_->initializeCrossing(...)`；
  - 发布 `/dog2/mpc/crossing_state`；
  - crossing 模式下不再走旧的 coarse `generateCrossingTrajectory(...)` 主路径；
  - 让 rail velocity 从 `JointState` 更新进入 controller；
  - 对 walking 之外的模式移除基于 trot 相位的摆动腿清零逻辑。
- 在 `mpc_controller.cpp` 中：
  - 修复 `rail+window` 约束对 interleaved 优化变量的索引错误。
- 在 `wbc_node_complete.cpp` 中：
  - 新增 crossing state 订阅；
  - 根据 stage 切换前腿/后腿的 `ELBOW/KNEE` 构型。
- 在 bringup 中：
  - 把 `crossing_trial` 默认 world 从 `step_block.sdf` 换成 `window_frame.sdf`；
  - 新增 `window_crossing_test.launch.py`；
  - 新增 `crossing_check.py` 以验证 crossing trigger、stage、rail motion、odom 穿越和关键 topic 新鲜度。

### 当前结果如何，是否成功
- 补丁已写完到本地临时副本，逻辑层面已经把 crossing 主路径、WBC stage 消费、窗型 world 和 PASS/FAIL 检查都接起来了。
- 还没进入最终成功判定；下一步是把这些补丁同步回远端工作区并实际 `colcon build + ros2 launch` 验证。

### 下一步准备做什么
- 把当前补丁同步到远端 `~/aperfect/carbot_ws`。
- 先跑 `colcon build --packages-select dog2_mpc dog2_wbc dog2_bringup --symlink-install`。
- 构建通过后，依次跑：
  - 平地 smoke 回归；
  - `window_crossing_test.launch.py`；
  - 如失败，继续按日志修编译或运行时问题，直到 crossing 至少能输出结构化 PASS/FAIL 结果。

## 2026-04-24 23:58:56 CST

### 现在想实现什么
- 让窗型 crossing 至少完成一次结构化验证闭环，并把当前失败形态明确落到结果文件和汇报里。
- 同时修掉 `crossing_check` 已经写出 PASS/FAIL 但 launch 不自动收尾的问题。

### 准备用什么方法实现
- 先不再盲目加大 crossing 约束，而是根据真实运行日志收敛时序：
  - 先 normal walking 接近窗前；
  - 再真正 `initializeCrossing(...)`。
- 对 `crossing_check.py` 强制 `SystemExit`，确保 result file 落地后进程真的退出。
- 补 README / troubleshooting，让后续复现实验的人能直接看到：
  - 推荐命令；
  - 结果文件位置；
  - 当前已知失败模式。

### 为什么这样做
- 第一轮真实验证已经说明：把 controller crossing 从一开始就启用，会在机器人还没走到窗前时触发大量 `OSQP status 3/7`，机身几乎不前进。
- 但我们已经拿到了很关键的信息：
  - `/dog2/mpc/crossing_state` 在工作；
  - rail 位移已经发生；
  - 失败是“机身没到窗前 + 控制链路后段变 stale”，不是 launch 根本没起来。
- 所以最合理的最小修正是先做时序解耦，再保留当前验证入口继续观察。

### 当前具体做了哪些操作
- 远端构建：
  - `colcon build --packages-select dog2_mpc dog2_wbc dog2_bringup --symlink-install`
  - 构建通过。
- 平地回归：
  - `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true ros_domain_id:=48 result_file:=/tmp/dog2_smoke_result_window_dev.txt`
  - 结果通过。
- 第一轮窗型 crossing：
  - `ros2 launch dog2_bringup window_crossing_test.launch.py ros_domain_id:=49 result_file:=/tmp/dog2_window_crossing_result_dev.txt`
  - 已得到 FAIL 结果文件。
- 根据 FAIL 结果，又修改了：
  - `src/dog2_mpc/src/mpc_node_complete.cpp`
    - crossing 请求后先 `PRE_APPROACH` 正常走到窗前，再初始化 controller crossing。
  - `src/dog2_bringup/dog2_bringup/crossing_check.py`
    - 写结果后强制退出。
  - `src/dog2_bringup/README.md`
  - `src/dog2_bringup/doc/TROUBLESHOOTING.md`

### 过程中遇到了什么问题
- 第一轮 crossing 的真实 FAIL：
  - `FAIL: body has not cleared window x region; missing=/dog2/mpc/foot_forces,/dog2/wbc/joint_effort_command,/dog2/wbc/rail_effort_command,/effort_controller/commands; stage=CROSSING:APPROACH; max_x=0.077; max_rail_delta=0.111`
- 第二轮复测又暴露出另一个不稳定点：
  - `gz_pose_to_odom` 持续打印 `dynamic_pose TF received but no transform matched model/base hints`
  - `mpc_node_complete` 一直 `Waiting for state...`
  - 导致 crossing trigger 发出后也没有真正进入控制闭环。

### 问题的原因判断是什么
- 第一轮 FAIL 的主要原因是 crossing 约束加载得过早，机器人还没走到窗前就进入重约束 QP，导致 MPC 解频繁失败。
- 第二轮失败更偏系统时序/桥接不稳定：
  - `gz_pose_to_odom` 没有从 dynamic pose 流里匹配到 `dog2/base_link`
  - 因而 `/odom` 缺失，研究栈状态机根本没法进入。

### 做了哪些修改和尝试
- 已把 crossing 控制时序改成：
  - `CROSSING:PRE_APPROACH`
  - 接近到 `window_x - activation_distance`
  - 再 `initializeCrossing(...)`
- 已让 `crossing_check.py` 在写完结果后直接退出，以便 launch 能基于 `OnProcessExit` 收尾。
- 已把当前推荐的 PASS/FAIL 命令和已知故障模式写进文档。

### 当前结果如何，是否成功
- 部分成功。
- 本轮已经形成了完整的工程闭环：
  - 代码修改已落远端；
  - 构建通过；
  - 平地 smoke 回归通过；
  - 窗型 crossing 已经能输出结构化 FAIL 结果，不再只是“人眼观察”。
- 但“窗型障碍跨越成功”这个最终目标，本轮还没有达成；当前卡点是：
  - crossing controller 约束时序仍需要继续收敛；
  - world/bridge 下的 `odom` 链偶发不稳定。

### 下一步准备做什么
- 把 `PRE_APPROACH` 版本再做一次干净复测，优先确认：
  - 是否能稳定拿到 `/dog2/state_estimation/odom`
  - 是否能在接近窗前后再进入 `CROSSING:APPROACH`
  - 是否能把 `max_x` 推到 `window_x` 附近
- 如果仍然卡在 QP，下一步最小动作应是：
  - 继续放宽 crossing 初段 rail/stage 约束；
  - 或把 crossing 真正启用点进一步后移，避免远离窗框时就加载 506 条约束。

## 2026-04-25 00:46:12 CST

### 现在想实现什么
- 继续收敛 crossing 启用时序和 `/odom` 相关观测链，优先把 `window_crossing_test` 从 `stage=UNKNOWN` 推进到真正的 `PRE_APPROACH/APPROACH`，并把 `max_x` 从 0.077 往 `window_x=1.55` 方向推。

### 准备用什么方法实现
- 先把 `gz_pose_to_odom` 的“空 TFMessage 噪声”和真实 frame 匹配问题分开看。
- 再把 crossing trigger 从“固定延时发射”改成“研究栈 ready 后再发”。
- 最后检查 `PRE_APPROACH` 本身有没有把 walking gait 真正推进起来，而不是只在参考轨迹层看起来像 walking。

### 为什么这样做
- 新一轮诊断表明，之前把锅完全甩给 `/odom` 不准确：
  - `/dog2/dynamic_pose_tf` 实际上能收到带 `dog2` 和 `base_link` 的 transform；
  - `/odom` fallback 也能真实发布；
  - 真正的问题是启动早期会夹杂空 `TFMessage`，以及 crossing 在 stack 没 ready 前被触发。
- 此外，`PRE_APPROACH` 如果 gait phase 不推进，就会把接触节律冻住，机器人很难真正往窗前走。

### 当前具体做了哪些操作
- 给 `gz_pose_to_odom.py` 增加了 transform 候选诊断，并确认：
  - 非空 `dynamic_pose_tf` 里确实有 `child_frame_id: dog2` 和 `child_frame_id: base_link`；
  - 之前的 `candidates=[]` 来自空 `TFMessage`，不是 frame 命名彻底错误。
- 修改 `gz_pose_to_odom.py`：
  - 对空 `TFMessage` 直接忽略，不再把它当成 frame 匹配失败反复告警。
- 修改 `crossing_trigger.py`：
  - 新增 readiness gating；
  - 默认等待 `/dog2/state_estimation/odom` 和 `/joint_states` 都变成新鲜流之后，再在 `delay_sec` 满足时发布 `/enable_crossing`。
- 复测 `window_crossing_test`：
  - 一次长测里已经确认 `gz_ros2_control`、`controller_manager`、`joint_state_broadcaster` 最终能起来；
  - crossing_check 不再停在“瞬时 early trigger”，而是进入 `CROSSING:PRE_APPROACH`；
  - 结果文件为：
    - `FAIL: body has not cleared window x region; stage=CROSSING:PRE_APPROACH; max_x=0.295; max_rail_delta=0.111`
- 进一步检查 `mpc_node_complete.cpp` 后发现：
  - `CROSSING:PRE_APPROACH` 使用 walking trajectory；
  - 但 `gait_phase_` 只在 `Mode::WALKING` 分支里推进；
  - crossing pre-approach 分支没有推进 gait，相当于把 contact phase 冻住。
- 已修复 `mpc_node_complete.cpp`：
  - 抽出 `advanceGaitPhase()`；
  - 在 `Mode::WALKING` 和 `CROSSING:PRE_APPROACH` 都推进 gait phase。
- 构建：
  - `colcon build --packages-select dog2_motion_control dog2_bringup --symlink-install`
  - `colcon build --packages-select dog2_mpc --symlink-install`
  - 都通过。

### 过程中遇到了什么问题
- 启动稳定性有明显随机性：
  - 同样的 `window_crossing_test`，有一轮能看到 `gz_ros2_control -> controller_manager -> joint_state_broadcaster` 正常起来；
  - 另一轮又会长时间停在 `Waiting for state...`，`crossing_trigger` 一直报告 `odom_age=none joint_state_age=none`。
- 这说明问题不只是 crossing 控制律，bringup 时序本身还存在非确定性。

### 问题的原因判断是什么
- `/odom` 本身不是主阻塞项，至少不是唯一阻塞项：
  - fallback adapter 是能发布 odom 的；
  - 真正让 crossing_check 停在 `UNKNOWN` 的，更多是 stack ready 时序。
- `PRE_APPROACH` 的 `gait_phase_` 冻结是一个直接限制前进距离的代码级 bug，这很可能是 `max_x` 长期卡在 0.295 左右的主因之一。
- 但 bringup 还有额外的随机性：
  - `gz_ros2_control` 有时能正常连接 `robot_state_publisher` 并加载 controller_manager；
  - 有时又会迟迟没有后续初始化日志。

### 做了哪些修改和尝试
- 保留：
  - `crossing_trigger` 的 readiness gating；
  - `gz_pose_to_odom` 的空包忽略。
- 试过把 `joint_state_broadcaster` 改成等 `gz_gain_setter` 退出后再起；
  - 结果连 flat smoke 也被拖慢；
  - 已经撤回，没有保留这个改法。
- 当前保留的有效改动是：
  - crossing trigger 等 stack ready；
  - pre-approach gait phase 真正推进。

### 当前结果如何，是否成功
- 部分成功。
- 和上一轮相比，已经明确向前推进了一步：
  - `window_crossing_test` 不再只停在 `stage=UNKNOWN`；
  - 至少有一轮已经稳定进入 `CROSSING:PRE_APPROACH`；
  - `max_x` 从 0.077 提升到了 0.295；
  - `rail` 运动依旧存在，`max_rail_delta=0.111`。
- 但还没到 `APPROACH`/controller crossing 初始化，更没有接近 `window_x=1.55`。
- 最新一轮在加入 gait-phase 修复后，bringup 又出现一次随机失稳，说明还需要继续做启动时序收敛。

### 下一步准备做什么
- 继续围绕“非确定性 bringup”做最小诊断：
  - 重点盯 `gz_ros2_control` 从 `create` 完成到 `controller_manager` 服务可用之间的时延和随机性；
  - 需要的话，把关键日志落到单独文件，避免被 MPC QP 调试输出淹没。
- 在 startup 重新稳定之后，再复测 gait-phase 修复版的 `window_crossing_test`，确认：
  - `max_x` 是否能继续高于 0.295；
  - `crossing_state` 是否能从 `PRE_APPROACH` 进一步推进。
- 如果 `PRE_APPROACH` 仍然明显推进不足，下一步优先考虑：
  - 调整 `crossing_approach_speed`；
  - 或在 crossing request 生效前增加一个更明确的站稳/起步条件，而不是仅靠 topic freshness。

## 

### 现在想实现什么
- 收敛  cold start 随机性，先把  和  启动做稳定，再复测 gait-phase 修复后  是否继续上升。

### 准备用什么方法实现
- 先排查并清理旧 Dog2 Gazebo/bridge 进程；再把启动前清理固化到  和 flat smoke 入口，避免旧 world 抢占 Gazebo transport 或残留 topic 造成假启动。

### 为什么这样做
- 远端发现一天前的  bridge 和一小时前的  Gazebo server 仍在运行；这类残留会直接影响 、dynamic pose bridge、world 匹配和  初始化，属于当前 startup 随机性的高概率原因。

### 当前具体做了哪些操作
- 定向检查了 Dog2/Gazebo 相关进程，确认存在旧 flat-ground bridge 和旧 window-frame Gazebo server。
- 手动 kill 了这些明确属于 Dog2 仿真的残留进程。
- 修改 ：启动前先清理  相关残留，清理完成后再启动 crossing trial 和 crossing check。
- 修改 ：启动前先清理  相关残留，减少平地回归被旧进程污染的概率。

### 过程中遇到了什么问题
- 清理动作如果直接放在 launch action 列表前面，存在清理进程快速退出、event handler 尚未注册的竞态。

### 问题的原因判断是什么
-  是异步 launch action；短命令可能在后续 handler 注册前退出，因此必须先注册 ，最后启动清理进程。

### 做了哪些修改和尝试
- 把  和  都放到  action 之前注册。
- 保留退出后的清理动作，形成启动前和退出后双保险。

### 当前结果如何，是否成功
- 代码修改已落远端；还未构建和复测。
- 当前没有 Dog2/Gazebo 残留进程，下一步可以做干净 cold start。

### 下一步准备做什么
- 构建 ，然后先跑 flat smoke 回归，再跑 ，重点观察  是否稳定、 是否稳定出现、 是否超过上一轮 0.295。

## 2026-04-25 01:05:34 CST

### 现在想实现什么
- 继续收敛 `dog2_window_frame` 和 flat smoke 的 cold start 随机性，先把 `/odom`、`gz_ros2_control`、`controller_manager`、`joint_state_broadcaster` 的启动链做稳定，再复测 gait-phase 修复后 `max_x` 能否继续从 0.295 往 `window_x=1.55` 推进。

### 准备用什么方法实现
- 先消除旧 Dog2 Gazebo/bridge 残留导致的 world/topic 污染。
- 再减少 `spawn_entity -> gz_ros2_control -> robot_state_publisher/get_parameters -> controller_manager` 之间的竞争。
- 最后重新运行平地 smoke 和 window crossing cold start。

### 为什么这样做
- 远端曾残留一天前的 `dog2_flat_ground` bridge 和一小时前的 `window_frame.sdf` Gazebo server；这种残留会污染 Gazebo transport、`/clock` 和 dynamic pose bridge。
- 新一轮 flat smoke 失败时，日志显示 `gz_ros2_control` 已开始请求 `robot_description`，但 `robot_state_publisher` 报参数服务响应超时，随后 `controller_manager` 没有真正进入可用状态。
- 因此当前主要问题不是 crossing 算法，而是 cold start 时序竞争和残留清理不可靠。

### 当前具体做了哪些操作
- 手动清理了明确属于 Dog2 仿真的残留进程。
- 修改 `window_crossing_test.launch.py`：启动前执行 `dog2_window_frame/window_frame.sdf//world/dog2_window_frame` 相关残留清理，清理完成后再启动 crossing trial 与 crossing check；退出后仍保留清理动作。
- 修改 `smoke_test.launch.py`：启动前执行 `dog2_flat_ground/flat_ground.sdf//world/dog2_flat_ground` 相关残留清理，减少 flat smoke 被旧进程污染的概率；退出后仍保留清理动作。
- 修正 preclean 命令的自匹配问题：把 `pkill -f dog2_...` 改成 `[d]og2_...`，把 `.sdf` 改成 `[.]sdf`，避免清理命令 kill 自己。
- 修改 `effort_research_sim.launch.py`：新增 `spawn_delay_sec`，默认延迟 2 秒再执行 `spawn_entity`，让 Gazebo server 和 `robot_state_publisher` 先稳定。
- 修改 `effort_research_sim.launch.py`：`gz_gain_setter` 不再和 `joint_state_broadcaster` 同时在 spawn 后启动，而是等 `joint_state_broadcaster` spawner 退出后再运行，避免它在 `gz_ros2_control` 初始化阶段反复打 `/set_parameters`。

### 过程中遇到了什么问题
- 第一次写 `WORK_REPORT.md` 时未转义 Markdown 反引号，shell 把若干技术词当命令执行，导致上一段记录里有空白字段。
- 第一次 preclean 使用普通 `pkill -f`，匹配到了自己的 shell 命令，launch 里出现 `exit code -15` 噪声。
- flat smoke 仍失败了一轮，具体失败点是 `controller_manager` 只在图里出现了不稳定服务痕迹，spawner 和 direct service call 都没有拿到可用响应。

### 问题的原因判断是什么
- 旧 world/Gazebo server 残留是 startup 随机性的外部污染源。
- `spawn_entity` 太早会让 Gazebo 插件和 RSP 参数服务在系统还没完全稳定时竞争 `robot_description`。
- `gz_gain_setter` 早启动会向尚未完成初始化的 `/gz_ros2_control/set_parameters` 反复请求，虽然不是唯一根因，但会放大初始化阶段噪声。

### 做了哪些修改和尝试
- 已把 preclean/cleanup 命令改成不会自匹配的正则。
- 已把 launch 顺序改成先注册 `OnProcessExit` handler，再启动 preclean，避免短命令退出过快导致 handler 挂不上。
- 已增加 `spawn_delay_sec` 和延后 `gz_gain_setter` 的最小改动。
- 已重新构建 `dog2_bringup`，构建通过。

### 当前结果如何，是否成功
- 代码修改已落远端，`dog2_bringup` 构建通过。
- 旧进程已清理干净。
- flat smoke 在修正前失败，失败现象和原因已记录；修正后的 smoke 还需要立即复测。

### 下一步准备做什么
- 重新跑 flat smoke，确认 preclean 不再自杀、`gz_ros2_control` 能稳定加载 `controller_manager` 和 `joint_state_broadcaster`。
- 如果 flat smoke 通过，立刻跑 `window_crossing_test`，观察 `/odom` 是否稳定、crossing 是否进入 `PRE_APPROACH/APPROACH`、`max_x` 是否超过 0.295。
- 如果仍卡 controller startup，继续沿 `robot_state_publisher/get_parameters` 与 `gz_ros2_control` 初始化日志排查，不转向调 crossing 参数。

## 2026-04-25 01:13:38 CST

### 现在想实现什么
- 在 startup 已稳定的基础上，把 window crossing 从长期卡在 `PRE_APPROACH` 推进到能初始化真正的 crossing 阶段，并继续把 `max_x` 往 `window_x=1.55` 附近推。

### 准备用什么方法实现
- 不改 URDF，不绕开 MPC/WBC，只把 crossing 初段门槛和接近速度做成 launch 参数。
- 对 window 场景使用更早的 crossing activation，让控制器在机器人已经能稳定到达的 `x≈1.05` 左右进入真正 crossing，而不是要求先在 `PRE_APPROACH` 硬走到 `x=1.30`。

### 为什么这样做
- 最新 window cold start 表明 `/odom` 链路、`gz_ros2_control`、`controller_manager` 和 rail 运动都已工作；`max_x` 从 0.295 提升到 1.146。
- 失败点变成了逻辑门槛：默认 `crossing_activation_distance=0.25` 等价于 `window_x - 0.25 = 1.30` 才初始化 crossing，但机器人在窗前约 1.146 处停住。
- 直接调大 window 场景的 activation distance 是最小可行修改，比改状态机结构或绕开控制栈风险低。

### 当前具体做了哪些操作
- 修改 `control_stack.launch.py`：新增并传入 `crossing_activation_distance`、`crossing_approach_speed` 到 `mpc_node_complete`。
- 修改 `system.launch.py`：把这两个参数从上层 launch 继续透传到 `control_stack.launch.py`。
- 修改 `crossing_trial.launch.py`：window crossing 默认 `crossing_activation_distance=0.50`、`crossing_approach_speed=0.20`。
- 修改 `window_crossing_test.launch.py`：同样暴露并默认使用上述 window 参数。
- 修改 `window_crossing_test.launch.py`：preclean/cleanup 也清理历史 orphan `crossing_check` 和 `crossing_trigger`，避免多轮测试后留下 PPID=1 的验证节点。
- 清理了当前未完成的 window run 和历史 orphan 验证节点。
- 重新构建 `dog2_bringup`，构建通过。

### 过程中遇到了什么问题
- 原 window run 的启动命令所在 SSH session 异常退出，但远端 launch 仍继续运行；通过进程列表确认后手动终止。
- 发现多轮历史 orphan `crossing_check/crossing_trigger` 残留，说明仅清 world/Gazebo 不够。

### 问题的原因判断是什么
- SSH session 退出不等于 ROS launch 一定退出；如果外层命令被断开，内部 launch 可能变成孤儿或继续由 timeout 管理。
- 验证节点残留虽然通常被 ROS_DOMAIN_ID 隔离，但会污染进程状态和后续人工判断，应该在 window 测试入口主动清理。

### 做了哪些修改和尝试
- 参数链路已贯通，flat smoke 默认不变，window crossing 默认提前切入。
- 已把 orphan 验证节点纳入 window preclean/cleanup。

### 当前结果如何，是否成功
- 部分成功。
- flat smoke 已 PASS。
- window crossing 已确认 `max_x=1.146`，显著高于上一轮 0.295，但仍卡在 `PRE_APPROACH`。
- 新参数改动已构建通过，尚未复测。

### 下一步准备做什么
- 重新运行 `window_crossing_test`，确认是否能在 `x≈1.05` 初始化 crossing。
- 验证重点：是否出现 `Initialized controller crossing`，stage 是否离开 `CROSSING:PRE_APPROACH`，`max_x` 是否继续接近或超过 `window_x=1.55`，rail delta 是否继续有效。

## 2026-04-25 01:37 CST - crossing 启动随机性与初段门控收敛

### 现在想实现什么
- 继续收敛 window crossing 的启动随机性，把 `/odom` 在 `dog2_window_frame` world 下的稳定性做实。
- 在 gait-phase 修复已经让 `PRE_APPROACH` 能前进的基础上，优先把 `max_x` 推到 `window_x` 附近，再进入真正 crossing 阶段。

### 准备用什么方法实现
- 先保证每次 window/flat 测试前主动清理同类旧 Gazebo world 和孤儿验证节点，降低 cold start 假阳性和随机失败。
- 不再仅依赖 `crossing_trigger` 固定延时触发，而是在触发 crossing 前用现有 `/cmd_vel -> MPC/WBC` 路径先把机器人送到窗前阈值，再发 `/enable_crossing`。

### 为什么这样做
- 直接提前 `crossing_activation_distance` 会让控制器更早进入 `CROSSING:PRE_APPROACH`，但该阶段姿态更低、推进更不稳定，实测多次 `max_x` 反而下降。
- `/cmd_vel` 预接近仍走现有研究栈，不是绕开 MPC/WBC 的动作播放器；它只改变 crossing 触发时序，让 crossing 状态机在更接近窗口的位置开始工作。

### 当前具体做了哪些操作
- 在 `window_crossing_test.launch.py` 增加 window 场景 preclean，清理旧 `dog2_window_frame`、`window_frame.sdf`、`crossing_check`、`crossing_trigger`。
- 在 `smoke_test.launch.py` 增加 flat 场景 preclean，避免历史 `dog2_flat_ground` 进程污染平地回归。
- 在 `effort_research_sim.launch.py` 增加 `spawn_delay_sec`，并把 `gz_gain_setter` 延后到 `joint_state_broadcaster` spawner 退出后执行，避免 Gazebo 插件初始化期间抢参数服务。
- 在 `crossing_trigger.py` 增加 odom-x 门控和 `/cmd_vel` 预接近能力：达到 `trigger_when_x_ge` 后才发布 `/enable_crossing`，并可自动停止 `/cmd_vel`。
- 重新构建 `dog2_bringup`，构建通过。

### 过程中遇到了什么问题
- 曾发现旧 flat world、window world 和多组 orphan `crossing_check/crossing_trigger` 残留。
- 最新一次带 `/cmd_vel` 门控的 window run 未拿到 `/odom` 和 `/joint_states`，`crossing_trigger` 一直停在 `odom_age=none joint_state_age=none`。

### 问题的原因判断是什么
- 第一类问题来自测试进程残留，已经通过 launch preclean/cleanup 收敛。
- 第二类问题是 Gazebo/ros2_control cold start 仍存在偶发初始化卡住：验证节点没有看到 odom/joint_states，说明不是 crossing 算法阶段失败，而是控制链还没完成启动。

### 做了哪些修改和尝试
- 已验证 flat smoke 在 startup 修复后通过。
- 已验证 window 在 gait-phase 修复和启动清理后曾达到 `max_x=1.146`，rail delta 约 `0.111`，说明 `/odom` 匹配和 rail 控制链可以工作。
- 尝试过 `crossing_activation_distance=0.50/0.85/1.10/1.20` 等提前切入方案，结果都没有稳定优于原来的 `max_x=1.146`，因此改为 `/cmd_vel` 预接近门控。

### 当前结果如何，是否成功
- 部分成功。
- 启动清理、flat smoke、参数链路、`/cmd_vel` 门控实现和构建均已完成。
- window crossing 的最新门控验证被 cold start 初始化卡住阻断，尚未证明能稳定进入 crossing。

### 下一步准备做什么
- 在确认残留进程已清空后，重新运行一次干净 cold start window crossing。
- 如果仍出现 `/odom`/`joint_states` 缺失，继续收敛 `gz_ros2_control`/spawn 时序；如果链路启动成功，则观察 `/cmd_vel` 门控是否能把 `max_x` 推到 `trigger_when_x_ge` 并触发 crossing。

## 2026-04-25 02:04 CST - 修正 crossing rail 方向与非 crossing rail 速度输入

### 现在想实现什么
- 让 crossing 使用当前 Dog2 真实 rail joint 限位和符号，避免状态机给出不可达的 rail 目标。
- 降低 WALKING/PRE_APPROACH 阶段 OSQP 因 rail 速度噪声和硬限位组合导致的 infeasible。

### 准备用什么方法实现
- 只修改 `CrossingStateMachine` 的 rail 约束、目标和完成条件，不动 URDF/xacro/mesh。
- 在 `mpc_node_complete` 中对用于 SRBD 预测的 rail velocity 做阶段化净化：真正进入 initialized crossing 前置零；进入 crossing 后限制幅值并在物理限位处阻止继续越界。
- 将 window 触发策略改成低阈值：先用 `/cmd_vel` 走到已验证可达的 `x≈0.35`，再进入 `CROSSING:PRE_APPROACH`，由 crossing 逻辑继续推进到窗前。

### 为什么这样做
- 代码排查发现状态机仍按旧符号使用 `[-0.111, +0.111, +0.111, -0.111]` 一类目标；但当前真实限位是 `lf/rh: [0, 0.111]`、`lh/rf: [-0.111, 0]`。
- 旧目标会使 `lf_rail_joint`、`lh_rail_joint` 在 crossing 中不可达，只能靠 soft slack 顶住，直接影响 rail 参与越障。
- 最新 window run 已证明 `/odom` 和控制链能启动，但 `/cmd_vel` 预接近在 `max_x=0.523` 后回退，说明 0.82 触发阈值过高。

### 当前具体做了哪些操作
- 修改 `crossing_state_machine.cpp`：
  - rail 约束改成真实 joint 顺序 `[lf_rail_joint, lh_rail_joint, rh_rail_joint, rf_rail_joint]`。
  - 物理限位改成 `[0.0, -0.111, 0.0, -0.111]` 到 `[0.111, 0.0, 0.111, 0.0]`。
  - `BODY_FORWARD_SHIFT` compact-body 目标改成 `[+0.111, -0.111, +0.111, -0.111]`。
  - `checkBodyForwardShiftComplete` 改为按真实正/负对角 rail 伸展判断。
- 修改 `mpc_node_complete.cpp`：
  - 初始化 `velocity_cmd_` 为零。
  - 增加 `sanitizeSlidingVelocities()`，非 initialized crossing 阶段把预测用 rail velocity 置零，initialized crossing 阶段限幅并阻止在限位处继续越界。
- 修改 `crossing_trial.launch.py` 和 `window_crossing_test.launch.py`：
  - `crossing_activation_distance` 默认改为 `1.05`，对应 `activation_x≈0.50`。
  - `crossing_trigger_x_threshold` 默认改为 `0.35`。
  - `pre_crossing_cmd_vel_x` 默认改为 `0.12`。
- 构建 `dog2_mpc` 和 `dog2_bringup`，均通过。

### 过程中遇到了什么问题
- `/cmd_vel` 预接近不是单调稳定前进，上一轮最大只到 `x=0.523`，随后 odom 回到负 x。
- MPC 日志在 `crossing_enabled_=0` 时反复出现 `OSQP solver failed with status 3`。

### 问题的原因判断是什么
- 高触发阈值会让系统一直停留在 WALKING 预接近；一旦行走姿态退化，就永远不会触发 crossing。
- 非 crossing 阶段使用 measured rail velocity 参与硬 rail 限位预测，容易把短时域 QP 推成不可行；这不是 crossing rail 目标问题，而是预测输入和约束的时序问题。

### 做了哪些修改和尝试
- 用真实 rail 限位替换状态机旧符号目标。
- 将触发阈值从 0.82 降到 0.35，把 crossing 初始化点从窗前更远处前移到 `x≈0.50`，降低预接近对稳定行走的依赖。
- 对 rail velocity 输入做阶段化净化，保留 initialized crossing 阶段的真实 rail 动作空间。

### 当前结果如何，是否成功
- 代码修改和构建成功。
- 尚未完成新参数下的 flat smoke 和 window crossing 复测。

### 下一步准备做什么
- 先跑 flat smoke，确认基础站立/前进/转向未回归。
- 再跑 window crossing cold start，重点观察是否能触发 crossing、是否离开 `PRE_APPROACH`、`max_x` 是否超过上一轮 1.146 或至少稳定接近 `window_x`。

## 2026-04-25 02:52 CST - window crossing 收敛结果与剩余阻塞

### 现在想实现什么
- 继续把 window crossing 的 `max_x` 推到 `window_x=1.55` 附近，并让状态机从 `APPROACH` 进入后续 rail compact/body shift 阶段。
- 同时把启动随机性收敛到可重复 cold start。

### 准备用什么方法实现
- 对启动链路：在 window crossing launch 中固定 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，并把测试前/测试后清理从普通 `pkill` 改成 `TERM -> sleep -> KILL`，避免 orphan Gazebo world 干扰下一轮。
- 对控制链路：修正 rail soft bound 数值尺度、放宽 crossing 初段 hard coordination、推迟 controller crossing 初始化到 `x≈1.0`。
- 对状态机：APPROACH 阶段不再要求 rail 回到 neutral 才允许进入下一阶段，避免 rails 已经真实运动后被 guard 卡住。

### 为什么这样做
- `gz_ros2_control` 偶发卡在请求 `robot_description` 后不加载 controller manager；实测旧 orphan `ign gazebo ... window_frame.sdf` 会导致新 world 并行存在，preclean 必须强制清理。
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 的一次验证能让 controller chain 正常启动，避免 FastDDS SHM lock 错误影响服务发现。
- 过早在 `x≈0.50` 初始化 crossing 会让机器人进入 `CROSSING:APPROACH` 后机身高度降到约 `0.057m` 并停在 `max_x≈0.796`；延迟到 `x≈1.0` 初始化能把 `max_x` 推到 `1.489`。

### 当前具体做了哪些操作
- `crossing_trial.launch.py` / `window_crossing_test.launch.py`：
  - 默认 `crossing_activation_distance` 改为 `0.55`，对应 `activation_x=1.00`。
  - 保留 `crossing_trigger_x_threshold=0.35` 和 `/cmd_vel` 预接近。
  - 固定 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
- `window_crossing_test.launch.py` / `smoke_test.launch.py`：
  - preclean/cleanup 改为先 `pkill -TERM`，等待 1 秒，再 `pkill -KILL`。
- `effort_research_sim.launch.py`：
  - 默认 `spawn_delay_sec` 从 `2.0` 提到 `4.0`。
- `mpc_controller.cpp`：
  - crossing soft rail slack 二次权重从 `1e4` 降到 `1e3`。
  - slack 上界从近似无穷改为 `1.0`，避免 OSQP 数值尺度过大。
  - crossing rail 对称/协调 hard guard 放宽到 `epsilon_sym=0.15`、`coord_tolerance=0.25`。
- `research_mpc.yaml`：
  - `slack_linear_weight` 从 `100000.0` 降到 `5000.0`。
- `crossing_state_machine.cpp`：
  - APPROACH 的 rail tracking guard 放开。
  - APPROACH 完成速度阈值从 `0.05` 放宽到 `0.25`。

### 过程中遇到了什么问题
- SSH 曾短时间超时，随后恢复。
- 有一个顽固 orphan `ign gazebo ... window_frame.sdf` 普通 SIGTERM 清不掉，导致后续 cold start 假失败。
- window crossing 的运动仍高度随机：同样的默认参数，一次 best run 达到 `max_x=1.489`，另一次 retry 只到 `max_x=0.612`。
- 延迟初始化后虽然 `max_x` 接近 `window_x`，但仍没有稳定进入 `BODY_FORWARD_SHIFT`，完整跨越没有 PASS。

### 问题的原因判断是什么
- startup 随机性主要来自旧 Gazebo world 残留和 DDS/ros2_control 初始化时序。
- crossing 运动随机性主要来自当前 WBC/MPC 简化模型在低高度下支撑不足：机身容易掉到很低，导致同样的参考轨迹有时能冲到窗前，有时在 PRE_APPROACH 就塌低停住。
- 状态机 guard 之前把 APPROACH 绑定到 rail neutral，与“rails 已经参与并发生位移”的真实过程冲突。

### 做了哪些修改和尝试
- flat smoke 回归通过：`/tmp/dog2_smoke_result_after_rail_fix.txt` 中 PASS，且 OSQP fail 计数为 0。
- window clean run：成功启动、trigger、初始化 crossing，`max_x=0.796`，`max_rail_delta=0.111`，但过早进入 APPROACH 后停住。
- window late-init run：`crossing_activation_distance=0.55`，在 `x=1.005` 初始化 crossing，best `max_x=1.489`、`max_rail_delta=0.112`，已接近 `window_x=1.55`，但未完成 PASS。
- stage guard retry：启动链路稳定，但运动回退到 `max_x=0.612`，说明动作稳定性仍未收敛。

### 当前结果如何，是否成功
- 部分成功。
- `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install` 通过。
- 平地 smoke 已 PASS。
- window crossing 已能稳定形成一条可执行验证入口，并至少一次 cold start 达到 `max_x=1.489`、rail 显著运动。
- 尚未达到完整验收：没有稳定 PASS，没有完成前腿过柱/机身过柱/后腿过柱全阶段。

### 下一步准备做什么
- 先解决 crossing 初段低高度/支撑不足：给 APPROACH/PRE_APPROACH 增加更强的 body height 约束或 WBC 垂向力分配下限。
- 再把 BODY_FORWARD_SHIFT 的 rail compact target 与 stage transition 做成可观测日志，验证 `lf_rail_joint/lh_rail_joint/rh_rail_joint/rf_rail_joint` 是否按 `[+,-,+,-]` compact-body 方向收敛。
- 继续用单命令：
  `ros2 launch dog2_bringup window_crossing_test.launch.py ros_domain_id:=<id> timeout_sec:=190.0 result_file:=/tmp/dog2_window_crossing_result.txt`
  作为回归入口。

## 2026-04-25 02:52 CST - 本轮总结

### 最终实现了什么
- 修正 crossing rail 目标和约束到真实 Dog2 joint 顺序：
  `[lf_rail_joint, lh_rail_joint, rh_rail_joint, rf_rail_joint]`。
- rails 的 compact-body 目标改为 `[+0.111, -0.111, +0.111, -0.111]`，不再使用旧的错误符号。
- 改善 window cold start：强制清理 orphan Gazebo，固定 FastDDS UDPv4，延后 spawn。
- 增强 crossing trigger：先用 `/cmd_vel` 到 `x>=0.35` 再触发，默认在 `x≈1.0` 初始化 controller crossing。

### 关键验证
- 构建：`dog2_mpc`、`dog2_bringup` 均通过。
- 平地 smoke：PASS。
- window best run：trigger 成功，controller crossing 在 `x=1.005` 初始化，`max_x=1.489`，rail delta `0.112`。

### 还剩什么问题
- window crossing 仍未稳定 PASS。
- 状态机尚未稳定推进到 `BODY_FORWARD_SHIFT` 之后。
- 机身高度在 crossing 初段经常降到过低，导致运动随机性大。

### 建议下一步
- 优先加强 crossing PRE_APPROACH/APPROACH 的高度和垂向支撑控制，而不是继续调 trigger 阈值。
- 之后再细化前腿过柱、机身过柱、后腿过柱的 3R+rail 协同轨迹。

## 2026-04-25 07:24 CST - 初段高度/支撑收敛开始

### 现在想实现什么
- 解决 window crossing 初段机身高度和支撑不足导致的随机性，优先让 `max_x` 稳定推到 `window_x=1.55` 附近。
- 当前重点不是继续调 trigger 阈值，而是让 PRE_APPROACH/APPROACH 阶段的 MPC 输出给 WBC 持续、确定的垂向支撑。

### 准备用什么方法实现
- 在 `mpc_node_complete.cpp` 的 active control path 中增加一个最小化侵入的垂向支撑后处理：MPC 求解后、发布 `/dog2/mpc/foot_forces` 前，根据 odom 高度和垂向速度给支撑腿补足 z 向力下限。
- 如果 MPC 偶发求解失败，不立即让 `/dog2/mpc/foot_forces` 断流，而是发布一个只包含垂向支撑的 fallback，避免 WBC 在关键初段失去输入。

### 为什么这样做
- 之前 best run 已经证明 trigger 和 window world 可以把机器人推进到 `max_x=1.489`，但同参数下也会退化到 `max_x=0.612`，说明核心问题是支撑链路随机性，而不是单一触发阈值。
- 当前 WBC 主要执行 `J^T f`，没有额外 body height 闭环；MPC 解或步态接触相位短时抖动会直接反映为机身塌低。
- 先在 MPC 输出端补足支撑力，改动范围小，不改 URDF，也不绕开 `MPC + WBC` 研究栈。

### 当前具体做了哪些操作
- 重新确认远端工作区和 dirty 状态，未改动 URDF/xacro/mesh。
- 检查 `ContactDetector::ContactState`，确认默认四足接触为 `true`，WALKING/PRE_APPROACH 会按 gait phase 清 swing leg force。
- 检查 `mpc_node_complete.cpp`，确认现在 MPC 求解失败会直接 return，可能造成 WBC 输入断流。

### 过程中遇到了什么问题
- 实验室机没有 `rg`，改用 `find + grep` 检查代码。
- `research_mpc.yaml` 位于 `src/dog2_bringup/config/research_mpc.yaml`，不是 `src/dog2_mpc/config/`。

### 原因判断
- 初段不稳定主要来自低高度时支撑力发布不连续和支撑腿分配不确定；在 crossing initialized 后当前日志中的 contacts 默认全接触，但 PRE_APPROACH 仍受 gait phase 影响，容易在低速/低高度下失稳。

### 做了哪些修改和尝试
- 目前正在准备修改 `mpc_node_complete.cpp` 和 `research_mpc.yaml`，尚未写回远端。

### 当前结果如何，是否成功
- 仍处于实现阶段，尚未构建或验证。

### 下一步准备做什么
- 增加可参数化的 `vertical_support_*` 配置。
- 构建 `dog2_mpc`/`dog2_bringup`，先跑 flat smoke，再跑 window crossing cold start，记录 `max_x`、rail delta、stage 是否推进。

## 2026-04-25 07:28 CST - 垂向支撑补强已实现并构建

### 现在想实现什么
- 把 PRE_APPROACH/APPROACH 的高度支撑做成确定行为，减少 crossing 初段随机塌低。

### 准备用什么方法实现
- 在 `mpc_node_complete.cpp` 中对 MPC 求解后的 `u_optimal` 做支撑力后处理。
- 用 `research_mpc.yaml` 暴露参数，便于后续直接通过配置调 `target_height`、`kp/kd` 和每腿最大支撑力。

### 为什么这样做
- 这是 active `MPC -> /dog2/mpc/foot_forces -> WBC -> effort_controller` 链路上的最小改动。
- 不修改 URDF，不新建动作播放器，不绕过 WBC。

### 当前具体做了哪些操作
- 新增参数：
  - `vertical_support_enabled`
  - `vertical_support_target_height`
  - `vertical_support_kp`
  - `vertical_support_kd`
  - `vertical_support_min_total_force_multiplier`
  - `vertical_support_max_leg_force`
  - `vertical_support_height_error_limit`
- 新增逻辑：
  - WALKING/PRE_APPROACH 仍按 gait contact mask 清 swing leg force。
  - initialized crossing 阶段默认四足都可补足垂向支撑。
  - MPC 求解失败时发布 12 维 fallback foot force，避免 topic 断流。
  - MPC 日志增加 `vz` 和 `fz`，便于判断支撑是否实际进入 WBC。

### 过程中遇到了什么问题
- 没有新增构建错误。

### 原因判断
- 当前补强是输出端支撑下限，不改变优化器主体；理论上不会破坏 rail target 或 crossing 状态机。

### 做了哪些修改和尝试
- 修改 `src/dog2_mpc/src/mpc_node_complete.cpp`。
- 修改 `src/dog2_bringup/config/research_mpc.yaml`。
- 执行 `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install`。

### 当前结果如何，是否成功
- 构建成功：`dog2_mpc` 和 `dog2_bringup` 均通过。

### 下一步准备做什么
- 跑 flat smoke 回归，确认平地 stand/forward/turn 不被支撑补强破坏。
- 跑 window crossing cold start，观察 `h/vz/fz`、`max_x`、rail delta 和状态机推进。

## 2026-04-25 07:32 CST - 修复测试启动残留进程污染

### 现在想实现什么
- 让 flat smoke 和 window crossing 的 cold start 真正从干净 ROS/Gazebo 进程状态开始，避免旧节点污染 `/controller_manager`、topic 和 service discovery。

### 准备用什么方法实现
- 扩大 `smoke_test.launch.py` 和 `window_crossing_test.launch.py` 的 preclean/cleanup 范围，不只清 world/gazebo，还清 Dog2 研究栈 orphan 节点。

### 为什么这样做
- 刚才 flat smoke 失败时并非算法错误，而是 `effort_controller` 和 `joint_state_broadcaster` 未激活，`/dog2/mpc/foot_forces` 等流全部缺失。
- 进程表中发现多个挂在 PID 1 下、运行超过 5 小时的旧 `robot_state_publisher`、`gz_pose_to_odom`、`sim_state_estimator_node.py`、`wbc_node_complete`、`wbc_effort_mux`，会造成假阳性/假阴性。

### 当前具体做了哪些操作
- 修改 `src/dog2_bringup/launch/smoke_test.launch.py`。
- 修改 `src/dog2_bringup/launch/window_crossing_test.launch.py`。
- 清理范围增加：
  - `robot_state_publisher`
  - `gz_pose_to_odom`
  - `sim_state_estimator_node.py`
  - `gait_scheduler_node.py`
  - `mpc_node_complete`
  - `wbc_node_complete`
  - `wbc_effort_mux`
  - debug adapter / visualization node
  - `parameter_bridge`
  - Gazebo/ign/gz sim
  - controller spawner

### 过程中遇到了什么问题
- smoke 第一次失败暴露出以前 cleanup 只覆盖 world 关键字，无法清理 launch 退出后被 PID 1 收养的 ROS 节点。

### 原因判断
- 这类 orphan 节点会让新测试看到旧 ROS graph，或者让 service/topic 状态不一致，导致 controller manager ready check 长时间失败。

### 做了哪些修改和尝试
- 对当前远端残留执行 `TERM -> sleep -> KILL` 强制清理。
- 重新构建 `dog2_bringup`，构建通过。

### 当前结果如何，是否成功
- 当前进程表已没有 Dog2/Gazebo 残留。
- 尚未重跑 smoke。

### 下一步准备做什么
- 立即重跑 flat smoke；如果通过，再跑 window crossing。

## 2026-04-25 07:42 CST - window world controller manager 启动收敛

### 现在想实现什么
- 解决 `dog2_window_frame` world 下偶发 `/controller_manager/list_controllers` 不出现的问题，让 window crossing cold start 至少进入真实控制链路。

### 准备用什么方法实现
- 对比 flat smoke 和 window 日志，先改启动时序，不改控制算法。
- window crossing 不再默认强制 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，改回与已 PASS 的 flat smoke 一致的默认 DDS 行为。
- 将 `spawn_delay_sec` 从 window launch 显式传到 `system.launch.py -> sim_base.launch.py -> effort_research_sim.launch.py`，window 默认使用 8 秒，让 Gazebo world 和 `robot_state_publisher` 服务先稳定，再创建机器人。

### 为什么这样做
- flat smoke 中 `gz_ros2_control` 能在请求 `/robot_state_publisher/get_parameters` 后收到 URDF 并加载 controller manager。
- window 失败日志中 `gz_ros2_control` 已连接 `robot_state_publisher` 服务，但 `robot_state_publisher` 响应超时，随后 controller manager 永远没有创建。
- 这说明问题在 robot spawn / plugin init 时序和 DDS 服务响应，而不是 crossing 状态机或 MPC 输出。

### 当前具体做了哪些操作
- 修改 `crossing_trial.launch.py`：
  - 新增 `spawn_delay_sec` 参数，默认 `8.0`。
  - 删除默认 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
  - 将 `spawn_delay_sec` 传入 `system.launch.py`。
- 修改 `window_crossing_test.launch.py`：
  - 新增 `spawn_delay_sec` 参数，默认 `8.0`。
  - 删除默认 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
  - 传递 `spawn_delay_sec`。
- 修改 `system.launch.py` / `sim_base.launch.py`：
  - 增加并向 research effort sim 传递 `spawn_delay_sec`。
- 强制清理上一轮 timeout 留下的 `ign gazebo` 和 `crossing_check` 残留。

### 过程中遇到了什么问题
- 上一轮 window 测试超时后，仍留下 `ign gazebo -r -s window_frame.sdf` 和 `dog2_crossing_check` orphan。

### 原因判断
- 外层 `timeout` 杀 launch 后，launch 的 `OnProcessExit` cleanup 不一定能执行完，因此测试入口必须有更强 preclean。
- window world 比 flat 更容易暴露 `robot_state_publisher` 服务响应超时；先拉长 spawn delay 并取消 UDP-only 是最小风险修复。

### 做了哪些修改和尝试
- 写回上述 launch 修改。
- `colcon build --packages-select dog2_bringup --symlink-install` 通过。

### 当前结果如何，是否成功
- 构建成功。
- 尚未重跑 window crossing。

### 下一步准备做什么
- 直接重跑 window crossing cold start。
- 如果 controller manager 仍不出现，再考虑在 test launch 里增加自动 retry，而不是继续等待单次启动。

## 2026-04-25 07:46 CST - PRE_APPROACH 卡滞后的初始化点调整

### 现在想实现什么
- 在 controller manager 已稳定启动后，继续推进 `max_x` 接近 `window_x`，让 crossing 不长期卡在 `CROSSING:PRE_APPROACH`。

### 准备用什么方法实现
- 将 `crossing_activation_distance` 默认从 `0.55` 提到 `0.75`，也就是 controller crossing 初始化点从 `x=1.00` 前移到 `x=0.80`。

### 为什么这样做
- 最新 window run 已经证明启动链路成功：controller manager 出现，MPC/WBC 持续工作，trigger 已发。
- 但机器人在低高度预接近下 `max_x` 到 `0.878` 后停住，无法达到旧的 `activation_x=1.00`，导致状态机一直显示 `CROSSING:PRE_APPROACH`。
- 直接把初始化点前移到 `x≈0.80` 是比继续加大 `/cmd_vel` 或等待更小风险的改动；它仍在现有 `CrossingStateMachine + MPC + WBC` 链路内完成。

### 当前具体做了哪些操作
- 修改 `crossing_trial.launch.py` 默认 `crossing_activation_distance=0.75`。
- 修改 `window_crossing_test.launch.py` 默认 `crossing_activation_distance=0.75`。
- 重新构建 `dog2_bringup`。

### 过程中遇到了什么问题
- SSH command 偶发返回 255，但远端 launch 实际继续运行；通过重新 SSH 检查进程和日志确认了真实状态。
- 上一轮测试被手动终止，避免继续在 PRE_APPROACH 长时间等待。

### 原因判断
- 目前不是 trigger 未发，也不是 rail 没动；问题是低高度预接近不能稳定推到旧初始化点。

### 做了哪些修改和尝试
- 使用中间初始化点 `x=0.80`，避免退回上一轮过早初始化的 `x=0.50`。

### 当前结果如何，是否成功
- 构建成功。
- 尚未验证新的初始化点。

### 下一步准备做什么
- 清理残留后重跑 window crossing，观察是否从 `PRE_APPROACH` 进入 `APPROACH/BODY_FORWARD_SHIFT`，以及 `max_x` 是否超过 `0.878`。

## 2026-04-25 07:51 CST - APPROACH QP 不可行修复

### 现在想实现什么
- 解决 initialized crossing 后 `APPROACH` 阶段 OSQP 大量 status 3/7 的问题，让机器人在 `APPROACH` 继续向窗口推进。

### 准备用什么方法实现
- 在 `MPCController::buildQP/addSlidingConstraints` 中识别 `CrossingState::APPROACH`。
- `APPROACH` 阶段跳过 crossing rail 约束和对应 slack 变量；从 `BODY_FORWARD_SHIFT` 开始再启用 rail compact-body 阶段约束。

### 为什么这样做
- 最新验证表明 controller manager 已稳定，trigger 已发，状态机已从 `PRE_APPROACH` 进入 `CROSSING:APPROACH`，`max_x` 从 0.878 提到 0.964。
- 但进入 `APPROACH` 后 QP 约束数量从 76 增加到 226，OSQP 连续不可行，MPC 只能发布垂向 fallback，缺少前进控制。
- `APPROACH` 语义仍是靠近窗口，不应过早用 crossing rail symmetry/coordination guard 卡死；真正 rail 重排应从 `BODY_FORWARD_SHIFT` 开始。

### 当前具体做了哪些操作
- 修改 `src/dog2_mpc/src/mpc_controller.cpp`：
  - `APPROACH` 阶段不分配 rail slack 变量。
  - `APPROACH` 阶段 `addSlidingConstraints()` 直接返回，保留状态目标跟踪和物理仿真限位。

### 过程中遇到了什么问题
- 第一次 `colcon build --packages-select dog2_mpc dog2_bringup` 中 `dog2_bringup` 发生一次 code -11 崩溃，但没有源码编译错误。

### 原因判断
- `dog2_mpc` 已编译通过，随后单独重跑 `dog2_bringup` 立即通过，判断为构建工具/环境瞬时问题，不是代码错误。

### 做了哪些修改和尝试
- 重跑 `colcon build --packages-select dog2_bringup --symlink-install --event-handlers console_direct+`，通过。

### 当前结果如何，是否成功
- 构建成功：`dog2_mpc` 和 `dog2_bringup` 均已通过。
- 尚未验证 window crossing 新行为。

### 下一步准备做什么
- 重跑 window crossing，重点观察 OSQP fail 是否下降、`max_x` 是否继续超过 0.964、状态机是否进入 `BODY_FORWARD_SHIFT`。

## 2026-04-25 07:59 CST - crossing 前进辅助与验证节点退出修复

### 现在想实现什么
- 降低 window crossing 对随机初始姿态/接触相位的敏感性，让 `APPROACH` 更稳定地推进到 `BODY_FORWARD_SHIFT`。
- 让 `crossing_check` 写出 PASS/FAIL 后真正退出，从而使单命令验证能自动收尾。

### 准备用什么方法实现
- 在 MPC 输出后处理里增加 crossing forward assist：在 `PRE_APPROACH/APPROACH` 给支撑腿补一个小的 +x 足端力。
- 将 APPROACH 完成速度阈值从 `0.25` 放宽到 `0.45`，避免到达窗口前沿后因为速度略大而卡住。
- `crossing_check.py` 在 `_finish()` 写结果后使用 `os._exit()` 直接退出 sentinel 进程。

### 为什么这样做
- 一轮验证已 PASS：`stage=CROSSING:BODY_FORWARD_SHIFT`、`max_x=1.673`、`max_rail_delta=0.111`。
- 但下一轮复测仍随机卡在 `APPROACH max_x≈0.957`，说明“能过”已经成立，但稳定性仍不足。
- 当前 WBC 对足端力直接做 `J^T f`，MPC 在低高度下能给出垂向力但前进驱动力不足；forward assist 仍在 MPC/WBC 链路内，不是外部动作播放器。

### 当前具体做了哪些操作
- 修改 `mpc_node_complete.cpp`：
  - 增加 `crossing_forward_assist_enabled`。
  - 增加 `crossing_forward_assist_force_per_leg`，默认 `14.0 N`。
  - 在 success 和 MPC fallback 路径都应用 forward assist。
- 修改 `research_mpc.yaml`，显式写入 forward assist 参数。
- 修改 `crossing_state_machine.cpp`，APPROACH 速度 guard 放宽到 `0.45`。
- 修改 `crossing_check.py`，确保 PASS/FAIL 后进程退出。

### 过程中遇到了什么问题
- 并行执行写回和清理命令时，清理动作打断了 SSH/SCP；已改为按顺序执行。

### 原因判断
- 清理命令涉及 launch/Gazebo/ROS 节点，不能和写文件/构建并行执行。

### 做了哪些修改和尝试
- 顺序清理残留、写回文件、重新构建。
- `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install` 通过。

### 当前结果如何，是否成功
- 构建成功。
- 已有一次 window crossing PASS 结果文件：`/tmp/dog2_window_crossing_result_approach_skip.txt`。
- forward assist 之后尚未再次验证。

### 下一步准备做什么
- 重跑 window crossing final 验证。
- 再跑 flat smoke 回归，确认新增 forward assist 不影响平地测试。

## 2026-04-25 08:10 CST - 本轮验证收尾与剩余阻塞

### 现在想实现什么
- 收尾本轮对 crossing 初段支撑、启动随机性和验证节点退出的修复，明确哪些能力已经可复现，哪些仍然没有完全闭合。

### 准备用什么方法实现
- 用三类结果做判断：
  - `colcon build` 是否通过；
  - window crossing 是否能从冷启动进入 `CROSSING:BODY_FORWARD_SHIFT`，并让 rails 真实移动；
  - flat smoke 是否仍能通过 stand/forward/turn 回归。

### 为什么这样做
- 当前目标不是只让某一次窗口场景看起来会动，而是把问题分解成可回归的工程入口。
- 如果 window crossing 已能进入 BODY 阶段但 flat smoke 回归失败，说明修复破坏了基础控制栈；如果 flat smoke 通过但 crossing 后续不稳定，则下一步应继续收敛 crossing 阶段控制，而不是回退启动/触发逻辑。

### 当前具体做了哪些操作
- 完成 `dog2_mpc` 和 `dog2_bringup` 构建验证。
- 重跑 window crossing，已有两次可用 PASS 记录：
  - `/tmp/dog2_window_crossing_result_approach_skip.txt`：`stage=CROSSING:BODY_FORWARD_SHIFT`，`max_x=1.673`，`max_rail_delta=0.111`。
  - `/tmp/dog2_window_crossing_result_body_assist.txt`：`stage=CROSSING:BODY_FORWARD_SHIFT`，`max_x=1.682`，`max_rail_delta=0.111`。
- 修复 `crossing_check.py` 的退出路径后，用一次失败 run 验证 launch 能正常返回并清理残留进程：
  - `/tmp/dog2_window_crossing_result_exitfix.txt` 写出 FAIL 后 launch 返回；
  - 后续 `ps` 检查没有发现 `gz sim`、`mpc_node_complete`、`wbc_node_complete`、`crossing_check` 等残留目标进程。
- 重跑平地 smoke：
  - `/tmp/dog2_smoke_result_final.txt` 写出 PASS；
  - 结果为 stand/forward/turn 全阶段通过，`turn_yaw_delta=3.079`。

### 过程中遇到了什么问题
- window crossing 仍存在启动随机性：一次验证在 controller/state 链路未正常出流时 FAIL，缺失 `/dog2/state_estimation/odom`、`/dog2/state_estimation/robot_state`、`/dog2/mpc/foot_forces`、`/dog2/wbc/joint_effort_command`、`/dog2/wbc/rail_effort_command` 和 `/effort_controller/commands`。
- 即使进入 BODY 阶段，`BODY_FORWARD_SHIFT` 后续仍没有稳定推进到后腿过柱和 recovery。
- 平地 smoke 虽然 PASS，但日志仍可见低高度和少量 OSQP status 3/7，说明垂向支撑 fallback 是有效兜底，不代表动力学控制已经足够干净。

### 原因判断
- `gz_pose_to_odom` 和启动清理已经比之前稳定；当前最大阻塞不再是单纯 trigger 阈值，而是初段机身高度、接触相位和 WBC 实际支撑能力不足。
- `APPROACH` 阶段过早施加 crossing rail 约束会使 QP 不可行，因此本轮把 rail compact-body 约束推迟到 `BODY_FORWARD_SHIFT` 是合理的最小修复。
- 当前 rails 已经不是装饰状态：PASS run 中 `max_rail_delta=0.111`，且 `/dog2/wbc/rail_effort_command`、`/effort_controller/commands` 参与验证；但 rails 的阶段化动作还没有稳定完成全流程。

### 做了哪些修改和尝试
- 在 `mpc_node_complete.cpp` 中增加垂向支撑后处理和 MPC 失败 fallback，避免 OSQP 临时不可行时整条力输出链断掉。
- 在 crossing `PRE_APPROACH/APPROACH/BODY_FORWARD_SHIFT` 中增加小幅前向支撑腿足端力辅助，帮助低高度初段继续向 `window_x` 推进。
- 在 `mpc_controller.cpp` 中跳过 `APPROACH` 阶段 crossing rail 约束和 slack 分配，避免 initialized crossing 后立即 QP 不可行。
- 在 `crossing_state_machine.cpp` 中放宽 `APPROACH` 完成速度 guard。
- 在 window/smoke launch 中增强 cold-start preclean，减少旧进程导致的假阳性和假阴性。
- 在 `crossing_check.py` 中改成写出 PASS/FAIL 后直接退出，保证单命令验证能自动收尾。

### 当前结果如何，是否成功
- 已成功：
  - `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install` 通过；
  - flat smoke 回归 PASS；
  - window crossing 至少两次进入 `CROSSING:BODY_FORWARD_SHIFT`，`max_x` 达到 `1.67+`，rails 有显著位移；
  - 验证节点退出/清理路径已修复。
- 未完全成功：
  - window crossing 还不是稳定 PASS；
  - 状态机尚未稳定推进到 `FRONT_LEGS_TRANSIT`、`REAR_LEGS_TRANSIT`、`RECOVERY` 全流程；
  - 机身高度和垂向支撑仍是下一阶段主要阻塞。

### 下一步准备做什么
- 优先加强 PRE_APPROACH/APPROACH 的 body height 和垂向支撑控制，而不是继续只调 trigger 阈值。
- 检查 WBC 的姿态/高度目标是否被低高度站立目标、rail clamp 或 joint posture 目标抵消。
- 在 crossing check 中继续提高阶段判据：不只看 `BODY_FORWARD_SHIFT`，下一步要区分前腿过柱、机身穿柱、后腿过柱和 recovery。

## 2026-04-25 08:10 CST - 本轮最终总结

### 最终实现了什么
- 把 window crossing 从容易卡在 `PRE_APPROACH/APPROACH` 推进到能冷启动进入 `CROSSING:BODY_FORWARD_SHIFT` 的版本。
- rails 已经在 crossing PASS run 中真实参与动作，`lf_rail_joint`、`lh_rail_joint`、`rh_rail_joint`、`rf_rail_joint` 对应通道能在 WBC/effort 输出链中产生显著变化。
- 平地 stand/forward/turn smoke 回归仍然通过。

### 关键改动有哪些
- `src/dog2_mpc/src/mpc_node_complete.cpp`：增加垂向支撑后处理、MPC 失败 fallback、crossing 前向辅助，并把这些输出继续送入现有 MPC/WBC 链路。
- `src/dog2_mpc/src/mpc_controller.cpp`：`APPROACH` 阶段暂不启用 crossing rail 约束，避免接近窗口时 QP 被提前锁死。
- `src/dog2_mpc/src/crossing_state_machine.cpp`：放宽 APPROACH 完成速度 guard。
- `src/dog2_bringup/launch/*.launch.py`：增强 smoke/window cold-start 清理，增加 spawn delay 传递，调整 crossing activation 默认距离。
- `src/dog2_bringup/dog2_bringup/crossing_check.py`：PASS/FAIL 后直接退出，保证验证命令可自动结束。

### 验证怎么做的
- 构建：
  - `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install --event-handlers console_direct+`
- window crossing：
  - `ros2 launch dog2_bringup window_crossing_test.launch.py ros_domain_id:=91 timeout_sec:=150.0 result_file:=/tmp/dog2_window_crossing_result.txt`
  - 已记录 PASS：`max_x=1.673/1.682`，`max_rail_delta=0.111`，stage 到达 `CROSSING:BODY_FORWARD_SHIFT`。
- flat smoke：
  - `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true ros_domain_id:=93 result_file:=/tmp/dog2_smoke_result.txt`
  - 已记录 PASS：`turn_yaw_delta=3.079`。

### 还剩什么问题
- crossing 仍不是稳定 PASS；一次修复退出路径后的 run 因启动链路无数据 FAIL。
- BODY_FORWARD_SHIFT 之后还没有稳定完成后腿跨柱和恢复常态。
- 低高度、接触相位和 WBC 支撑能力仍是主要短板，不能只靠 trigger/activation 阈值继续解决。

### 建议的下一步
- 先收敛 PRE_APPROACH/APPROACH 的 body height：检查 state estimator 高度、WBC 姿态/高度目标、joint posture 目标和 rail 目标之间是否相互抵消。
- 再把 `crossing_check.py` 的 PASS 门槛从“进入 BODY 且 rail 动过”升级为“BODY 穿柱 + 后腿过柱 + recovery”。
- 最后针对 `BODY_FORWARD_SHIFT` 的 QP 和 WBC effort 输出做分阶段日志，确认前后腿 rail 重排和 3R 腿部抬腿/折叠/落地动作是否同步。

## 2026-04-25 10:00 CST - PRE_APPROACH/APPROACH 支撑诊断收敛

### 现在想实现什么
- 不再凭感觉判断“机身高度不足到底是不是 rail 目标和 3R 支撑互相打架”，而是把问题拆成三组可复现实验：
  1. `crossing_force_full_support:=true`，固定四足支撑；
  2. `crossing_freeze_rail_targets:=true`，冻结 crossing 的 rail 目标/rail-stage 约束；
  3. 再进一步把最终 `effort_controller` 的 rail 通道置零，做真正的 3R-only 诊断。

### 准备用什么方法实现
- 在 `mpc_node_complete.cpp` 加两个研究参数：
  - `crossing_force_full_support`
  - `crossing_freeze_rail_targets`
- 在 `mpc_controller.cpp` 增加 diagnostic path：
  - freeze rail target 时，用 measured rail 位置替代 crossing rail reference；
  - 同时跳过 rail-stage 约束和 rail-window 约束。
- 在 `wbc_node_complete.cpp`、`wbc_effort_mux.py`、`crossing_check.py` 增强日志：
  - 打印 MPC 分腿足端力；
  - 打印 WBC 分腿 `coxa/femur/tibia/rail` effort 和 `femur+tibia` 绝对值；
  - 打印 mux 最终 16 通道分腿输出；
  - 记录 crossing run 的 `min_z/max_z`。

### 为什么这样做
- 之前只知道 crossing 初段经常掉高、随机性大，但不知道原因是在：
  - gait contact phase；
  - MPC 足端力；
  - WBC 力矩映射；
  - 还是 rail 目标/rail actuator 抢占了控制自由度。
- 先把这三层拆开，比继续调 trigger 阈值更有效，也能避免在错误方向上长时间调参。

### 当前具体做了哪些操作
- 修改 `src/dog2_mpc/src/mpc_node_complete.cpp`：
  - 支持 `crossing_force_full_support` 和 `crossing_freeze_rail_targets`；
  - 在周期日志中打印 `full_support/freeze_rails` 状态；
  - 增加 `MPC leg_forces` 分腿日志。
- 修改 `src/dog2_mpc/include/dog2_mpc/mpc_controller.hpp` 和 `src/dog2_mpc/src/mpc_controller.cpp`：
  - 新增 `setFreezeCrossingRailTargets()`；
  - freeze rail target 时，不再把 crossing rail 目标写入参考轨迹，也不再施加 rail-stage 约束。
- 修改 `src/dog2_wbc/src/wbc_node_complete.cpp`：
  - 从 `/joint_states` 读取各腿关节速度和 rail 速度；
  - 打印 `WBC leg_effort`，显式包含 `femur/tibia` effort 和机械功率。
- 修改 `src/dog2_bringup/dog2_bringup/wbc_effort_mux.py`：
  - 打印最终 `/effort_controller/commands` 的分腿 breakdown；
  - 新增 `freeze_rail_effort` 参数，可把最终四个 rail 通道直接置零。
- 修改 `src/dog2_bringup/dog2_bringup/crossing_check.py`：
  - 在结果文件和周期日志中加入 `min_z/max_z`。
- 修改 launch/config：
  - `control_stack.launch.py`、`system.launch.py`、`crossing_trial.launch.py`、`window_crossing_test.launch.py` 和 `research_mpc.yaml` 传递这些新参数。

### 过程中遇到了什么问题
- 第一次做 `freeze_rail_targets` 诊断时，发现 `max_rail_delta` 仍然到 `0.112`，而且最终 `/effort_controller/commands` 里的 rail 通道仍然非零。
- 这说明“冻结 rail 目标/约束”并不等于“冻结 rail 实际动作”。

### 原因判断
- WBC 当前是 `tau = J^T f`，而 rail 对应的雅可比列在 x 方向上是显式非零的。
- 所以即使 MPC 不再给 rail target，只要足端还在给前向/支撑力，WBC 仍会在 rail 通道上生成 effort。
- 也就是说，之前用“冻结 rail 目标”去判断 rail/posture 是否冲突，诊断不够干净。

### 做了哪些修改和尝试
- 新增 `freeze_rail_effort` 并接入 `wbc_effort_mux` 的最终输出链。
- 第一次接线时把参数错接到了 `wbc_node_complete`，导致 mux 仍输出非零 rail；
  - 通过启动日志发现 `wbc_effort_mux ready ... freeze_rail_effort=False`；
  - 随后修正到真正的 `wbc_effort_mux` 节点参数。

### 当前结果如何，是否成功
- 构建通过：
  - `colcon build --packages-select dog2_mpc dog2_wbc dog2_bringup --symlink-install` 通过。
- 三组诊断结果如下：
  - `full_support only`：
    - `/tmp/dog2_window_crossing_full_support.txt`
    - `PASS: stage=CROSSING:BODY_FORWARD_SHIFT; max_x=1.653; min_z=0.059; max_z=0.441`
  - `freeze rail targets/constraints only`：
    - `/tmp/dog2_window_crossing_freeze_rail.txt`
    - `FAIL: stage=CROSSING:APPROACH; max_x=1.343; min_z=0.047; max_z=0.438`
    - 但这组不够纯，因为 rail 通道仍在动。
  - `freeze rail effort on final mux`：
    - `/tmp/dog2_window_crossing_freeze_rail_effort_verify.txt`
    - `PASS: stage=CROSSING:APPROACH; max_x=1.656; min_z=0.063; max_z=0.446`
    - 启动日志明确显示 `wbc_effort_mux ready ... freeze_rail_effort=True`
    - 最终 mux 日志中四个 rail 通道都为 `0.0`。

### 结论
- 结论 1：`full_support` 能证明 MPC 和 WBC 的垂向支撑链路是通的。
  - 在有效 run 中，MPC 四腿都能持续给 `fz≈100N`；
  - WBC 的 `femur+tibia` 通道也持续有显著 effort。
- 结论 2：问题不是“垂向力没有进入 femur/tibia”。
  - 这一点已经被 `WBC leg_effort` 日志坐实。
- 结论 3：问题也不是“rail 目标/rail effort 是初段掉高的主要根因”。
  - 真正的 3R-only 诊断里，rail 输出已经被压到 0，但 `min_z` 仍只有 `0.063`，和 `full_support only` 的 `0.059` 基本同级。
  - 说明把 rail 去掉并没有显著改善 PRE_APPROACH/APPROACH 的 body height。
- 结论 4：下一步主矛盾应转向 3R 支撑本体，而不是继续盯 rail 目标冲突。

### 下一步准备做什么
- 优先检查 3R 支撑本体：
  - femur/tibia torque limit 是否过早饱和；
  - 当前关节构型下的雅可比是否让垂向支撑效率过低；
  - body pitch/roll 扰动是否在消耗本该用于抬高机身的 effort。
- 针对 PRE_APPROACH/APPROACH 单独加更强的“高度优先”策略，而不是继续调 trigger。
- 如果继续验证 crossing，本轮建议把 PASS 标准和诊断标准分开：
  - crossing 行为验证仍看 `max_x/stage`；
  - 高度诊断单独看 `min_z/max_z` 和 `femur+tibia` effort。

## 2026-04-25 10:27 CST - 3R 支撑诊断闭环与初段姿态补偿

### 现在想实现什么
- 把 PRE_APPROACH/APPROACH 的主矛盾从“猜测”变成“有数据支撑的结论”，并在此基础上继续把 window crossing 的初段推进能力拉回到 `BODY_FORWARD_SHIFT`。
- 重点不再调 trigger，而是回答三个问题：
  1. `femur/tibia` 是否频繁接近有效饱和；
  2. 当前雅可比下的垂向支撑容量是否太差；
  3. body `roll/pitch` 扰动是否才是主要失稳源。

### 准备用什么方法实现
- 在 `wbc_node_complete.cpp` 增加更细的分腿观测：
  - `fu/tu`：`femur/tibia` 力矩利用率；
  - `sat`：近饱和标记；
  - `jzf/jzt`：当前雅可比的垂向力矩系数；
  - `fz_cap/fz_margin`：按当前雅可比和 `max_torque` 估算的纯垂向支撑容量与余量。
- 在 `mpc_node_complete.cpp` 增加 crossing 初段姿态支撑补偿：
  - 读取 `roll/pitch` 与 `wx/wy`；
  - 在 PRE_APPROACH / APPROACH / BODY_FORWARD_SHIFT 里对四腿 `fz` 做左右/前后重分配；
  - 打印 `MPC attitude` 日志，看补偿是否真的生效。
- 如果补偿方向正确但效果仍被削弱，再检查是否被 `vertical_support_max_leg_force` 顶死。

### 为什么这样做
- 上一轮已经确认：
  - rail 不是当前初段掉高的主因；
  - MPC 到 WBC 的垂向支撑链是通的。
- 所以下一层最值得查的是“支撑有没有真正变成有用的姿态稳定力矩”，而不是继续围绕 rail 或 activation distance 做文章。

### 当前具体做了哪些操作
- 修改 `src/dog2_wbc/src/wbc_node_complete.cpp`：
  - 给 `WBC leg_effort` 增加 `fu/tu/sat/jzf/jzt/fz_cap/fz_margin` 字段。
- 修改 `src/dog2_mpc/src/mpc_node_complete.cpp`：
  - 新增 `attitude_support_*` 参数；
  - 在 crossing 初段增加 `applyAttitudeSupport()`，按 `roll/pitch` 与角速度对四腿 `fz` 做重分配；
  - 新增 `MPC attitude` 周期日志。
- 修改 `src/dog2_bringup/config/research_mpc.yaml`：
  - 默认打开 `attitude_support_enabled`。

### 过程中遇到了什么问题
- 第一次跑新诊断时，日志里没有出现新字段，表现得像“补丁完全没生效”。

### 原因判断
- 不是代码没写进去，而是我当时把 `scp` 和 `colcon build` 并行执行了。
- 构建过程吃到的是旧源码；后面通过时间戳和二进制字符串确认：
  - 远端源文件时间已更新；
  - 但 `build/.../mpc_node_complete` 与 `build/.../wbc_node_complete` 还是旧时间戳，且不包含 `MPC attitude` / `fu=` 等字符串。

### 做了哪些修改和尝试
- 先顺序重建 `dog2_mpc dog2_wbc dog2_bringup`，确认二进制里已经包含新日志字符串。
- 然后跑 `window_crossing_test.launch.py`（`ros_domain_id:=100`）做第一轮有效诊断：
  - 结果：`FAIL: stage=CROSSING:APPROACH; max_x=1.325; min_z=0.053; max_z=0.448; max_rail_delta=0.112`
  - 日志统计：
    - `attitude_samples=110`
    - `roll_abs_max=3.141`
    - `pitch_abs_max=0.813`
    - `wx_abs_max=9.86`
    - `wy_abs_max=8.205`
    - `att_dz_abs_max=18.0`
    - `leg_samples=440`
    - `fu_max=0.789184`
    - `tu_max=0.403298`
    - `sat_count=0`
    - `fz_cap_min=126.722`
    - `fz_margin_min=26.7222`
- 由此得到结论：
  - `femur/tibia` 没有接近饱和；
  - 当前雅可比估算的垂向容量也没有贴着 100N 上限；
  - 真正的主矛盾是姿态扰动巨大，而不是“3R 力矩不够”或“雅可比垂向容量不够”。
- 在此基础上继续修：
  - 发现 `vertical_support` 先把每条支撑腿都顶到 `100N`，导致姿态补偿只能把一部分腿往下减，另一部分腿无法往上加，右ing torque 被截断。
  - 因此修改 `applyVerticalSupport()`：
    - 在 crossing 初段稳定化阶段主动给 `attitude_support_max_leg_delta` 预留 `fz` headroom；
    - 同时把 `attitude_support_max_leg_delta` 从 `18.0` 提到 `24.0`。

### 当前结果如何，是否成功
- 第二轮 window crossing 验证（`ros_domain_id:=101`）：
  - `PASS: crossing validated; stage=CROSSING:BODY_FORWARD_SHIFT; max_x=1.659; min_z=0.043; max_z=0.399; max_rail_delta=0.112`
  - 说明“给姿态补偿预留 headroom”后，crossing 初段推进能力恢复，系统又能稳定推到 `BODY_FORWARD_SHIFT`。
- 第二轮统计结果：
  - `attitude_samples=39`
  - `roll_abs_max=3.142`
  - `pitch_abs_max=0.498`
  - `wx_abs_max=7.273`
  - `wy_abs_max=6.184`
  - `att_dz_abs_max=24.0`
  - `leg_samples=184`
  - `fu_max=0.652664`
  - `tu_max=0.385461`
  - `sat_count=0`
  - `fz_cap_min=143.303`
  - `fz_margin_min=43.3034`
- 这些数据进一步说明：
  - 仍然不是 `femur/tibia` 饱和；
  - 也不是雅可比垂向容量不足；
  - 但姿态扰动仍然很大，当前补偿虽然把 crossing 推回 PASS（按现有检查标准），却还没把 `min_z` 拉上来。
- 平地 smoke 回归也已通过：
  - `ros2 launch dog2_bringup smoke_test.launch.py ... ros_domain_id:=102`
  - 结果：`PASS: turn_yaw_delta=2.822 z=0.156`

### 是否成功
- 部分成功。
- 成功点：
  - 3R 支撑诊断问题已经定性，不再盲猜；
  - window crossing 再次通过现有 PASS 标准并推进到 `BODY_FORWARD_SHIFT`；
  - flat smoke 回归通过。
- 未完全成功点：
  - `min_z` 仍然会掉到 `0.043` 左右；
  - 状态机还没有稳定推进到 `FRONT_LEGS_TRANSIT / REAR_LEGS_TRANSIT / RECOVERY`。

### 下一步准备做什么
- 继续把初段从“能到 BODY”收敛到“更稳、更少姿态炸裂”：
  1. 检查 PRE_APPROACH 是否还在继承 walking reference / gait 节奏引入不必要姿态扰动；
  2. 评估是否需要在 crossing 初段进一步降低/重写前向推进参考，使其更接近 quasi-static body-forward-shift；
  3. 在 `BODY_FORWARD_SHIFT -> FRONT_LEGS_TRANSIT` 之间补更强的阶段约束与失败判定，而不是仅凭 max_x 判断。
- 如果 classical 方案在更强阶段约束后仍无法稳定把 roll 峰值压下去，再考虑把 crossing 初段做成更明确的学习/优化接口；但当前还没有到必须引入强化学习的程度。
