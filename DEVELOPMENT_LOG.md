# DEVELOPMENT_LOG

## 2026-04-12 20:03 URDF/xacro 文档与腿链分层：关节语义、惯性注释、Phase 1 几何下沉与 Phase 2 回滚

- **dog2_description / `dog2.urdf.xacro`**
  - 文件头：修正与 `l1`–`l4` `slider_inertia_xyz` 实际符号不一致的旧注释（惯性 COM 偏移说明改为与属性一致）。
  - 关节限位：`femur_*` / `tibia_*` 与阻尼/摩擦命名拆分（避免两个转动副共名 `knee_*`）；`foot_tip` 与 `tibia_link` Gazebo 前注释改为「mesh 碰撞体」口径，去掉易误导的「原语」表述。
  - **`base_link` inertial**：为 `rpy="${half_pi} 0 0"` 增加 **FROZEN** 维护注释（CAD→REP-103 映射、主惯量直觉、`ixy` 含义；禁止为美观随意改）。
  - **Leg kinematic contract**：名义姿态下在 `base_link` 的带符号语义轴写清（rail **+X**，coxa **−Z**，femur/tibia **−Y**）；与 `dog2_motion_control/dog2_motion_control/joint_semantics.py` 的说明对齐。
  - **安装层 / 语义链 / 几何层**：在 `Leg instantiations` 与 `leg_knee_xyz_R` 后增加 Phase 0/2 分层与调查顺序注释。
  - **Phase 1（仅 `rf`）**：`thigh_rpy` / `thigh_col_rpy` 从实例参数下沉为 **`femur_link` 的 visual/collision `<origin rpy>`**（`prefix=='rf'` 分支），实例层不再传；不增加 `*_geom_fixed`。
  - **Phase 2（数值吸收 `leg_hip_rpy_R`）**：曾将 **`rh` / `rf`** 改为 `leg_mount` **L** + `leg_rail_axis_L` + 默认 coxa `rpy` + Pinocchio **平移**闭合的 `*_hip_xyz_closure`；验证发现 **未约束 `coxa_link` 完整 SE(3)**，与「仅闭足端/平移」不等价。
  - **回滚**：**`rh` 与 `rf` 的 Phase 2 mount 吸收均已撤销**，恢复 `leg_rail_rpy_R`、`leg_rail_axis_R`、`leg_hip_rpy_R`、`leg_knee_xyz_R`；`rf` 恢复显式 `hip_xyz="0.0116 0.0199 0.055"`；删除 `rh_hip_xyz_closure` / `rf_hip_xyz_closure` 属性。**Phase 1 的 `rf` 大腿 mesh/collision flip 保留**。

- **dog2_motion_control**
  - `joint_semantics.py`：与 xacro 腿运动学合同一致的带符号语义轴表述；`leg_parameters.py`：`rh` / `rf` 的 `hip_offset` 与回滚后 URDF 一致（`rh` 宏默认 `hip_xyz`；`rf` 与显式 `0.0116 0.0199 0.055` 对齐）。

- **验证（回滚后稳态）**
  - `python3 src/dog2_description/scripts/check_joint_semantics.py`：PASS  
  - `pytest src/dog2_motion_control/test/test_kinematics.py`（18 项）：PASS  

- **后续方向（已写入 xacro 注释）**
  - 不再单独用「平移闭合」重做 `leg_hip_rpy_R` 吸收；下一轮以 **`coxa`/`femur`/`tibia` 等完整 SE(3)** 为约束，或走 **axis_frame 显式化** 模板腿。

## 2026-04-12 16:38 semantic `base_link` 落盘 + legacy `urdf_shift` 收口 + 运行时模型混载定位

- **本轮目标与决策**
  - 目标不是“把 `base_offset_joint` 机械清零”，而是**保住新的 semantic `base_link` 语义**，同时把历史 `urdf_shift_*` 从源码层彻底拿掉。
  - 经几何推导确认：若强行将 `base_offset_joint` 直接改成 identity，并把整层变换完整吸收到 `base_link` 子节点中，四个 `leg_mount` 的几何中心会整体漂到 `(0.2492, 0.12503, -0.2649)`，等于把刚完成的 semantic `base_link` 主根语义重新打回旧 `base_footprint` 体系。
  - 因此本轮明确采用：**保住 semantic `base_link`，删除 `urdf_shift_*`，但保留非 identity 的 `base_offset_joint` 作为显式 `base_footprint -> semantic base_link` 放置。**

- **里程碑提交 1：semantic `base_link` 正式重定义**
  - 提交：`e2c184e` `Redefine semantic base_link in dog2 URDF`
  - 主要工作：
    - `src/dog2_description/urdf/dog2.urdf.xacro`
      - 定义 semantic `base_link` 原点：`(0.248975, 0.122500, 0.0)`。
      - `base_link_cad_fixed` 从 identity 改为非 identity，表达 CAD 壳层相对 semantic trunk root 的固定偏移。
      - 四个 `*_leg_mount_fixed`、trunk inertial、trunk box collision 全部围绕新的 semantic `base_link` 重标。
      - `base_link` 语义正式收口为 trunk inertial + trunk primitive collision + leg mounts；`base_link_cad` 收口为 visual-only。
    - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
      - guardrail 升级到 semantic base 语义，检查新的 inertial / collision / shell offset / leg_mount 坐标。
    - `src/dog2_description/scripts/check_joint_semantics.py`
      - joint 语义检查补上 `leg_mount` 新拓扑。
    - `src/dog2_description/scripts/verify_stage2_rail_geometry.py`
      - Stage 2 golden 随 semantic base 语义重基线，rail parent 以 `*_leg_mount` 为准。
    - 文档同步：
      - `src/dog2_description/README_JOINT_LIMITS.md`
      - `src/dog2_description/doc/base_frame_migration.txt`
      - `src/dog2_description/GAZEBO_CONTROL_FIX.md`
    - 删除仓库中 stale 的 `src/dog2_description/urdf/dog2.urdf`，明确 xacro 为单一真源。

- **里程碑提交 2：移除 legacy `urdf_shift_*` bookkeeping**
  - 提交：`a19268c` `Remove legacy urdf_shift bookkeeping`
  - 主要工作：
    - `src/dog2_description/urdf/dog2.urdf.xacro`
      - 删除 `urdf_shift_x / urdf_shift_y / urdf_shift_z` 属性。
      - 将 `base_offset_joint` 改写为**显式常量**：
        - `xyz = (0.2492, 0.12503, -0.2649)`
        - `rpy = (0, 0, pi)`
      - 明确说明：`base_offset_joint` 名字保留仅为兼容，不再代表隐式 CAD 补偿层。
    - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
      - 增加检查：`base_offset_joint` 必须匹配显式常量。
      - 增加检查：xacro 源中**不允许**再出现真实的 `urdf_shift_*` 属性或表达式（注释文字不算）。
    - 文档同步：
      - `src/dog2_description/README_JOINT_LIMITS.md`
      - `src/dog2_description/doc/base_frame_migration.txt`
      - `read.md`

- **验证结果（本轮全部通过）**
  - `python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict src/dog2_description/urdf/dog2.urdf.xacro`：PASS
  - `python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro`：PASS
  - `python3 src/dog2_description/scripts/verify_stage2_rail_geometry.py --xacro src/dog2_description/urdf/dog2.urdf.xacro --golden src/dog2_description/config/migration_stage2_stand.json`：PASS
  - `colcon build --packages-select dog2_description --symlink-install`：PASS
  - 导出展开 URDF 核对：
    - `base_offset_joint = (0.2492, 0.12503, -0.2649), rpy=(0,0,pi)`
    - `base_link_cad_fixed = (-0.248975, -0.1225, 0.0), rpy=(0,0,0)`

- **运行时异常排查：定位为“源码已更新，但运行态混入旧模型/旧环境”**
  - 现象：
    - RViz 中 `base_link` / `base_link_cad` 仍重合。
    - `ros2 param get /robot_state_publisher robot_description` 导出的运行时 URDF 仍显示：
      - `base_link_cad_fixed = identity`
      - 注释仍是旧的 Stage 3 语义。
  - 本地源码导出的期望 URDF 则明确为：
    - `base_link_cad_fixed xyz="-0.248975 -0.1225 0.0"`
  - 进一步排查发现：
    - 当前 shell 正确 source 到主工作区：`/home/dell/aperfect/carbot_ws/install/dog2_description`
    - 但正在运行的 `rviz2` 使用的是 cleanup worktree 的 RViz 配置：
      - `/home/dell/aperfect/carbot_ws_leg_mount_cleanup/install/dog2_description/share/dog2_description/rviz/dog2.rviz`
  - 结论：
    - **问题不是源码回退，而是运行时混入了旧 `robot_description` / 旧进程 / 多 workspace 痕迹。**
    - 因此运行时看到的 `base_link` / `base_link_cad` 重合，不能作为当前主工作区 URDF 语义未生效的证据。

- **当前语义稳态（截至本条日志）**
  - `base_link`
    - semantic trunk root
    - trunk inertial
    - trunk primitive collision
    - all four `*_leg_mount`
  - `base_link_cad`
    - visual shell only
  - `base_link_cad_fixed`
    - non-identity fixed shell offset
  - `base_offset_joint`
    - 显式 `base_footprint -> semantic base_link` 放置
    - 非 identity 是**有意保留**，不是遗留 bug
  - `urdf_shift_*`
    - 已从 xacro 源移除

- **备注**
  - 本轮只提交了 `dog2_description` / 文档相关变更，**未混入** `dog2_motion_control` 工作树中尚未收口的 Gazebo / controller / rail slip 改动。

## 2026-04-11 22:00 URDF「坐标净化」：问题分层、迁移阶段、可验证性与当前稳态

系统记录本次在 URDF 侧做的**语义拆分与逐层归位**（非单纯改 `xyz` 好看），便于后续文档/汇报引用。

### 一、一开始到底在净化什么

要净化的不是几个 `xyz`，而是把原来**缠在一起的三类语义拆开**：

1. **控制/运动学语义**：控制器、Pinocchio、腿根安装、足端解算以哪个 frame 为根。  
2. **CAD/外观语义**：mesh 导出的大偏移、壳体朝向、视觉装配。  
3. **动力学/碰撞语义**：inertial、collision 挂在哪个 link，谁才是「机身」。

原问题不是「模型全错」，而是三类语义混在一起，导致：`base_link` 名像主机身却不是；`base_link_cad` 像壳层却扛 inertial/collision；腿已参考 `base_link` 但 trunk 物理体未完全迁过来；CAD 偏移仍存在又不能到处乱补。

**坐标净化本质**：把历史 CAD 偏移、控制主根、动力学主根**拆开并重新归位**。

### 二、总体策略：先分层，不硬删偏移

不一上来删 CAD 偏移（会牵动 footprint→base、trunk、四腿、Python、站姿、Gazebo）。采用：

**先搭双基准脚手架，再逐层把语义迁回 `base_link`。**

- `base_footprint`：地面投影 / 规划语义  
- `base_link`：未来的控制/运动学/动力学主根  
- `base_link_cad`：暂存 CAD 壳层与历史 mesh 装配  

即**建新结构再搬运**，不是硬删旧世界。

### 三、第一层：`urdf_shift` 封边

不把 `urdf_shift_*` 立刻删掉，而是**只允许出现在 `base_offset_joint`**，禁止在 TF、Python、外参、控制器里二次补偿，避免「双重补偿」。  
目标：偏移仍是历史包袱，但**变成单层、可控**。

### 四、第二层：`base_link` / `base_link_cad` 双层脚手架

- `base_link`：未来主语义 frame  
- `base_link_cad`：CAD trunk 承载层  
- `base_link_cad_fixed`：两者之间固定连接（早期 **identity**，世界几何先不变）  

早期 `base_link` 仍是轻量占位根，真实 trunk inertial/visual/collision 仍在 `base_link_cad`。  
**成果**：控制根与 CAD 壳层在**结构上**首次分离。

### 五、第三层（Stage 2）：腿安装语义迁回 `base_link`

四条 `*_rail_joint` 的 **parent 统一为 `base_link`**；**rail 的 `xyz/rpy` 数值先不动**（当时 `base_link_cad_fixed` 为 identity）。  
**效果**：世界系腿几何基本不变，腿安装语义正式认 **`base_link-local`**。  
净化掉「腿挂在谁身上」；FK/IK、Pinocchio、`leg_parameters`、nominal、左右镜像均可围绕单一腿根收口。

### 六、第四层（Stage 3A）：trunk inertial 迁到 `base_link`

把 `base_link_cad` 上真实 trunk inertial 迁到 `base_link`，去掉 `base_link` 占位 inertial。  
identity 阶段无需换系与惯量主轴旋转。  
**净化**：「谁才是动力学主机身」——至少 inertial 层 `base_link` 名副其实。

### 七、第五层（Stage 3B）与当前停靠点

方向：trunk collision 从 `base_link_cad` 挪到 `base_link`，`base_link_cad` 退为 visual-only（`base_link` = inertial + collision，`base_link_cad` = visual-only）。  

与 Gazebo 启动链、控制、rail 监控等问题交织后曾**回退以排查**，故**稳态叙述**：inertial 已在 `base_link`；**collision 归属与 3B 为正确方向，但当前稳定参考仍以 3A 为主**（3B 非当前强制停靠点）。

### 八、配套：把净化结果变成可验证

- **导出展开 URDF**：脚本支持 xacro→展开，对齐「源码以为」与「Gazebo/Pinocchio 真吃到」。  
- **Stage 2 几何校验**：golden + 脚本锁定 stand 下 hip/foot 世界坐标、rail parent 是否为 `base_link`。  
- **`check_urdf_shift_boundary.py` 升级**：校验 `base_offset_joint` 边界、`base_link` 承载 trunk inertial、`base_link_cad` 不再承载 inertial、rail parent 等——**规则可执行化**。

### 九、当前「净化后」稳态（简述）

**已净化**：CAD 偏移封在 `base_offset_joint`；双 base 已建立；腿语义在 `base_link`；trunk inertial 在 `base_link`；`base_link` 明确为未来主根。  

**未彻底完成**：`base_link_cad` 仍扛 visual；collision 最终归属与 Gazebo 链仍在收口；`base_link_cad_fixed` 仍为 identity；最外层 `urdf_shift_*` 尚未删除。  

工作**未完成但关键一半已扶正**：主语义清晰、历史包袱隔离，后续可低风险继续收 collision 与 offset。

### 十、意义（四条）

1. **控制根扶正**：base frame、腿基座、Pinocchio root、gait body frame 可统一围绕 `base_link`。  
2. **历史偏移封边**：不再是无处不在的幽灵变量。  
3. **腿/机身语义分离**：安装、惯性、CAD visual 分层清晰。  
4. **删 CAD 偏移有路径**：先主根、再 collision、最后 `base_offset_joint` 与 mesh 原点。

### 十一、一句话总结

**不是粗暴删掉 CAD 偏移，而是先把控制根、腿安装根、动力学根、CAD 壳层拆开，再逐层把应属于 `base_link` 的迁回 `base_link`。**  
核心成果：模型从「历史补丁拼起的 URDF」转向**语义清楚、可继续演进**的 URDF。

---

## 2026-04-07 21:52 rail 符号/足端帧/限位语义统一 + 回归测试闭环（Position ↔ IK ↔ MPC/WBC）

- **结论（最重要）**
  - 修复了 `KinematicsSolver` 与 URDF 在 `rail_joint` 方向定义上的**符号冲突**：URDF `axis="-1 0 0"`，求解器侧 FK/IK 统一改为沿 **-X** 平移，消除“同一个 `rail_m` 语义相反”的高危假象源。
  - 将 `KinematicsSolver` 的末端点定义统一为 **`tibia_link` frame 原点**，不再使用 `shin_xyz` 作为足端/接触代理偏移点，和 `MPCRobotController` 侧的接触代理帧定义对齐，避免“控制点/优化点不是同一点”的隐性分叉。
  - 停止用 YAML 的统一 `rail.min/max` 覆盖 per-leg rail 方向语义：IK 的 rail 限位保持由 `leg_parameters.py` 的每腿限位（正/负向）决定，避免配置把物理语义冲掉。
  - 补齐了**可证伪**的几何回归验证脚本：`solve_fk()` 与 Pinocchio 的 `*_tibia_link` 在 `base_link` 坐标系下对齐误差达到 `1e-13 m` 量级（数值噪声级），证明“改 IK 不改 MPC”路线在几何层面闭环成立。

- **工程改动：rail 方向符号 + 足端定义统一（IK/FK）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/kinematics_solver.py`
    - `rail_translation_axis` 从 `+X` 改为 `-X`，匹配 URDF `axis="-1 0 0"`。
    - FK/IK 末端点从 `T @ shin_xyz` 改为 `tibia_link` 原点（不再对 `shin_xyz` 施加末端偏移）。

- **工程改动：per-leg rail 限位语义保护（不被 YAML 覆盖）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
    - `_apply_config_joint_limits_to_ik()`：仅同步 `coxa/femur/tibia`（haa/hfe/kfe）旋转关节限位；不再统一覆盖 `rail` 限位，rail 仍来自 `leg_parameters.py` 的每腿定义。

- **工程改动：执行层 clamp 与 IK 同步刷新入口**
  - 文件：`src/dog2_motion_control/dog2_motion_control/joint_controller.py`
    - 新增 `reload_joint_limits()`：从 `LEG_PARAMETERS` 重新加载 `self.joint_limits`。
  - 文件：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
    - `reload_config_from_file()`：在重载配置后尝试调用 `joint_controller.reload_joint_limits()`，避免“IK 改了，执行层 clamp 还是旧表”的半 reload 假象。

- **工程改动：最小几何回归测试（FK/IK 自洽 + FK vs Pinocchio 对齐）**
  - 文件：`src/dog2_motion_control/scripts/test_kinematics_consistency.py`（新增）
    - FK/IK 自洽（确定性）：比较 `FK(global)->to_leg_frame` 与 `_forward_local(local)`，`geom_local_max ~ 1e-17 m`（PASS）。
    - FK vs Pinocchio 对齐：将 Pinocchio 的 `*_tibia_link` 世界坐标通过 `base_link` frame 逆变换映射到 `base_link` 坐标系后与 `solve_fk()` 对比，最大误差 `~2e-13 m`（PASS）。
    - 增加按腿统计、worst-case 样本输出、PASS/WARN/FAIL 阈值；并在未 source 工作区时自动补齐 `sys.path` 以便直接运行脚本。

- **已知风险（未在本轮强行修复，避免引入新假象）**
  - `solve_ik()` 的数值收敛性在“近站立扰动采样”下仍存在较高失败率：这反映的是当前 DLS+seeds 的收敛域/初值策略问题，而非几何链路不一致。建议作为“阶段 2.5：IK 收敛性整治”单独推进，再进入阶段 3（在线 rail 冗余闭环放开与 effort rail lock 模式化）。

## 2026-03-31 11:14 Gazebo 质量缩放 A/B 试验基建 + 导轨跟踪瓶颈定位
- **结论（最重要）**
  - `gz_ros2_control` 的 `position_proportional_gain` 是“位置误差→期望速度”的系数（而非力矩 PID 的 P），工程上不应暴力拉高到几十；在本环境中该参数也不会从 controllers YAML/URDF 自动注入，必须运行时 set。
  - A/B 试验显示：`mass_scale=0.25` 时 rail 跟踪压力显著降低，而 `mass_scale=1.0` 在同样工况下可复现 rail tracking error 瞬态暴涨并触发 slip 急停，说明真实载荷下导轨伺服带宽不足是关键矛盾之一（与“无限带宽 IK 指令”形成撕裂）。

- **工程改动：质量缩放参数化（便于可重复 A/B）**
  - 文件：`src/dog2_description/urdf/dog2.urdf.xacro`
    - 新增 xacro 参数 `mass_scale`（默认 `1.0`），并将 `base_link` 及四条腿主要 link 的 `mass`、`inertia` 按比例缩放（用于 A/B，不改变几何）。
  - 文件：`src/dog2_motion_control/launch/spider_gazebo_complete.launch.py`
    - 新增 launch 参数 `mass_scale`，并通过 xacro mappings 注入 URDF 生成链路。
    - 用法：
      - `ros2 launch dog2_motion_control spider_gazebo_complete.launch.py mass_scale:=0.25`
      - `ros2 launch dog2_motion_control spider_gazebo_complete.launch.py mass_scale:=1.0`

- **工程改动：自动注入 gz_ros2_control 增益（避免“忘记 set 参数”污染对照）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/gz_gain_setter.py`（新增）
    - 启动后等待 `/gz_ros2_control` 出现，自动设置 `position_proportional_gain`。
  - 文件：`src/dog2_motion_control/setup.py`
    - 注册 console script：`gz_gain_setter = dog2_motion_control.gz_gain_setter:main`
  - 文件：`src/dog2_motion_control/launch/spider_gazebo_complete.launch.py`
    - 新增 launch 参数 `p_gain`（默认 `1.5`），启动时自动执行一次注入。
    - 用法：`ros2 launch ... p_gain:=1.5`

- **步态/控制侧收敛（用于减少 rail 指令带宽与触发概率）**
  - 文件：`src/dog2_motion_control/config/gait_params.yaml`
    - 收敛步态基线：`stride_length 0.10→0.06`、`stride_length_max 0.10→0.08`、`stride_height 0.05→0.03`、`cycle_time 1.0→1.2`
    - 新增/启用导轨限速：`control.max_rail_velocity: 0.15`（50Hz 下约 3mm/周期）

- **实验记录（摘要）**
  - `mass_scale=1.0` + `p_gain=1.5` + `vx=0.06`：可复现 rail tracking error 瞬态（如 25mm/29mm 量级）并触发 `CRITICAL: Rail slip detected!`，进入 emergency descent。
  - `mass_scale=0.25`：rail tracking 报警显著减少（仍需在统一口径下跑满时长做最终统计）。

## 2026-03-30 20:29 Gazebo 导轨急停去抖 + 急停锁定无阶跃 + 步幅上限解锁
- **背景现象（Gazebo）**
  - `rail_position_controller` 在仿真中存在毫米级稳态跟踪误差，原先以 `±0.5mm` 判定“导轨滑移”会导致机器人一进入步态就急停。
  - 急停分支将导轨目标硬锁到 `0.0m`，在存在位置偏差时形成位置阶跃（step input），进一步放大冲击与误触发风险。
  - `GaitGenerator` 将 `stride_length` 同时用作运行时自适应步长上限，导致 YAML 里 `stride_length=0.02` 时步幅被永久卡死为 2cm（无论 `/cmd_vel` 多大）。

- **修复 1：导轨滑移监控参数化 + 去抖（治标）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/joint_controller.py`
  - 移除硬编码 `RAIL_SLIP_THRESHOLD_M=0.0005`，改为 ROS2 参数：
    - `rail_slip_threshold`（默认 `0.005`，5mm）
    - `rail_slip_patience`（默认 `3` 帧）
  - 新增 `rail_slip_counters`：仅当某导轨误差连续超阈值达到 `patience` 才触发 CRITICAL 急停；误差回落则清零计数。

- **修复 2：急停导轨锁定“无阶跃”（治标）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/joint_controller.py`
  - 重构 `lock_rails_with_max_effort()`：急停锁定目标优先使用 `current_joint_states` 的测量位置；测量缺失时回退 `last_rail_targets`，避免硬锁 `0.0m` 产生冲击。

- **修复 3：步幅上限解耦（治本）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/gait_generator.py`
    - 新增 `GaitConfig.stride_length_max`，并将 `_stride_length_limit` 迁移为使用 `stride_length_max`（缺省回退到 `stride_length`）。
  - 文件：`src/dog2_motion_control/dog2_motion_control/config_loader.py`
    - 默认配置与校验支持 `gait.stride_length_max`，并映射到 `GaitConfig`。
  - 文件：`src/dog2_motion_control/config/gait_params.yaml`
    - 将基线步态参数调整为更明显的“可观察大步”：
      - `stride_length=0.10`
      - `stride_length_max=0.10`
      - `stride_height=0.05`
      - `cycle_time=1.0`

- **临时策略（用于验证导轨锁定工况）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
  - 在 `CRUISE_3DOF` 且 `current_rail_alpha≈0` 时，将 `rail_hint` 临时强制为 `0.0m`，便于先在“导轨锁定”工况下验证步态稳定性与急停逻辑不误触发。

- **验证要点**
  - 启动仿真后持续发布 `/cmd_vel`（例如 `x≈0.12 m/s` 起步），应不再因单次瞬态 rail 误差立即触发急停。
  - `/spider_debug_info` 中 `joint_positions.rail_m`、足端目标与步幅应随速度输入显著增大，不再被 2cm 上限锁死。

## 2026-03-30 16:07 越障阶段数量与进度语义对齐修复
- **状态机阶段数量修复（必须修改项）**
  - 将 `CrossingStateMachine` 的阶段注释与语义从 **8 stages** 修正为 **9 stages**：`src/dog2_mpc/include/dog2_mpc/crossing_state_machine.hpp`。
  - 修正 `CrossingStateMachine::getProgress()`：进度严格映射到 `APPROACH..CONTINUE_FORWARD` 的 **0~1**，`COMPLETED` 进度固定为 **1.0**，避免进度越界：`src/dog2_mpc/src/crossing_state_machine.cpp`。
- **文档同步**
  - 将 `dog2_crossing_design_guide.md` 中关于 “8 stages/8个阶段” 的描述统一为 **9 stages**，避免文档与实现继续漂移。
- **工程验证**
  - `colcon build --packages-select dog2_mpc --symlink-install` 编译通过。

## 2026-03-30 17:06 越障 guard 与 rail soft bound 理论加固
- **stable transition graph 加固（rail tracking + 支撑稳定双 guard）**
  - 在 `src/dog2_mpc/src/crossing_state_machine.cpp` 中将 `canTransitionToNext()` 改为：除了原有阶段完成条件（progress guard）外，再叠加
    - `rail_tracking_error < rail_tracking_error_threshold_`（默认 0.005m，支持 ROS2 参数覆盖）
    - 支撑稳定 guard（见下条）
  - 相关阈值接口/成员见 `src/dog2_mpc/include/dog2_mpc/crossing_state_machine.hpp`。
- **两足支撑（退化线段）死锁修复：使用 Capture Point 动态稳定裕度**
  - 先前针对 `contact_count < 3` 的支撑裕度 guard 采用静态几何 1D/X 近似，且在 `contact_count==2`（Trot 对角线步态、支撑退化为线段）时会导致 guard 永真/永假，进而 stage transition 死锁。
  - 本次在 `src/dog2_mpc/src/crossing_state_machine.cpp`
    - 将 `computeSupportPolygonMargin()` 的退化线段分支改为返回 **CP 到当前两足连线段的垂距**（而非纯几何到 x 范围距离）
    - 并在 `contact_count==2` 时在 `canTransitionToNext()` 中以 `d_cp < 0.025m` 作为动态放行条件
  - CP 定义：`CP = [x_com + sqrt(h/g)*vx_com, y_com + sqrt(h/g)*vy_com]`，其中 `h = max(1e-6, z_com)`，`g=9.81`。
- **rail soft bound 约束升级为真正可行的“软约束”（Slack 解耦 + 精确惩罚）**
  - 目标：避免 OSQP 在边界附近出现 `Infeasible/Unbounded`，同时又允许微小越界由 slack 吸收，并通过 exact penalty 逼近硬约束极限。
  - 在 `src/dog2_mpc/src/mpc_controller.cpp`：
    - 取消原先“5mm 宽容区当软约束”的硬化做法
    - 引入 **rail 下界/上界**两套解耦 slack 变量：
      - `s_lower(k,i) >= 0`
      - `s_upper(k,i) >= 0`
    - 约束形式：
      - 下界：`d_i + s_lower(k,i) >= d_min(i)`
      - 上界：`d_i - s_upper(k,i) <= d_max(i)`
    - **非负约束（s >= 0）与上界（近似无穷）**：新增对应不等式行进入 QP 的 `A/l/u`，确保 OSQP 不会因为 slack 未被约束而触发 `OSQP_UNBOUNDED`。
    - exact penalty：
      - slack 惩罚从仅二次项（Hessian）升级为：同时在梯度向量 `q` 中对 slack 加入一次项强惩罚（`current_slack_weight_`）。
  - 变量结构与 penalty weight 通过：
    - `src/dog2_mpc/include/dog2_mpc/mpc_controller.hpp` / `src/dog2_mpc/src/mpc_controller.cpp` 实现（新增 `current_slack_weight_` 与 `setSlackLinearWeight()`）。
- **exact penalty 权重动态调参（禁止硬编码）**
  - 在 `src/dog2_mpc/src/mpc_node_complete.cpp` 与 `src/dog2_mpc/src/mpc_node_16d.cpp`：
    - 新增 ROS2 参数 `slack_linear_weight`（默认 `1e5`）
    - 在控制循环中动态读取并通过 `mpc_controller_->setSlackLinearWeight(...)` 下发给 QP 的 exact penalty

**工程验证（构建 + 关键可执行测试）**
- `colcon build --packages-select dog2_mpc --symlink-install` 编译通过
- 运行：
  - `./test_sliding_constraints`（通过）
  - `./test_crossing_mpc_simple`（通过，crossing + slack 相关 QP 能成功 OSQP 求解）
  - `./test_crossing_mpc`（通过）
  - `./test_crossing_system`：在当前玩具/参数下表现为超时（但未观察到 OSQP 求解结构级崩溃）

## 2026-03-18 10:47 今日工作概述
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

## 2026-03-19 10:47 今日工作概述
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

---

## 2026-03-19 16:05 urdf_shift 边界收敛与自动校验

针对“全局 urdf_shift 多层补偿导致双重偏移风险”的问题，落实为“唯一补偿层 + 自动校验”的工程约束：
- **唯一补偿层**：`urdf_shift` 仅允许出现在 `base_offset_joint`，其余 origin 均使用 base_link-local 常量。
- **URDF 边界自检脚本**：新增 `check_urdf_shift_boundary.py`，通过 xacro 展开后验证：
  - `base_link` visual/inertial 原点保持固定
  - 四条腿根 `*_rail_joint` 原点落在期望范围
- **避免双重补偿**：在 `dog2.urdf.xacro` 中明确写入注释边界，禁止 TF/控制/外参层再做补偿。

> 该修复确保 CAD 历史偏移只在一个层级出现，避免后续维护人员误在控制层重复修正。

## 2026-03-19 14:32 终局一致性重构（dog2 Spider Topology）

在本轮迭代中，我们把问题从“局部参数调不对”提升到了“系统语义与物理真实是否同构”的层面：
**保持物理拓扑不变，只在算法与描述层做可验证的映射重构**。最终确立并固化以下单一真源：
- 机头朝向：`-X`
- 逆时针腿序：`1=LF, 2=LH, 3=RH, 4=RF`
- 控制与模型命名必须与上述物理事实严格一致

### Architecture Refactoring

- **URDF 与 CAD 偏移彻底解耦**
  - 识别并隔离 CAD 导出携带的全局平移：`urdf_shift_x/y/z`。
  - 在 `dog2.urdf.xacro` 的实例化层采用 **one-time compensation**（一次性补偿）策略，避免把 CAD 偏移泄漏到控制层。
  - 关键原则落地：URDF 负责把“几何壳体”落在正确物理位置；控制层/算法层仅消费物理一致的坐标，不再背负 CAD 历史包袱。

- **HAL 语义化命名重构（从编号到解剖学语义）**
  - 从 CAD 风格无语义编号（如 `j1/j11/...`）迁移到生物力学语义接口：
    - `rail`（导轨）
    - `coxa`（基节）
    - `femur`（腿节）
    - `tibia`（胫节）
  - 同步重构 `dog2.urdf.xacro` 宏定义与 `ros2_controllers.yaml` 接口命名，实现：
    - **硬件实例 ID** 与 **算法逻辑 ID** 解耦
    - 上层控制器不再依赖 CAD 的偶然命名细节

- **拓扑一致性闭环（Model ↔ Control ↔ Gait）**
  - 统一腿序映射：`leg1=lf`, `leg2=lh`, `leg3=rh`, `leg4=rf`。
  - 与 `joint_names.py`、`leg_parameters.py`、`gait_params*.yaml` 做同源对齐，避免跨文件“半重构”导致错腿驱动。

### Bug Fixes

- **实例化层前后腿参数调用歧义修复**
  - 明确 CAD 网格关系是“左右镜像主导”，而非“前后镜像可直接复用”。
  - 修复腿实例化中 `prefix` 与 `leg_num` 的张冠李戴问题，消除 `LF/LH/RH/RF` 的错绑风险。

- **Xacro 解析崩溃修复（XML 1.0 注释陷阱）**
  - 根因：注释块内出现连续减号 `--`，触发 `XML parsing error: invalid token`。
  - 修复：重写分隔注释符号（使用 `=` / `*` 等安全字符），并建立注释语法红线，防止再次引入静态解析失败。

### Tech Debt Cleared

- 清除“CAD 导出坐标即真值”的隐性技术债，建立“CAD 仅提供网格，系统坐标由机器人软件栈定义”的工程边界。
- 清除“编号即语义”的命名技术债，统一为可读、可审计、可追踪的语义化关节体系。
- 清除“多源配置各自为政”的一致性债务：将腿序映射固化为跨 URDF / YAML / Python 的共享契约。

### Next Steps

1. **自动化一致性测试**
   - 增加启动前拓扑自检脚本：校验 URDF、控制器 joints、步态 `leg_sequence`、`leg_base_positions` 的符号与顺序一致性。
2. **TF 与外参防呆机制**
   - 在传感器外参与 TF 广播链路中加入“偏移来源标记”，硬性禁止二次补偿（Double Offset）。
3. **面向越障任务的控制层验证**
   - 在 Gazebo Fortress 中执行低姿态越障回归：验证 rail 重心迁移 + crawl gait 在窄窗框工况下的稳定裕度。
4. **MPC/WBC 接口前置整理**
   - 以 `rail/coxa/femur/tibia` 语义接口为统一输入，降低后续约束建模与调参复杂度。

> 在非标构型机器人中，最昂贵的 bug 往往不是“算法算错”，而是“语义错绑”。
> 本轮工作的价值在于：把物理真实与算法逻辑做了工程化解耦，并建立了可持续验证的映射闭环。

---

## 2026-03-19 继续：URDF 偏移边界防呆接入构建测试

本轮将“唯一补偿层”从文档约束进一步升级为**默认启用的可执行测试**，并补充脚本参数化能力，降低浮点误报风险。

### 💻 工程实现
- **脚本参数增强（防误报）**
  - `check_urdf_shift_boundary.py` 新增：
    - `--tolerance`（兼容别名 `--tol`）用于设置容差。
    - `--strict` 启用严格模式（容差固定为 `1e-6`）。
  - 输出信息增加模式与容差打印，失败信息携带 `tol`，便于快速定位。

- **CMake 默认开启 + 可显式关闭**
  - 在 `dog2_description/CMakeLists.txt` 增加开关：
    - `DOG2_ENABLE_URDF_SHIFT_BOUNDARY_CHECK`（默认 `ON`）。
  - 在 `BUILD_TESTING` 且开关开启时，注册 `ctest`：
    - `dog2_urdf_shift_boundary_check`
    - 调用脚本并默认使用 `--strict`。
  - 对缺失 `python3` / `xacro` 场景给出 warning 并跳过，避免阻塞极简环境。

- **安装集成**
  - 将 `scripts/check_urdf_shift_boundary.py` 纳入 `install(PROGRAMS ...)`，确保安装后环境也可直接运行。

### ✅ 验证结果
- 本地执行脚本通过：
  - `[PASS] URDF shift boundary checks passed`。
- 新增检查满足“默认安全、按需关闭”的工程实践目标。

### 🚀 后续建议
1. 若仓库后续引入正式 CI（如 GitHub Actions / GitLab CI），可直接在 `colcon test` 流程中复用该 `ctest`，形成“本地 + CI”双保险。
2. 若 CAD 再导出导致基准变化，优先更新期望常量并保留严格模式，确保偏移边界语义不回退。

---

## 2026-03-19 继续：关节被动耗散与URDF风格一致性加固

本轮在“偏移边界已收敛”的基础上，继续处理仿真稳定性与模型可维护性：
通过补齐关节 `dynamics`、参数化接触模型、统一材质命名和角度常量，降低高频抖动与配置漂移风险。

### 💻 工程实现
- **关节被动耗散补齐（全可动关节）**
  - 在 `dog2.urdf.xacro` 中为所有可动关节统一加入 `<dynamics damping/friction>`：
    - `rail (prismatic)`：`damping=0.25`、`friction=0.05`
    - `coxa (revolute)`：`damping=0.12`、`friction=0.02`
    - `femur/tibia (revolute)`：`damping=0.15`、`friction=0.02`
  - 参数全部抽为 xacro property，便于后续批量联调与版本对比。

- **接触模型参数化与保守基线下调**
  - 将足端（当前以 `tibia_link` 代理）接触参数抽为统一属性：
    - `foot_contact_mu1/mu2=1.0`
    - `foot_contact_kp=2.0e5`（由 `1.0e6` 下调）
    - `foot_contact_kd=50.0`（由 `100.0` 下调）
    - `minDepth=0.0005`、`maxVel=0.05`
  - 增加注释说明：后续若引入独立 foot link，再迁移接触定义。

- **URDF 可读性与一致性清理**
  - 修复多处 `material name=""` 空命名，替换为具名材质：
    - `mat_base_gray`、`mat_blue`、`mat_white`。
  - 统一角度常量来源，避免 `1.5708 / 3.1416 / 3.14159265` 混用：
    - 增加 `pi`、`half_pi` 属性
    - `rpy` 与 `hip_rpy` 统一改为引用常量。

### ✅ 验证结果
- 严格模式边界检查通过：
  - `python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict`
  - 输出：`[PASS] URDF shift boundary checks passed (mode=strict, tol=1e-06).`

### 🧭 调参原则（已固化）
1. `joint damping` 与控制器 `D` 存在等效叠加，联调时避免双侧同时大幅上调。
2. 若低速跟踪出现“爬行/死区”，优先小幅下调 `friction` 再观察。
3. 分层联调顺序保持：`rail -> coxa -> femur/tibia -> 全身步态`。

---

## 2026-03-19 继续：控制器命名/关节语义一致性修复与启动链路收敛

本轮围绕“控制器激活超时 + 上层节点断连重试”进行了针对性收敛，核心是统一 **URDF / ros2_control / motion_control** 三层命名与时钟语义，清理导致握手失败与运行时崩溃的配置偏差。

### 💻 工程实现
- **xacro 常量冲突修复**
  - 将 `dog2.urdf.xacro` 中自定义 `pi` 重命名为 `PI_CONST`，并同步替换引用，避免与 xacro 内置全局符号冲突警告（`redefining global symbol: pi`）。

- **ROS 包清单规范化**
  - `dog2_demos/package.xml` 中 `test_dependency` 全量替换为 format=3 合法标签 `test_depend`，消除 colcon 解析告警。

- **控制器关节命名与 URDF 对齐**
  - `dog2_description/config/ros2_controllers.yaml`：
    - 12个旋转关节从 `*_haa/*_hfe/*_kfe` 统一改为 `*_coxa/*_femur/*_tibia`。
    - 导轨关节从 `j1~j4` 统一改为 `lf/lh/rh/rf_rail_joint`。
  - `dog2_motion_control/dog2_motion_control/joint_names.py`：
    - `REVOLUTE_JOINT_TYPES` 改为 `['coxa','femur','tibia']`。
    - 相关辅助函数与说明同步更新为新语义命名。

- **关节限位键值同步修复（KeyError 根因）**
  - `dog2_motion_control/dog2_motion_control/leg_parameters.py` 中 `joint_limits` 键从 `haa/hfe/kfe` 同步迁移为 `coxa/femur/tibia`，修复 `KeyError: 'coxa'`。

- **仿真时钟与生成姿态策略收敛**
  - `spider_gazebo_complete.launch.py`：
    - `spider_controller_node` 参数 `use_sim_time` 从 `False` 改为 `True`，避免仿真时钟与系统时钟混用导致状态判定抖动。
    - spawner `-c` 目标恢复为 `/controller_manager`（与 Humble + gz_ros2_control 默认行为一致）。
  - `dog2.urdf.xacro`：移除 `world_to_base_joint` 固连（避免与 spawn pose 职责重叠），保留由 launch 统一管理初始高度。

### 🐛 问题链路复盘
- **初始现象**：`Switch controller timed out`、`Not existing command interfaces`、上层控制器持续重连与紧急姿态。
- **分层根因**：
  1. 控制器 joints 与 URDF joints 命名体系不一致（历史命名残留）。
  2. motion_control 内部关节类型和限位字典键未完成同源迁移，触发运行时 `KeyError`。
  3. 启动阶段 controller_manager 目标名与插件实际行为不一致，导致 spawner 等待错误服务。
  4. `use_sim_time=False` 放大了仿真联调中的连接状态误判风险。

### ✅ 当前状态与结论
- 命名链路（URDF ↔ controllers.yaml ↔ Python 控制层）已完成同源统一。
- 运行时字典键崩溃（`KeyError: 'coxa'`）已消除。
- 启动链路已回归 ROS 2 Humble + gz_ros2_control 的默认 manager 约定。
- 后续如仍出现控制器激活超时，优先从仿真步进/启动负载角度排查，而非继续回退命名层。

### 🚀 下一步计划
1. 以最小命令集回归验证控制器状态机（`list_controllers`、`joint_states`、轨迹话题发布）。
2. 在控制器全部可激活后，再开展落地高度与接触参数二次调优（避免混叠问题源）。
3. 将“命名一致性 + 时钟一致性 + manager 目标一致性”沉淀为启动前自动检查项。

---

## 2026-03-19 22:02 继续：仿真时钟启动竞态修复与调试键名一致性收口

本轮完成了“控制器已激活但上层仍误判时钟未推进”的竞态修复，并修正调试信息发布路径中的历史键名残留，消除 `KeyError: 'haa'` 崩溃点。

### 💻 工程实现
- **调试关节键名统一（直接崩溃点修复）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
  - `_publish_debug_info()` 中关节读取键从旧命名：
    - `haa / hfe / kfe`
  - 统一迁移为当前语义命名：
    - `coxa / femur / tibia`
  - 结果：消除启动后调试发布分支触发的 `KeyError: 'haa'`。

- **起立状态机时钟竞态防护**
  - 文件：`src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
  - 调整 `start()` 与 `_execute_standup_trajectory()`：
    1. 当 `/clock` 尚未就绪（`now.nanoseconds == 0`）时，不再“跳过起立并直接进入 READY”。
    2. 改为保持 `STANDING_UP` 并等待时钟同步，避免状态机提前越级。
    3. 增加一次性等待日志标志 `_clock_wait_logged`，避免日志刷屏。
    4. 时钟就绪后再初始化 `standup_start_time`，按既定 minimum-jerk 轨迹执行起立。

- **Gazebo 自动运行确认**
  - 文件：`src/dog2_motion_control/launch/spider_gazebo_complete.launch.py`
  - 已确认 GUI / headless 两条分支均携带 `-r`，默认自动推进仿真时钟。

### ✅ 结果与结论
- 控制器链路维持可激活状态（`joint_state_broadcaster / joint_trajectory_controller / rail_position_controller`）。
- 上层节点不再因启动瞬间时钟未同步而误跳过起立流程。
- 调试路径键名与 URDF/控制器命名体系保持一致，运行时字典访问稳定。

### 🧪 建议复现命令
```bash
cd ~/aperfect/carbot_ws
rm -rf build/dog2_motion_control install/dog2_motion_control
colcon build --packages-select dog2_motion_control --symlink-install
source install/setup.bash
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py
```

### 🚀 后续计划
1. 记录一次完整启动日志，确认“等待时钟同步 -> 起立完成 -> READY_FOR_MPC”三段状态转移连续可复现。
2. 在 debug 模式下增加轻量状态摘要（状态机阶段、dt、phase）用于联调可视化。
3. 将 `/clock` 就绪检查抽成可复用启动健康检查函数，供后续控制节点复用。

---

## 2026-03-27 04:44 冗余IK基线参数化与离线越障分析基建

本轮工作聚焦“导轨冗余自由度可控化 + 可量化评估”两条主线：
将 3-DoF 解析近似 + 导轨参数化基线从硬编码推进到可配置形态，并补齐离线越障分析工具链，为后续 QP/WBC 深化提供可复现实验地基。

### 💻 工程实现
- **IK 正则化求解基线增强（KinematicsSolver）**
  - 在 `kinematics_solver.py` 中新增模块化求解子函数：
    - `_compute_local_jacobian`（局部雅可比数值计算）
    - `_solve_3r_with_fixed_rail`（固定导轨下 DLS + 正则 3R 求解）
    - `_rail_regularization_cost`（导轨中位/姿态偏差/连续性综合代价）
  - 引入 `_last_solution` 作为每条腿的时序缓存，显著抑制导轨和关节解的帧间抖动。

- **步态相位先验接入（GaitGenerator → Controller → IK）**
  - 在 `gait_generator.py` 新增 `get_phase_progress_scalar(leg_id)`，输出约 `[-0.5, 0.5]` 的步长相位标量。
  - 在 `spider_robot_controller.py` 中使用该标量构建 `rail_hint`，并通过 `solve_ik(..., rail_offset=rail_hint)` 注入导轨先验，形成“步态意图驱动冗余分配”的闭环。

- **离线越障分析脚本（新增）**
  - 新增 `src/dog2_motion_control/scripts/obstacle_analysis.py`：
    - 复用现有 `GaitGenerator + KinematicsSolver` 做离线时域仿真
    - 支持台阶与窗框型障碍参数化注入
    - 输出关键指标：IK 失败数、导轨归一化利用率、关节解帧间变化量（连续性）
  - `setup.py` 注册脚本入口：`obstacle_analysis = scripts.obstacle_analysis:main`。

### ✅ 验证结果
- 已对核心改动文件执行静态检查，未引入新的 linter 错误。
- 导轨先验 + 状态缓存链路已打通，具备后续参数联调与离线指标对比能力。

### 🚀 下一步计划
1. 将 IK 正则参数（阻尼、导轨中位惩罚、连续性惩罚等）统一纳入 YAML 并支持运行时更新。
2. 增加离线报告的场景批处理与 CSV 导出，建立参数扫描基准集。
3. 在接触模型层补充足端接触一致性对照实验，评估导轨策略在高差落足下的稳定裕度。

---

## 2026-03-27 15:24 落足缓冲过渡（摆动末段 10%）首版落地

本次完成“第一步：落足缓冲过渡代码编写”，在保持无状态锚点法主体不变的前提下，最小侵入式加入摆动相末段 Z 轴缓冲，目标是从轨迹源头降低落足硬冲击。

### 💻 工程实现
- **落足缓冲参数化接入（YAML）**
  - 在 `src/dog2_motion_control/config/gait_params.yaml` 的 `gait` 下新增：
    - `foot_landing_buffer.enable`
    - `foot_landing_buffer.swing_phase_ratio`
    - `foot_landing_buffer.poly_order`
    - `foot_landing_buffer.target_landing_vel_z`
  - 默认开启 `enable: true`，便于直接仿真观察；可一键关闭回退。

- **配置加载链路打通（ConfigLoader）**
  - `config_loader.py` 默认配置增加 `gait.foot_landing_buffer`。
  - 在 gait 参数校验中增加对缓冲参数的范围校验：
    - `swing_phase_ratio ∈ (0,1)`
    - `poly_order ∈ {3,5}`
    - `target_landing_vel_z ∈ [0,0.3]`
  - `get_gait_config()` 已将上述参数注入 `GaitConfig`。

- **轨迹生成器缓冲逻辑（GaitGenerator）**
  - 复用现有 `_apply_landing_buffer()`：仅在摆动相最后 `swing_phase_ratio` 时间窗生效。
  - 采用 5 次多项式（最小冲击）`s=6t^5-15t^4+10t^3`（可降级 3 次）。
  - 仅修正 Z 轴，不改动 X/Y 迈步轨迹，确保对水平步态最小扰动。
  - 增加日志：首次进入缓冲窗口 info + 连续 debug（buffer_phase/Z pos/Z vel）。

- **腿序语义修正**
  - `gait_generator.py` 中腿序日志映射统一为：`1=LF, 2=LH, 3=RH, 4=RF`。

### ✅ 当前状态
- 代码与参数链路已落地，可直接编译并在 Gazebo 中观察落足段速度收敛表现。
- 回退机制已具备：仅需将 `foot_landing_buffer.enable` 设为 `false` 即可恢复旧逻辑。

### 🚀 下一步
1. 先做仿真验证（日志 + 可视观感 + 话题数据）确认缓冲生效与稳定性收益。
2. 若验证通过，再进入足端碰撞代理与接触参数微调。

---

## 2026-03-27 15:24 落足缓冲过渡（第一步）落地

本轮按“最小侵入、可回退”的原则，完成摆动相末段落足缓冲改造：
在 **摆动相最后 10% 时间窗** 内对足端 Z 轨迹施加 5 次多项式缓动（最小冲击），用于将落足瞬间速度平滑收敛至近零，降低硬接触冲击。

### 💻 工程实现
- **步态发生器缓冲逻辑接入**
  - 文件：`src/dog2_motion_control/dog2_motion_control/gait_generator.py`
  - 在摆动相末段新增 `_apply_landing_buffer()`：
    - 默认使用 5 次多项式 `s(t)=6t^5-15t^4+10t^3`
    - 兼容 3 次多项式降级分支（配置可选）
    - 仅修正 Z 轴（X/Y 水平轨迹保持不变）
  - 新增缓冲阶段日志：
    - 进入缓冲窗口时打印 `enter landing buffer phase`
    - debug 输出 `buffer_phase / Z pos / Z vel` 便于联调

- **参数化配置打通（含回退开关）**
  - 文件：`src/dog2_motion_control/config/gait_params.yaml`
  - 新增配置块：
    - `foot_landing_buffer.enable`
    - `foot_landing_buffer.swing_phase_ratio`
    - `foot_landing_buffer.poly_order`
    - `foot_landing_buffer.target_landing_vel_z`
  - 文件：`src/dog2_motion_control/dog2_motion_control/config_loader.py`
    - 增加默认值、参数校验与 `GaitConfig` 映射，确保 YAML→运行时参数完整生效

- **腿序标识对齐（日志语义统一）**
  - 文件：`src/dog2_motion_control/dog2_motion_control/gait_generator.py`
  - `LEG_ORDER` 调整为项目固化约定：`1=LF, 2=LH, 3=RH, 4=RF`

### ✅ 当前状态
- 缓冲逻辑已接入轨迹生成链路，并具备配置开关回退能力。
- 当 `foot_landing_buffer.enable=false` 时，可无代码改动回退到旧轨迹行为。

### 🧪 建议验证
1. 观察日志是否出现各腿 `enter landing buffer phase`。
2. 关注落足前 `Z vel` 是否平滑趋近 `target_landing_vel_z`（默认 `0.01 m/s`）。
3. Gazebo 观察落足冲击是否减弱、底盘高频振动是否下降。

---

## 2026-03-27 15:39 坐标系注释补全与 default_nominal 符号修正

针对默认名义落足点在离线/兜底路径下的左右符号一致性风险，本轮完成最小修复并补充显式注释，防止后续误改。

### 💻 工程实现
- 文件：`src/dog2_motion_control/dog2_motion_control/gait_generator.py`
- 在 `default_nominal` 上方新增坐标系说明注释：
  - `base_link` 采用标准 ROS 右手系：`X前、Y左、Z上`
  - 左侧腿（LF/LH）应满足 `Y>0`
  - 右侧腿（RF/RH）应满足 `Y<0`
- 修正 `default_nominal` 的左右腿 Y 符号：
  - `lf/lh`: `+0.2`
  - `rf/rh`: `-0.2`

### ✅ 结果
- 默认（未传入 `nominal_positions`）轨迹基准与 URDF / leg_base_positions 的坐标语义一致。
- 正常主链路（FK 标定注入 nominal）保持不受影响；离线脚本/兜底路径一致性提升。

---

## 2026-03-27 16:10 离线越障分析批处理与 CSV 导出

为提升参数联调效率，本轮在离线分析工具中补齐“场景批跑 + 可落盘指标”能力，减少手工重复执行与结果抄录成本。

### 💻 工程实现
- 文件：`src/dog2_motion_control/scripts/obstacle_analysis.py`
- 新增批处理参数：
  - `--batch`：启用预设场景批量执行
  - `--scenarios`：逗号分隔场景列表（`flat/step/window/mixed`）
  - `--csv-out`：将每腿指标导出为 CSV
- 场景机制：
  - `flat`：无障碍基线
  - `step`：仅台阶扰动
  - `window`：仅窗框扰动
  - `mixed`：台阶 + 窗框组合
- 输出结构改造：
  - 统一按 `scenario + leg` 汇总指标
  - 控制台按场景分组打印 `IK failures / rail norm / joint dq`
  - CSV 字段包含：
    - `scenario, leg, ik_failures, rail_mean_abs_norm, rail_max_abs_norm, rail_mean_norm, joint_mean_dq, joint_max_dq`

### ✅ 当前状态
- 脚本语法检查通过（`python3 -m py_compile`）。
- 运行时依赖 `ament_index_python`，需在 ROS2 环境下执行完整离线分析。

### 🧪 推荐用法
```bash
cd ~/aperfect/carbot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run dog2_motion_control obstacle_analysis \
  --batch \
  --scenarios flat,step,window,mixed \
  --duration 12.0 \
  --dt 0.02 \
  --csv-out /tmp/obstacle_metrics.csv
```

---

## 2026-04-01 16:32 MPC 架构层清洗（参数外置 + 构建规范 + Launch 收敛）

完成 `dog2_mpc` 工程架构层标准化，核心是“单一参数源 + 干净构建元数据 + 启动链路无硬编码”：

- **参数文件外置与统一**
  - 新增 `src/dog2_mpc/config/dog2_ctrl_params.yaml`，覆盖 `mpc_node` / `mpc_node_16d` / `mpc_node_complete` 的 `declare_parameter` 参数集合。
  - 参数按逻辑分组整理（注释分段）：`dynamics`、`mpc_core`、`constraints`、`safety_thresholds`、`mpc_weights`、`nominal_pose`。
  - 同步收敛旧 `mpc_params.yaml` 到同一参数口径，避免双配置源漂移。

- **构建系统净化**
  - `src/dog2_mpc/CMakeLists.txt` 删除模板化教学注释块（`ament_lint_auto` 区域冗余说明），保留有效依赖与目标结构。
  - `src/dog2_mpc/package.xml` 移除 `TODO` 描述与许可证占位，补全规范 `<description>` 与 `<license>`。
  - 补齐与 CMake 对齐的依赖声明：`pkg-config`、`eigen`、`osqp`，并保持运行时依赖一致。

- **Launch 结构重构**
  - 重构并清理：
    - `launch/mpc_wbc_simulation.launch.py`
    - `launch/complete_simulation.launch.py`
    - `launch/simple_crossing_sim.launch.py`
    - `launch/test_mpc_wbc_only.launch.py`
  - 全部显式加载 `dog2_ctrl_params.yaml`，删除陈旧注释块、测试残留和节点内硬编码控制参数。
  - 启动文件统一为简洁 PEP8 风格：参数声明最小化、Node 结构扁平、职责清晰。

---

## 2026-04-01 继续：仓库级尾项清洗（杂质文件 + 命名一致性）

在架构层清洗后，继续完成仓库尾项净化，目标是减少“无意义变更噪音”和“历史命名残留”：

- **仓库杂质清理**
  - 删除根目录误生成空文件：`=`
  - `.gitignore` 增补测试工件目录：`.hypothesis/`

- **离线脚本命名一致性收口**
  - 文件：`src/dog2_motion_control/scripts/offline_obstacle_analysis.py`
  - 将旧关节输出键名统一为当前语义命名：
    - `haa_rad/hfe_rad/kfe_rad` → `coxa_rad/femur_rad/tibia_rad`
  - 对应局部变量同步替换，确保离线 CSV 字段与主控制链路术语一致，降低后处理误读风险。

---

## 2026-04-01 继续：全仓统一清洗第二轮（package 元数据收口）

本轮聚焦“包元数据可发布性”与“占位文本归零”，在不触及功能逻辑前提下完成统一收口：

- **占位 maintainer/许可证清理**
  - 统一替换为规范字段：
    - `<maintainer email="dev@example.com">Developer</maintainer>`
    - `<license>Apache-2.0</license>`（保留已有明确 `MIT` 的包不改）
  - 覆盖文件：
    - `src/dog2_mpc/package.xml`
    - `src/dog2_demos/package.xml`
    - `src/dog2_description/package.xml`
    - `src/dog2_dynamics/package.xml`
    - `src/dog2_interfaces/package.xml`
    - `src/dog2_gait_planner/package.xml`
    - `src/dog2_state_estimation/package.xml`
    - `src/dog2_wbc/package.xml`
    - `src/dog2_visualization/package.xml`（仅 maintainer 占位替换）

- **占位 description 收口**
  - 将 `dog2_dynamics / dog2_interfaces / dog2_gait_planner / dog2_state_estimation / dog2_wbc` 的 `TODO: Package description` 替换为简洁功能描述，避免后续打包/审查阶段被占位信息阻断。

- **结果**
  - 全仓 `package.xml` 中 `todo.todo`、`TODO license/description`、`Your Name/your@email.com` 已清零。

---

## 2026-04-02 17:05 构建环境异常定位（colcon setup 链路）

- **现象**
  - `colcon build --packages-select dog2_motion_control` 显示构建成功，但后续 `source install/setup.bash` 报错：
    - `not found: "/home/dell/aperfect/carbot_ws/local_setup.bash"`
  - `ros2 launch dog2_motion_control ...` 提示包未找到，仅搜索到 `/opt/ros/humble`。

- **根因判断（高置信）**
  - 当前 shell 环境中残留了 `COLCON_CURRENT_PREFIX=/home/dell/aperfect/carbot_ws`（工作区根目录），导致 `/opt/ros/humble/setup.bash` 在链式 source 时错误拼接到工作区根目录，去寻找不存在的 `setup.sh/local_setup.sh`。
  - 同时该环境污染会让本工作区 `install/setup.bash` 的 prefix 链解析异常，最终出现“构建成功但 overlay 未生效”。

- **影响范围**
  - 主要影响当前终端会话的 ROS 环境叠加顺序，不是 `dog2_motion_control` 包本体编译失败。

- **建议修复（记录）**
  1. 打开全新终端（或先 `unset COLCON_CURRENT_PREFIX`）。
  2. 在工作区根目录执行：
     - `source /opt/ros/humble/setup.bash`
     - `colcon build --packages-select dog2_motion_control --symlink-install`
     - `source /home/dell/aperfect/carbot_ws/install/setup.bash`
  3. 验证：`echo $COLCON_CURRENT_PREFIX`（应为空或仅在脚本内部短暂使用）；`ros2 pkg list | grep dog2_motion_control` 应可见。

- **备注**
  - 若仍异常，检查是否误 `source` 了工作区根目录下不存在的 `setup.*`，以及 shell 启动脚本（`.zshrc`）是否写入了固定 `COLCON_CURRENT_PREFIX`。

---

## 2026-04-02 17:10 控制栈双架构重建（Position + MPC）

- **目标**
  - 在 `dog2_motion_control` 中形成可并行维护的双控制架构：
    1. **Position 控制链路**（基础动作/调试/快速验证）
    2. **MPC 控制链路**（模型预测控制主线）

- **本轮落地**
  - 新增并接入 MPC 控制节点实现：
    - `dog2_motion_control/mpc_robot_controller.py`
  - 补齐双启动入口：
    - `launch/spider_gazebo_position.launch.py`
    - `launch/spider_gazebo_mpc.launch.py`
  - 配置侧新增控制器参数文件：
    - `config/effort_controllers.yaml`
  - `setup.py` 同步安装项与 console scripts，确保 Position/MPC 入口可被 `ros2 run/launch` 正确发现。

- **架构意义**
  - Position 作为“保底可跑通链路”，用于硬件映射、关节方向和基础步态调试。
  - MPC 作为“性能/越障主链路”，用于后续约束优化、地形适配和稳定性提升。
  - 两条链路共用同一套包与仿真场景，降低切换成本，支持快速 A/B 对比调参。

- **后续建议**
  1. 统一两条 launch 的参数命名与默认值来源（YAML 单一真源）。
  2. 增加 `README` 运行矩阵（Position/MPC 启动命令、适用场景、预期话题）。
  3. 为 MPC 链路补充最小闭环自检项（话题/控制器状态/关节命令频率）。

## 2026-04-02 17:59 接触序列（trot）与 Swing 足端 PD 接入
- **MPC 接触约束动态刷新**
  - 文件：`src/dog2_motion_control/dog2_motion_control/mpc_robot_controller.py`
  - 为 `ConvexMPC.solve` 新增 `contact_pred: (N,4)`，根据 stance/swing 动态更新各腿 `Fz_max`：stance=150N，swing=0N。
- **摆动腿轨迹接管（Swing PD）**
  - 在 `_on_control_timer` 内构建 trot 步态相位并生成 `contact_pred`。
  - 当某腿为 swing 时叠加足端笛卡尔空间 PD，并使用抛物线 clearance 生成期望轨迹（抬升→落足）。
- **Safe Fallback 退化机制一致性**
  - OSQP 失败时使用“上一帧输入序列前移”fallback，并在 fallback 输出上强制执行当前 contact schedule（swing 腿力清零、stance 腿满足摩擦/推力幅值）。

## 2026-04-02 20:57 最近工作补录（MPC 启动链路与仿真参数收口）
- **启动入口与安装链路收口**
  - 文件：`src/dog2_motion_control/setup.py`
  - 完成新控制入口安装项对齐，确保 `mpc_robot_controller` 与双 launch 依赖的脚本入口在 `colcon build --symlink-install` 后可被正确发现。

- **MPC/Position 双启动脚本并行化**
  - 文件：
    - `src/dog2_motion_control/launch/spider_gazebo_mpc.launch.py`
    - `src/dog2_motion_control/launch/spider_gazebo_position.launch.py`
  - 统一 Gazebo 仿真启动骨架，区分 Position 与 MPC 控制链路，便于同场景下快速 A/B 切换验证。

- **控制器参数文件补齐**
  - 文件：`src/dog2_motion_control/config/effort_controllers.yaml`
  - 新增 effort 控制器配置，作为 MPC 链路的控制器参数来源，减少运行期硬编码。

- **URDF 侧联调调整（配合控制链路验证）**
  - 文件：`src/dog2_description/urdf/dog2.urdf.xacro`
  - 按当前仿真联调需求继续小幅收敛模型参数，确保新启动链路下机器人可稳定进入控制循环。

- **典型“反折腿/奇异点”死局定位与规避（仿真启动阶段）**
  - **现象**：机器人一出生贴地或关节初始角接近直腿/错误象限时，MPC/WBC 想要产生向下支撑力，\( \tau = J^T F \) 在奇异/近奇异构型会把腿推向“膝盖朝上”的反折象限，形成力矩正反馈，越用力越折腿。
  - **处置 1（优先）**：提高 spawn 初始高度（如 `-z 0.6~0.8`），让落地前 0.5~1s 有时间把腿收拢到“膝盖朝下”的可控象限，再切入 MPC。
  - **处置 2**：站立/落足目标点（\(p_{des}\)）确保足端在 hip 正下方且 Z 轴深度足够（避免“脚尖目标过浅/过近”诱发反折）。
  - **处置 3（硬防线）**：在 `dog2.urdf.xacro` 中对 `femur/tibia` 的 `limit lower/upper` 做物理可行域裁剪，禁止进入反折角区间，避免错误象限被控制力矩“掰进去”。

- **当前状态**
  - `dog2_motion_control` 已具备 Position/MPC 双入口与对应参数文件；下一步将重点做启动命令矩阵回归（控制器激活状态、关键话题与命令频率一致性）。

## 2026-04-02 21:15 双 AI 并行开发隔离策略（按功能模块）

- **结论**
  - 可以同时使用两个 AI 并行开发 Position 与 MPC 两条链路。
  - 但必须实施“文件级物理边界”，否则会在公共基础设施文件上发生覆盖冲突。

- **危险区（公共文件，人工 Gatekeeper）**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
  - `src/dog2_motion_control/setup.py`
  - `src/dog2_visualization/CMakeLists.txt`
  - `src/dog2_motion_control/package.xml`
  - 规则：以上文件不允许任一 AI 直接自动修改；如需调整，先由 AI 给出 snippet，再由人工审阅并手动合入。

- **安全区（可并行独立修改）**
  - **Position 框架（AI-1 领地）**
    - `src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
    - `src/dog2_motion_control/launch/spider_gazebo_position.launch.py`
    - `src/dog2_motion_control/config/ros2_controllers.yaml`（若存在）
  - **MPC 框架（AI-2 领地）**
    - `src/dog2_motion_control/dog2_motion_control/mpc_robot_controller.py`
    - `src/dog2_motion_control/config/effort_controllers.yaml`
    - `src/dog2_motion_control/launch/spider_gazebo_mpc.launch.py`

- **执行规则（双 AI 协作）**
  1. **提示词隔离**：在每个 AI 会话首条消息明确“只允许修改的文件列表 + 禁改公共文件”。
  2. **公共文件守门**：公共文件仅人工合入，不启用自动改写。
  3. **高频 Git 快照**：每完成一个可运行小里程碑立即提交，保证可快速回滚。

- **推荐提交粒度**
  - `position:` 前缀用于旧链路增量。
  - `mpc:` 前缀用于新链路增量。
  - `infra:` 前缀用于公共文件人工合入（尽量小提交、可追溯）。

- **备注**
  - 当前仓库已具备 Position/MPC 双入口雏形，后续以“分区开发 + 人工守门 + 小步提交”作为默认协作模式。

## 2026-04-02 21:37 Odom 闭环阻塞修复（桥接可用但无源数据场景）

- **问题复盘**
  - 现象：`mpc_controller` 持续告警 `No odometry received yet on '/odom'`，系统进入降级模式。
  - 根因：Gazebo 的 `/world/empty/model/dog2/odometry` 在当前场景中存在话题端点但无稳定消息流，仅靠常规 `parameter_bridge` 不足以提供状态闭环。

- **落地改动**
  - 新增节点：`src/dog2_motion_control/dog2_motion_control/gz_pose_to_odom.py`
    - 从 `/world/empty/dynamic_pose/info`（桥接为 `TFMessage`）提取 `dog2/base_link` 位姿。
    - 通过有限差分估计 twist，发布 `nav_msgs/Odometry` 到 `/odom`。
    - 增加鲁棒保护：`dt` 有效窗口、线速度/角速度限幅、twist 一阶低通、零时间戳回退到当前仿真时钟。
  - 启动链路：`src/dog2_motion_control/launch/spider_gazebo_mpc.launch.py`
    - 增加 `dynamic_pose` 桥接与 `gz_pose_to_odom` 启动。
    - 保持 `odom_gz_topic`/`odom_topic` 参数化，MPC 侧统一订阅 `odom_topic`。
  - 安装与依赖：
    - `src/dog2_motion_control/setup.py` 增加 `gz_pose_to_odom` console script。
    - `src/dog2_motion_control/package.xml` 增加 `nav_msgs`、`tf2_msgs` 依赖。

- **验证结果**
  - `colcon build --packages-select dog2_motion_control --symlink-install` 通过。
  - 启动验证中可观察到：
    - `gz_pose_to_odom ready ...`
    - `Publishing MPC effort commands (odom='/odom', ...)`
    - 不再出现 `No odometry received yet on '/odom'` 持续告警。

---

## 2026-04-03 14:36 Gazebo 出生姿态几何 + MPC 站立阶段抽搐定位

- **现象**
  - MPC 链路下，机器人在 Ignition Gazebo 中出生后机身仍然“卡在世界里”：四足被地面碰撞撑住，关节 PD 在 `mode=joint_pd_standup` 阶段缓慢把机身顶起并伴随明显抽搐。
  - 日志侧确认 `joint_state_broadcaster` / `effort_controller` / `mpc_controller` 全部正常激活，`mpc_robot_controller` 周期性输出 `Publishing MPC effort commands (mode=joint_pd_standup, ...)`。

- **工程改动 1：按 standing_pose 自动计算 spawn_z，并引出安全裕度参数**
  - 文件：`src/dog2_motion_control/launch/spider_gazebo_mpc.launch.py`
    - 利用 Pinocchio 从 `gait_params.yaml` 中的 `standing_pose`（当前基线：`hip_pitch=-0.3, knee_pitch=0.6`）构造 FreeFlyer+关节空间姿态，计算四足 `tibia_link` 在根坐标下的最小 z：`min_foot_z`。
    - 将出生高度设为：`spawn_z = max(0.25, -min_foot_z + spawn_z_margin)`，并在启动日志中打印：
      - `[spider_gazebo_mpc.launch] standing_pose spawn_z=0.462 m (min_foot_z=-0.452 m)`（当前基线）。
    - 新增 launch 参数：
      - `spawn_z_margin`（默认 `0.040`），便于在仿真中快速调高机身初始高度，例如：
        - `ros2 launch dog2_motion_control spider_gazebo_mpc.launch.py use_gui:=true spawn_z_margin:=0.060`

- **工程改动 2：MPC 站立阶段关节空间 PD 抽搐缓解**
  - 文件：`src/dog2_motion_control/dog2_motion_control/mpc_robot_controller.py`
    - 软化并重配站立/idle 阶段关节空间 PD 增益（只在 `mode=joint_pd_standup` / `idle_hold` 下生效）：
      - rail: `kp=800.0, kd=140.0`
      - coxa: `kp=40.0, kd=8.0`
      - femur/tibia: `kp=70.0, kd=14.0`
    - 新增 standup 防抽搐机制：
      - 期望姿态 ramp：在 standup 阶段，将 `standing_pose` 目标从当前关节角以 `standup_ramp_time_sec`（默认 1.0s）线性过渡，避免在碰撞约束下立即施加大刚度拉回。
      - 足端穿地检测：每周期用 Pinocchio 计算足端世界高度 `min_foot_world_z`，若低于阈值 `standup_penetration_foot_z_threshold_m`（默认 0.0），则对 PD 增益施加缩放：
        - `kp ← kp * standup_penetration_kp_scale`（默认 0.35）
        - `kd ← kd * standup_penetration_kd_scale`（默认 1.3）
      - 相关新参数：
        - `standup_ramp_time_sec`
        - `standup_penetration_foot_z_threshold_m`
        - `standup_penetration_kp_scale`
        - `standup_penetration_kd_scale`
    - 在 `_on_control_timer()` 中根据 `startup_elapsed_sec` 计算 `ramp_alpha`，并在非 `mpc_gait` 模式下通过 `_joint_space_hold_tau(...)` 组合：
      - 重力补偿 + ramp 期望姿态 + 穿地自适应增益。

- **当前状态与下一步**
  - 经过上述几何与控制双侧收敛后，机器人出生时机身明显高于地面，且站立阶段关节 PD 不再出现强烈“拼命跺地板”现象，但在特定 spawn_z 组合下仍可观察到轻微抽搐。
  - 下一步计划：
    1. 继续通过 `spawn_z_margin` 做几何侧 A/B，找到“脚先落地、机身不过高”的舒适区间。
    2. 在该区间内固定几何参数，仅通过 YAML/参数收紧 standup PD 增益，验证抽搐是否完全消失。
    3. 待站立稳定后，再打开 `mpc_gait` 模式，检查接触切换（stance/swing）与 Swing 足端 PD 的配合情况。

---

## 2026-04-10 16:50 URDF 可视层去债（base_link 偏移清洗）+ Blender/DAE 工具链打通

- **背景与目标**
  - 在“CAD 原点历史偏移 + URDF 补偿层”并存的情况下，先执行最小风险去债：仅清洗 `base_link` 的 visual/collision 偏移，不触碰 joint 拓扑与惯性参数，避免引入“腿分离/轴心错位”。
  - 目标是把 `base_link` 的可视层从“URDF 外部补偿”迁移为“网格内部吸收偏移”，为后续 `urdf_shift` 深度重构做准备。

- **问题定位（本轮关键排障）**
  - 在 `zsh` 下误 `source /opt/ros/humble/setup.bash` 会触发 `BASH_SOURCE` 兼容问题，出现错误路径拼接（尝试加载工作区根 `setup.sh`），导致 `ros2 pkg prefix` 与 xacro 查包异常。
  - `collada_urdf` 工具链在 Ubuntu 包名为 `collada-urdf-tools`，且可执行文件默认不在 PATH（位于 `/usr/lib/collada_urdf/urdf_to_collada`）。
  - `urdf_to_collada` 依赖 ROS1 风格包解析，若未设置 `ROS_PACKAGE_PATH`，会出现 `package 'dog2_description' not found` 并导致 mesh 资源加载失败（虽可生成 DAE，但几何可能缺失）。

- **已执行操作**
  - Blender 侧：
    - 单独处理 `base_link` 网格，按反向偏移原则将原 URDF visual/collision 偏移量压入 STL。
    - 导出新网格文件：`src/dog2_description/meshes/untitled.stl`（先不覆盖原 `base_link.STL`，用于安全 A/B）。
  - URDF 侧：
    - 文件：`src/dog2_description/urdf/dog2.urdf.xacro`
    - 将 `base_link` 的 `<visual>/<collision>` origin 从 `-0.9780 0.87203 -0.2649` 改为 `0 0 0`。
    - 临时将 mesh 引用从 `base_link.STL` 指向 `untitled.stl`，用于阶段性验证。
  - 转换链路：
    - xacro 展开时通过显式参数绕过 `$(find ...)` 依赖：
      - `controllers_yaml:=/home/dell/aperfect/carbot_ws/src/dog2_description/config/ros2_controllers.yaml`
    - DAE 生成命令：
      - `export ROS_PACKAGE_PATH=/home/dell/aperfect/carbot_ws/src:${ROS_PACKAGE_PATH}`
      - `/usr/lib/collada_urdf/urdf_to_collada /tmp/dog2.urdf /tmp/dog2.dae`
    - 产物大小从约 `82K` 增长至约 `5.5M`，表明 mesh 资源已被正确解析并写入。

- **阶段性结论**
  - “骨架与皮肤分层”策略成立：只清洗 `base_link` visual/collision 层，不修改 joint origin，不会在理论上导致腿部拓扑分离。
  - 当前改动属于“可逆验证态”：通过 `untitled.stl` 先做可视一致性验证，待验证通过后再回收为正式文件命名。

- **下一步（已约定）**
  1. 用 RViz/Gazebo 做 A/B：确认机身外观不瞬移、腿不分离、关节旋转行为不变。
  2. 若通过：将 `untitled.stl` 收口替换 `base_link.STL`，并恢复 xacro mesh 文件名一致性。
  3. 阶段 2 再处理 `urdf_shift` 去债（需全链路重基准，不与本轮混做）。
  4. 阶段 3 进行动力学可信度审计（整机 CoM、惯量正定、轴心一致性）。

---

## 2026-04-10 20:00~21:00 URDF/mesh 偏移策略复盘与批处理尝试（对话记录工作量）

- **核心产出**
  - 明确并讲清 `base_footprint -> base_link -> (visual/collision)` 的“三层”关系，以及“骨架偏移 vs 皮肤偏移”的维护风险。
  - 梳理 `visual origin`/`collision origin`/`inertial` 三者对“显示/碰撞/动力学”的影响边界（视觉不进动力学，但碰撞与惯量会）。
  - 给出 Blender 多对象编辑与批量导出路径；并在实际操作中验证：对全部 STL 统一平移会导致腿/机身视觉分离（不能一刀切）。

- **工程动作（实验性）**
  - 新增并迭代批处理脚本：`src/dog2_description/scripts/batch_rebase_mesh_and_zero_origins.py`（用于批量平移 STL、尝试归零 origin）。
  - 实测发现：直接写回 xacro 的 XML 会破坏 xacro 语义（`controllers_yaml` 解析异常）；已回退并恢复稳定启动。

- **阶段性结论**
  - 无 CAD 源时，mesh 去偏移应按 link/局部策略推进；避免“全局同一平移”这种会打散装配语义的操作。
  - 后续若继续去债：优先在 CAD/Blender 侧消除网格自身偏移，再让 URDF 的 `base_footprint->base_link` 回归物理语义（尽量只保留 Z）。

---

## 2026-04-10 22:35 URDF 轴语义/前向定义对话收敛与回退记录

- **本轮对话聚焦**
  - 统一澄清 `origin rpy`（坐标系摆放）与 `axis xyz`（关节轴定义）的职责边界，以及其对 Jacobian/IK 推导复杂度的直接影响。
  - 讨论并确认工程方向：将关节语义从“负轴+姿态补偿”逐步收敛到更易维护的统一坐标契约。

- **已执行改动（对话内）**
  - mesh 命名去歧义：将 `l1/l11/l111/...` 体系重命名为与 URDF link 对齐的语义名（`*_rail_link/*_coxa_link/*_femur_link/*_tibia_link` 及 collision 对应文件），并同步更新 `dog2.urdf.xacro` 引用。
  - rail 语义尝试：将 `*_rail_joint` 轴改为 `axis="1 0 0"`，并按 `new_low=-old_up, new_up=-old_low` 同步调整四腿滑块限位，保证“向前伸出=正位移”的符号一致性。
  - base 朝向实验：先后尝试调整 `base_offset_joint` 与 `base_link` mesh 旋转用于验证“视觉前向 vs TF 前向”关系。

- **回退与当前状态**
  - 按用户要求，已回退本轮“机身朝向实验”相关改动：
    - `base_offset_joint` 恢复为 `rpy="0 0 ${PI_CONST}"`。
    - `base_link` 的 visual/collision mesh 旋转恢复为 `rpy="0 0 0"`。
  - 当前保留项：rail 轴语义与 mesh 语义命名改造仍在（未要求回退）。

- **结论与后续建议**
  - 启动链路实际读取的是 xacro（`view_dog2.launch.py`），排除了“编辑了 xacro 但运行 urdf”的路径偏差。
  - 下一阶段建议按“先 rail、再旋转关节族、每步回归”推进，避免一次性全量改轴导致控制符号镜像风险。

---

## 2026-04-10 22:45 URDF 语义重构文件审查记录（9 文件）

- **审查范围**
  - `src/dog2_description/CMakeLists.txt`
  - `src/dog2_description/scripts/check_joint_semantics.py`
  - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
  - `src/dog2_description/urdf/dog2.urdf.xacro`
  - `src/dog2_motion_control/dog2_motion_control/joint_semantics.py`
  - `src/dog2_motion_control/dog2_motion_control/kinematics_solver.py`
  - `src/dog2_motion_control/dog2_motion_control/leg_parameters.py`
  - `src/dog2_motion_control/test/test_kinematics.py`
  - `src/dog2_motion_control/test/test_rail_limits.py`

- **审查结论**
  - 本轮改动方向正确，核心目标“rail 语义统一（+q = base_link +X）”在 URDF、参数层、运动学层、测试层之间基本一致。
  - mesh 命名从 `l1/l11/...` 切换到 `lf/lh/rh/rf_*_link` 的引用链完整，未发现明显漏改路径。

- **已执行验证**
  - `check_joint_semantics.py` 执行通过：`[PASS] Joint semantic checks passed.`
  - `check_urdf_shift_boundary.py --strict` 执行通过：`[PASS] URDF shift boundary checks passed ...`

- **残余风险/阻塞（非本轮新增）**
  - 运行 `pytest` 时被既有问题阻塞：`test_joint_controller.py` 导入 `get_revolute_joint_name` 失败，导致无法完成该包全量用例采集。
  - 该问题与本轮 9 文件语义改动无直接因果，但会影响“一键全绿”验证。

- **建议下一步**
  1. 保留当前语义改动，先完成 RViz/Gazebo 运行时拖动验证（四腿 rail +q 行为一致性）。
  2. 单独修复 `test_joint_controller.py` 的导入问题后，再跑 `dog2_motion_control` 全量测试以完成回归闭环。

---

## 2026-04-11 17:20 base 迁移阶段 2 收口 + 碰撞模型回退（胫骨 / 机身）

- **base frame 迁移（Stage 1–2，摘要）**
  - `base_footprint` → `base_offset_joint` → `base_link`（控制 / Pinocchio 根，占位惯量）→ `base_link_cad_fixed`（单位）→ `base_link_cad`（机身质量 / 视觉 / 碰撞）。
  - 四条 `*_rail_joint` 的 **parent = `base_link`**；rail `xyz` 未改（`base_link_cad_fixed` 为单位时世界几何与挂 `base_link_cad` 时一致）。
  - `leg_parameters.py`：rh/rf `base_position.y` 与 URDF 对齐为 `0.1825`；注释统一为 base_link-local。

- **验收与工具**
  - `config/migration_stage2_stand.json`：canonical stand（freeflyer + 各腿 rail=0 + 站立角）下 **`{leg}_coxa_link`（hip）与 `{leg}_foot_link`（足）世界坐标** golden，`tol_m=1e-5`。
  - `scripts/verify_stage2_rail_geometry.py`：`ros2 run` / CMake `dog2_stage2_rail_geometry_check`；`--write-golden` 可重写 golden。
  - `scripts/export_dog2_urdf.py`：展开 xacro 供基线导出。
  - `check_urdf_shift_boundary.py`：校验 `base_link_cad` 视觉/惯量原点与 `base_link_cad_fixed` 身份。

- **测试与已知项**
  - `test/test_kinematics.py`：`TestRailLocking` / Hypothesis 属性测试耗时与边界条件已收紧；**18 passed**。
  - `test_kinematics_consistency.py`：足端对齐段增加 **`[KNOWN_PRE_STAGE2]`**，将 lf/lh 历史 FK vs Pinocchio 偏差与 Stage2 rail 迁移因果区分；进程仍 **exit 0**。

- **碰撞模型（撤销 box 原语，恢复 mesh）**
  - **`tibia_link`**：去掉 AABB `box`、`tibia_col_xyz` 与 L/R 常量；恢复 **`meshes/collision/l${leg_num}111_collision.STL`**。
  - **`base_link_cad` 机身碰撞**：去掉 `base_collision_box_size/xyz`；恢复与 visual 同原点的 **`base_link.STL`** mesh（`origin xyz="-0.9780 0.87203 -0.2649"`）。

- **备注**
  - Stage2 golden 仅依赖运动学；上述碰撞回退 **无需** 重算 golden；`verify_stage2_rail_geometry.py` 与 `check_urdf_shift_boundary.py --strict` 经验证仍 PASS。

## 2026-04-11 16:31 Cursor 对话归档：足端闭环、原语碰撞、MPC 接触桥接、控制接口（本对话）

- **足端 `foot_fixed` / `foot_tip` 与 Python 对齐**
  - `*_foot_fixed` 原点在父系 **`tibia_link`** 下表达；调试足端为 `foot_link` 绿色小球，`foot_sphere_radius = 0.012 m`（直径 `d = 2r`）。
  - 文中「+Y / −Y」均指 **`tibia_link` 的 Y 轴方向**（与 `base_link` 侧向在任意姿态下未必恒同向）。

- **已尝试后撤销的改动（按用户要求回退，当前代码中不存在）**
  - 沿 **`base_link` −Z** 平移 **一球半径**：已在 xacro / `leg_parameters` 中撤销。
  - 沿 **`tibia_link` +X** 平移 **一球直径**（`+0.024 m`）：已撤销，足端 X 恢复为仅 `±foot_tip_lateral_inboard_m`（`±0.024`）。

- **当前保留的足端 Y 向调整（相对历史基准 −0.269179 / −0.269202）**
  1. 先沿 **`tibia_link` +Y** 平移 **半直径**（`d/2 = r = 0.012 m`）。
  2. 再沿 **`tibia_link` −Y** 平移 **一个半直径**（`1.5d = 3r = 0.036 m`）。
  - **净效果**：Y 方向相对原始 CAD 基准再偏移 **`+r − 3r = −2r`**（即 **`−0.024 m`**），数值上腿 1–3 的 Y 约为 **−0.293179**，腿 4 约为 **−0.293202**。

- **涉及文件**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - `foot_sphere_radius`、`foot_tip_plus_y_half_diameter_m`（`= r`）、`foot_tip_minus_y_one_and_half_diameter_m`（`= 3r`）。
    - `leg12` / `leg3` / `leg4` 的 `foot_tip_xyz` 中 Y 分量：`plus − minus − 0.269179`（腿 4 用 `0.269202`）；足端球 `radius` 使用 `${foot_sphere_radius}`。
  - `src/dog2_motion_control/dog2_motion_control/leg_parameters.py`
    - `FOOT_TIP_PLUS_Y_HALF_DIAMETER_M`、`FOOT_TIP_MINUS_Y_ONE_AND_HALF_DIAMETER_M` 与 xacro 对齐；`foot_tip_offset_tibia[1]` 使用 `+= PLUS − MINUS`；`FOOT_TIP_Z_DOWN_M` 仍用 `FOOT_SPHERE_RADIUS_M` 展开，与 URDF `foot_tip_z_down_m` 一致。

- **备注**
  - 若后续「向下 / 侧向」需严格按 **`base_link`** 轴定义，需在具体站姿下做轴变换或改为运行时补偿，不宜仅改标量 `foot_tip_xyz`。

- **`*_foot_link` 物理与 Gazebo 主触地（球心 = frame 原点）**
  - `collision` 与 `visual` 同为球体 `radius=0.012`；`mass=0.05 kg`，实心球惯量 `I=(2/5)mr^2` 对角 `2.88e-06`。
  - Gazebo 摩擦/接触刚度（`mu1/mu2/kp/kd/minDepth/maxVel`）挂在 **`foot_link`**，已从 `tibia_link` 迁走；胫骨另设低摩擦/低刚度以防蹭地抢接触。
  - 各腿 `foot_link` Gazebo 内增加 **`contact` sensor**（`<contact/>` 监测该 link 全部碰撞），供仿真触地反馈。

- **原语碰撞（数值稳定性）**（*注：机身 / 胫骨 box 已于 **2026-04-11 17:20** 回退为 mesh，见上条日志。*）
  - `base_link`：`mesh` 碰撞改为与 base STL **AABB 对齐** 的 `box`。
  - `tibia_link`：胫骨碰撞 STL 改为 **AABB 对齐** 的 `box`，左右腿 `tibia_col_xyz` 分 L/R 常量传入宏。

- **左右镜像 xacro 正规化（减轻单腿 rpy 硬编码）**
  - 抽出 `leg_rail_rpy_L/R`、`leg_hip_rpy_R`、`leg_knee_xyz_R`、`leg_thigh_mesh_flip_rpy` 等属性；四腿实例引用常量，右前大腿 mesh 仍保留必要的 `thigh_rpy` 翻转。

- **ros2_control：向 effort / hybrid 铺路**
  - 各驱动关节在保留 **`position`** 命令接口的同时增加 **`effort`**；`ros2_controllers.yaml` 顶部注明 JTC 仅用 position、力矩模式见 `effort_controllers.yaml`。

- **MPC + Gazebo 足端接触（可选）**
  - `mpc_robot_controller`：`use_gz_foot_contact` 与四条 `gz_contact_topic_*`；订阅 `ros_gz_interfaces/Contacts`，用 **实测** 刷新当前步 `contact_now`（摆腿/混合）；**MPC 视界 `contact_pred` 仍跟步态**。
  - `spider_gazebo_mpc.launch.py`：`gz_world_name`、`bridge_foot_contact`；`parameter_bridge` 映到 `/dog2/foot_contact/{lf,lh,rh,rf}`；`_estimate_min_collision_z` 支持 **mesh/box/sphere** 以配合 spawn 高度估计。
  - `dog2_motion_control/package.xml`：依赖 **`ros_gz_interfaces`**。

- **测试**
  - `test_spherical_foot_geometry_property.py` 更新为 `lf/lh/rh/rf_foot_link`、半径 `0.012`、调试绿色材质。

- **Git 存档（同次提交）**
  - 标签建议：`archive/2026-04-11-1631`；可选 `git archive` 生成 `carbot_ws-archive-20260411-1631.tar.gz` 供离线快照。

## 2026-04-11 20:22 trunk 迁移 Stage 3A / 3B（本对话）

- **Stage 3A：trunk inertial 迁移到 `base_link`**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - `base_link`：用真实 trunk inertial 替换占位惯量，`origin xyz="0.2492 0.12503 0.0" rpy="${half_pi} 0 0"`，`mass=6.0`。
    - `base_link_cad`：删除 inertial，保留 visual + collision。
    - `base_link_cad_fixed`：保持 identity，不改 `xyz/rpy`。
  - 说明注释同步为 Stage 3A 语义：`base_link = trunk inertial carrier`，`base_link_cad = CAD visual/collision carrier`。
  - 该步对应独立提交：`a9eeb58` (`dog2_description: update xacro trunk scaffold`)。

- **Stage 3B：trunk collision 迁移到 `base_link`**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - `base_link`：新增 trunk collision，几何保持与原 `base_link_cad` 完全等价：
      - `origin xyz="-0.9780 0.87203 -0.2649" rpy="0 0 0"`
      - `mesh filename="package://dog2_description/meshes/base_link.STL"`
    - `base_link_cad`：删除 collision，退为 **visual-only**。
    - `base_link_cad_fixed`：仍保持 identity，未改世界几何。
  - 说明注释同步为 Stage 3B 语义：`base_link = trunk inertial + collision`，`base_link_cad = CAD visual carrier`。

- **边界检查脚本升级到 Stage 3B**
  - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
    - 旧假设：`base_link_cad` 挂 inertial / collision。
    - 新假设：
      - `base_offset_joint` 位姿必须保持补偿边界：
        - `xyz = (0.2492, 0.12503, -0.2649)`
        - `rpy = (0, 0, pi)`
      - `base_link` 必须有 trunk inertial
      - `base_link` 必须有 trunk collision
      - `base_link_cad` 只允许 visual，不再有 inertial / collision
      - `base_link_cad_fixed` 必须保持 identity
      - 四条 `*_rail_joint` 的 parent 必须仍为 `base_link`

- **验证结果**
  - `colcon build --packages-select dog2_description --symlink-install`：PASS
  - xacro 展开检查：
    - `base_link has_collision=True has_visual=False has_inertial=True`
    - `base_link_cad has_collision=False has_visual=True has_inertial=False`
  - `python3 src/dog2_description/scripts/check_urdf_shift_boundary.py ...`：PASS
  - `python3 src/dog2_description/scripts/check_joint_semantics.py ...`：PASS
  - `python3 src/dog2_description/scripts/verify_stage2_rail_geometry.py --xacro ... --golden ...`：PASS
    - `all *_rail_joint parents are base_link`
    - `stand pose hip/foot world match golden (tol=1e-05 m)`
  - `python3 -m pytest src/dog2_motion_control/test/test_kinematics.py -q`：**18 passed**

- **当前 trunk 语义状态**
  - `base_link`：控制 / odom / Pinocchio 主 frame，承载 trunk inertial + trunk collision。
  - `base_link_cad`：CAD 壳体 visual-only。
  - `base_link_cad_fixed`：identity scaffold，尚未进入去偏移阶段。

- **后续建议**
  - 下一步优先补一轮 RViz / Gazebo 运行时目检：
    - RViz `Collision Enabled` 下确认 trunk collision 不跳位
    - Gazebo 确认 spawn 高度、蹭地与主接触行为无异常
  - 等 Stage 3B 运行时验证通过后，再单独考虑：
    - `base_link_cad = visual-only` 的文档收口
    - trunk collision mesh → primitive box / 多 box
    - 最后才触及 `urdf_shift_*` / `base_offset_joint` / `base_link_cad_fixed`

## 2026-04-12 20:03 语义根收口 + 限位单一真值 + 注释终态化（本对话）

- **URDF 根语义收口（删除假 footprint）**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - 删除 `base_footprint` 与 `base_offset_joint`，`base_link` 成为唯一 URDF 根。
    - 明确口径：若未来需要真实 `base_footprint`（ground projection / planning），由运行时 TF 发布，不在 URDF 内伪造。
  - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
    - 新增唯一根检查：root 必须且仅能是 `base_link`。
    - 将 `base_footprint` / `base_offset_joint` 设为禁止回流项（forbidden link/joint）。

- **移除 `base_link_cad` 包装层（方案 B 落地）**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - 删除 `base_link_cad` 与 `base_link_cad_fixed`。
    - trunk visual 直接并入 `base_link`；mesh 文件保持不动，仅把两层历史偏移合并到 `base_link` 的 `visual origin`。
  - `src/dog2_description/scripts/check_urdf_shift_boundary.py`
    - 边界检查改为：`base_link` 同时承载 trunk inertial + collision + visual。
    - 禁止 `base_link_cad` / `base_link_cad_fixed` 回流。

- **关节限位收口为单一真值源（URDF）**
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - 旋转关节限位拆分为 `coxa` / `femur` / `tibia` 三组独立属性，不再用统一 `hip/knee` 复用。
    - 当前统一大限位：`coxa [-2.618, 2.618]`，`femur [-2.8, 2.8]`，`tibia [-2.8, 2.8]`。
  - `src/dog2_motion_control/dog2_motion_control/urdf_joint_limits.py`（新增）
    - 运行时从 `dog2.urdf.xacro` 展开并解析关节限位。
  - `src/dog2_motion_control/dog2_motion_control/leg_parameters.py`
    - `joint_limits` 改为从 URDF 解析结果注入。
  - `src/dog2_motion_control/dog2_motion_control/spider_robot_controller.py`
    - 去除用 YAML 覆盖旋转关节限位的路径，避免出现“URDF 一套、控制器一套”。
  - `src/dog2_motion_control/config/gait_params*.yaml`
    - 移除 `haa/hfe/kfe` 旋转限位字段，仅保留 rail 相关兼容项。
  - `src/dog2_motion_control/test/test_urdf_joint_limits_sync.py`（新增）
    - 新增同步回归：保证控制侧读取限位与 URDF 一致。

- **文档与注释终态化**
  - `src/dog2_description/README_JOINT_LIMITS.md`
  - `src/dog2_description/doc/base_frame_migration.txt`
  - `src/dog2_description/urdf/dog2.urdf.xacro`
    - 去除 `Dual-base scaffold` / `Stage 4` 等过渡期表述，改为当前职责说明。
    - `foot_tip` 注释改为当前几何定义口径，不再使用“回退前工作区基准”语气。
    - `base_link` inertial `rpy="${half_pi} 0 0"` 的语义注明为冻结映射项（有新证据前不改）。

- **验证（本轮执行）**
  - `python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict`：PASS
  - `python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro`：PASS
  - `python3 -m py_compile ...`：PASS
  - `python3 -m pytest src/dog2_motion_control/test/test_urdf_joint_limits_sync.py src/dog2_motion_control/test/test_joint_controller.py src/dog2_motion_control/test/test_kinematics.py -q`：`28 passed`

## 2026-04-12 21:52 关节轴显式化收尾（四腿模板统一）

- **范围与目标**
  - 目标：把四条腿都收敛到同一显式轴语义模板，消除“`axis=-X` + 多层 `rpy` 隐式拧轴”的可维护性风险。
  - 约束：本轮只做关节轴显式化，不触碰 trunk、`leg_hip_rpy_R` / `leg_knee_xyz_R`、`lh/rf hip_xyz`、`rf` mesh flip 的历史归因。

- **URDF 主改动（`src/dog2_description/urdf/dog2.urdf.xacro`）**
  - 为 `lf/lh/rh/rf` 统一引入三段固定对齐层：
    - `*_coxa_axis_frame -> *_coxa_joint -> *_coxa_drive_frame -> *_coxa_pose_fixed`
    - `*_femur_axis_frame -> *_femur_joint -> *_femur_drive_frame -> *_femur_pose_fixed`
    - `*_tibia_axis_frame -> *_tibia_joint -> *_tibia_drive_frame -> *_tibia_pose_fixed`
  - 关节轴显式语义统一为：
    - `coxa_joint axis="0 0 -1"`
    - `femur_joint axis="0 -1 0"`
    - `tibia_joint axis="0 -1 0"`
  - `rf` 在本轮完成 `coxa -> femur -> tibia` 三层收尾后，四条腿模板一致。

- **检查与回归护栏**
  - `src/dog2_description/scripts/check_joint_semantics.py`
    - 语义检查改为沿固定链追 joint frame，不再依赖“单层 origin rpy”假设，可正确覆盖 axis-frame/pose-fixed 结构。
  - `src/dog2_motion_control/test/test_lf_zero_pose_se3.py`（新增）
    - 新增并扩展四腿 `q=0` SE(3) 回归：
      - `*_coxa_link`, `*_femur_link`, `*_tibia_link`, `*_foot_link`
      - 同时比较平移与旋转，避免“仅 foot 不漂”的假通过。

- **本轮验证**
  - `python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro`：PASS
  - `python3 src/dog2_description/scripts/check_urdf_shift_boundary.py --strict`：PASS
  - `python3 -m pytest src/dog2_motion_control/test/test_lf_zero_pose_se3.py -q`：`4 passed`
  - `python3 -m pytest src/dog2_motion_control/test/test_kinematics.py -q`：`18 passed`

- **阶段结论**
  - 四条腿关节轴语义模板化已完成，当前进入“历史量归因”阶段：
    - `leg_hip_rpy_R`
    - `leg_knee_xyz_R`
    - `lh/rf hip_xyz`
    - `rf` mesh flip（继续保持在几何层，不回流到关节语义层）
