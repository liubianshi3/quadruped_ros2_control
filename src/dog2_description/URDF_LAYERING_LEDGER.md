# URDF 历史量分层归因台账（Ledger）

**包路径**：与本文件同包 `dog2_description` 内的 `urdf/dog2.urdf.xacro` 为运动学真值源之一。

**文档用途**：记录剩余「历史量」在 **A 安装层 / B 语义链层 / C 几何层** 中的归属、证据与重审条件。  
**不是**追求「零特例」，而是追求**每项可审计**。

---

## 迁层停手条件（全局）

1. **允许迁层**：仅当能证明在 **`q=0`** 下 **`{leg}_coxa_link`、`{leg}_femur_link`、`{leg}_tibia_link`、`{leg}_foot_link`** 的完整 **SE(3)** 与当前基线一致（含旋转），而非仅足端或仅平移闭合。
2. **必须停手**：若迁层或分解后上述任一连杆 SE(3) 漂移 → **立即回退**，并将该项**定稿为 B（或注明保留在当前层）**，禁止「再微调一组数试试」式迭代。
3. **数值与结构**：本台账建立后，任何迁层改动仍须**单独 PR/提交**，并重新跑护栏与测试。

---

## 护栏与测试引用（全局）

| 角色 | 路径（仓库内） |
|------|----------------|
| 关节轴与语义（base_link 约定下） | `dog2_description/scripts/check_joint_semantics.py` |
| `q=0` 位姿 SE(3)  golden（示例腿） | `dog2_motion_control/test/test_lf_zero_pose_se3.py` |
| 运动学往返/一致性 | `dog2_motion_control/test/test_kinematics.py` |
| 运动学一致性（脚本入口，若流程使用） | `dog2_motion_control/scripts/test_kinematics_consistency.py` |
| 典型站立位形 golden（迁移/阶段 2） | `dog2_description/config/migration_stage2_stand.json` |

运行方式以各文件头部说明与项目惯例为准；迁层前后至少应覆盖 **语义检查** + **与 SE(3) 相关的测试**。

---

## 归因表（初稿）

列说明：**当前层** = 该量**当前主要影响**的层级；**候选目标层** = 仅当满足停手条件上方可迁入的方向。

| 名称 | 当前层 | 候选目标层 | 保留/迁层证据 | `q=0` SE(3) 风险 | 当前结论 | 允许何时重审 | 护栏/测试引用 |
|------|--------|------------|---------------|------------------|----------|--------------|----------------|
| `leg_thigh_mesh_flip_rpy`（`rf` 大腿 **visual/collision** `<origin rpy>`，macro 内） | **C** | **C**（不重升到关节语义） | 仅作用于网格/碰撞显示帧；关节语义由 `femur_axis_fixed` / `femur_joint` / `femur_pose_fixed` 承担；与 Phase 1「macro 内部几何补丁」一致 | 若错误升到 actuated joint 语义层，会污染控制/IK 与轴约定；**保持现状**下无额外风险 | **已结案 / 冻结**（C 层几何补丁） | 重导 `l411` STL、更换大腿碰撞几何、或资产级 mesh 坐标清洗时 | `check_joint_semantics.py`；几何大改后建议加跑 `test_lf_zero_pose_se3.py` 及全腿 SE(3) 相关测试 |
| `leg_hip_rpy_R`（右腿髋静态 rpy — 遗留命名常量） | **已迁出 `coxa_joint` revolute 静态项**（**`rh` / `rf` 均同构**：`*_coxa_axis_fixed` + `*_coxa_pose_fixed`；`*_coxa_joint` 原点恒等、`axis` `0 0 -1`） | **A**（mount）若未来要再吸收：须完整 SE(3) 证明（Phase 2 已证平移闭合不够） | **证据**：原单关节 `rpy="π 0 π/2"`（及 `rf` 的 `hip_xyz`）与 **`hip_xyz` + 两段 fixed 的 RPY** 在 `rail→coxa_link` 上 **SE(3) 等价**（已用 Pinocchio / `test_lf_zero_pose_se3` / `check_joint_semantics` 验证）。宏实例**不再**传 `hip_rpy`；常量仅保留供 diff、台账与 `grep`。**≠**「从仓库删除该符号」 | mount 误吸收仍高 | **`rh`：已清算。`rf`：已清算。** 历史量**从 revolute B 迁到 fixed** 已**全局结案**（就「是否仍压在 `coxa_joint` 上」而言）；**勿**说成「从未存在」或「与 hip 几何无关」 | 改右腿 coxa 模板、或 CAD 推翻等价分解时 | `check_joint_semantics.py`；`test_lf_zero_pose_se3.py`；`test_kinematics.py`；`migration_stage2_stand.json` |
| `leg_knee_xyz_R`（`rh`/`rf` 的 `knee_xyz` → `femur_axis_fixed` 平移） | **B**（右腿模板语义参数；**几何承载在 fixed** `*_femur_axis_fixed`） | **B**；不建议再并入 `coxa_pose_fixed` / mount（混淆髋/膝语义，等价拆分无收益） | **`rh` / `rf`**：膝铰链偏移均在对应 `*_femur_axis_fixed` 的 `xyz`，**不在** `*_femur_joint`。`leg_hip_rpy_R` 清算完成后，下一项历史线即本量：**优先定性 + 证据化**，**倾向接受最终留在语义 B**；勿一开就想往 mount 搬 | **高**（若强行重分配 fixed 而未保持 SE(3)） | **待再审（下一批）**；与 `leg_hip_rpy_R` 同纪律：`q=0` **coxa/femur/tibia/foot** 全链 SE(3) 为唯一闸门 | CAD 膝铰链或 `femur_axis_fixed` 模板重定义时 | 同上全链路 SE(3) 相关测试；`migration_stage2_stand.json` |
| `lh` `hip_xyz` 宏实例覆盖（`0.016 0.0199 0.055`） | **B**（实例层相对默认的链上参数） | **A** 或 **B**（逐证据定） | 与宏默认 `hip_xyz` 呈 **X 符号对翻**类关系，可能为镜像约定；**不得**与 `rf` 合并叙述 | 中–高：改动会移动 coxa 关节原点 | **待逐腿审计** | 有机械安装图/CAD 证明可迁入 mount 时；或左腿模板统一重构时 | `check_joint_semantics.py`；`test_kinematics.py`；`migration_stage2_stand.json` |
| `rf` `hip_xyz` 宏实例覆盖（`0.0116 0.0199 0.055`） | **B**（实例层独立偏移） | **A** 或 **B**（逐证据定） | **非**简单 `±0.016` 镜像，属**独立历史偏移**；**不得**默认等同 `rh` 或并入「统一右腿故事」 | 中–高 | **待逐腿审计**（**最后批次**，与 `lh` 不并案） | 同左，且需与 `rh`/`rf` 机械差异书面一致时 | 同上 |

---

## 与 `dog2.urdf.xacro` 的对应关系（便于查代码）

- **`leg_thigh_mesh_flip_rpy`**：`rf` 分支内 `thigh_vis_rpy` / `thigh_col_rpy_eff`（visual/collision `<origin rpy>`）。
- **`leg_hip_rpy_R`**：xacro **属性保留**，宏实例**不传** `hip_rpy`。**`rh` / `rf`** 均在 `*_coxa_axis_fixed`、`*_coxa_pose_fixed` 承担髋静态姿态（与各自 `hip_xyz` 配合）。源码 `grep` 应**无** `hip_rpy="${leg_hip_rpy_R}"`。
- **`leg_knee_xyz_R`**：`rh`、`rf` 实例传入 `knee_xyz="${leg_knee_xyz_R}"`（进入 `femur_axis_fixed` 的 `xyz`）。
- **`lh` / `rf` `hip_xyz`**：`lh`、`rf` 实例各自 `hip_xyz="..."` 覆盖宏默认。

---

## 口径：`leg_hip_rpy_R`（避免过度表述）

**状态摘要（务必先读）**

- **`leg_hip_rpy_R`：对 `rh` 已清算；对 `rf` 已清算**（就「髋静态 rpy 是否仍压在 `coxa_joint` 上」而言）。二者等价姿态均在 **`{rh,rf}_coxa_axis_fixed` + `{rh,rf}_coxa_pose_fixed`**；**不要**再写成「只有 `rh` 清完、`rf` 还压在 revolute 上」。
- **仍须避免的过度表述**：**不要**把「清算完成」说成「这个符号从模型里消失了」——命名常量 **`leg_hip_rpy_R` 仍保留**（旧单关节 rpy 的数值标签，便于审计）；**不要**说成「右腿髋与历史 rpy 无关」。
- **与「全局」的关系**：就 **「revolute B 层是否仍承载该历史 rpy」** 这一问题，已 **全局结案（否）**；若将来讨论 **mount 再吸收**，那是**另一项**迁层命题，仍受 Phase 2 停手条件约束。

**最短排查路径（每次迁层/质疑时）**

1. **展开 `rh` / `rf` 链**，核对 `*_coxa_joint`：`xyz` 与 `rpy` 均为零、`axis` 为 `0 0 -1`；髋静态 rpy 不得再有效落在 revolute 的 `<origin rpy>` 上，只应体现在 **coxa 前后 fixed** 的等价表达里。
2. **`grep` / 源码**：`hip_rpy="${leg_hip_rpy_R}"` 应 **0 处**；`rh` / `rf` 实例均不传 `hip_rpy`。
3. **护栏**：`check_joint_semantics.py` + **`q=0` 下 `coxa` / `femur` / `tibia` / `foot` 完整 SE(3)**（如 `test_lf_zero_pose_se3.py`）。仅足端不漂、或仅平移不漂、或语义脚本过但 SE(3) 回归失败，均视为**未保住整条链等价**，须回退。

**`leg_knee_xyz_R`（若以后再动）**：先冻结 `rh` 基线；**仅 `rh` 试改**；任一连杆 `q=0` SE(3) 漂则立即回退并定稿为 B；优先不要往 mount 硬搬。

---

## 修订记录

| 日期 | 说明 |
|------|------|
| 2026-04-12 | 初稿：建立台账、C 层 `rf` mesh flip 结案、其余项标为待清算/待审计。 |
| 2026-04-12 | **`rh` 清算（局部）**：`leg_hip_rpy_R` 对 `rh` 已**不再是 `coxa_joint` 本体的 B 层依赖**，等价在 `coxa` 前后 fixed；当时 **`rf` 仍待清算**（台账口径收紧）。 |
| 2026-04-12 | **`rf` 清算**：`rf` 与 `rh` **共享**同一套 `coxa_axis_fixed` / `coxa_pose_fixed` RPY 分解（仅 `hip_xyz` 按腿区分）；`leg_hip_rpy_R` **宏不再传入**；**`leg_hip_rpy_R`：对 `rh`、对 `rf` 均已清算**（revolute B 层已腾空）。**下一步**：`leg_knee_xyz_R` 定性/证据化（倾向接受语义 B），**勿先动** `lh`/`rf` `hip_xyz`、`rf` mesh flip、trunk、Stage2 golden。 |
