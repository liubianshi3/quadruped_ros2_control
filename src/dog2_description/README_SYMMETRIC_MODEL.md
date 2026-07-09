# Dog2 Symmetric 仿真模型

完全对称的 dog2 仿真变体，用于 IK / WBC / MPC 推导和调试。

## 文件对照

| 模型 | URDF xacro | 控制参数 | 机器人名 |
|---|---|---|---|
| real (默认) | `src/dog2_description/urdf/dog2.urdf.xacro` | `dog2_motion_control.leg_parameters.LEG_PARAMETERS` | `dog2` |
| symmetric | `src/dog2_description/urdf/dog2_symmetric.urdf.xacro` | `dog2_motion_control.leg_parameters_symmetric.LEG_PARAMETERS_SYMMETRIC` | `dog2_symmetric` |

## symmetric 模型差异

- **Leg mount**：严格关于 base_link 中心对称（前/后 X 幅度相等，左/右 Y 幅度相等）
- **Base_link 惯性**：origin = (0,0,0)，纯对角惯性张量
- **rf hip_xyz**：x=0.016（镜像自 lh），不再使用 real 模型的非对称实测值
- **leg_knee_xyz_R**：z=-0.0274（与左膝幅度一致）
- **leg4 shin / foot_tip**：完全等于 leg3，无独立漂移
- **Shin COM**：左 ±0.026  /  右 -0.026（精确镜像）

## 启动 / 算法接入原则

### launch 中选择 symmetric 模型

所有核心 launch 已支持 `model_variant` 参数，默认值为 `"real"`：

```bash
# symmetric 仿真
ros2 launch dog2_mpc complete_simulation.launch.py model_variant:=symmetric
ros2 launch dog2_mpc mpc_wbc_simulation.launch.py model_variant:=symmetric
ros2 launch dog2_mpc simple_crossing_sim.launch.py model_variant:=symmetric
ros2 launch dog2_motion_control spider_gazebo_mpc.launch.py model_variant:=symmetric
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py model_variant:=symmetric
ros2 launch dog2_motion_control spider_fortress_simple.launch.py model_variant:=symmetric
ros2 launch dog2_motion_control spider_gazebo_position.launch.py model_variant:=symmetric
ros2 launch dog2_bringup control_stack.launch.py model_variant:=symmetric
ros2 launch dog2_bringup effort_research_sim.launch.py model_variant:=symmetric

# RViz2 直接查看 symmetric mesh
ros2 launch dog2_description view_dog2_xacro.launch.py model_variant:=symmetric

# Gazebo Fortress GUI 查看 symmetric mesh
ros2 launch dog2_description dog2_fortress_with_gui.launch.py model_variant:=symmetric

# Gazebo Fortress headless smoke test
ros2 launch dog2_description gazebo_headless.launch.py model_variant:=symmetric
```

### Python 控制模块中选择 symmetric 参数

```python
from dog2_motion_control.model_variant import get_leg_parameters

params = get_leg_parameters("symmetric")   # LEG_PARAMETERS_SYMMETRIC
params = get_leg_parameters("real")        # LEG_PARAMETERS (默认)
```

```python
from dog2_motion_control.kinematics_solver import create_kinematics_solver

solver = create_kinematics_solver(model_variant="symmetric")
```

### 重要约束

- **不允许在同一次仿真里 URDF 用 symmetric、控制参数却用 real**（或者反之），否则 mount、惯性、足端不一致
- Blender visual mesh 和 collision mesh 已生成到 `src/dog2_description/meshes_symmetric/`
- 当前 symmetric 模型保证**关节 / 足端 / 惯性 / 控制参数 / visual mesh / collision mesh**使用独立对称变体
- real 模型的 `dog2.urdf.xacro`、`dog2_properties.xacro`、`leg_parameters.py` **默认行为永远不变**

## 验证

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

python3 src/dog2_description/scripts/check_symmetric_urdf.py
python3 src/dog2_description/scripts/check_real_model_guard.py
```

## Golden

`symmetric stand golden` 由 Pinocchio 从 `dog2_symmetric.urdf.xacro` 生成，存放在：
`src/dog2_description/config/migration_symmetric_stand.json`
