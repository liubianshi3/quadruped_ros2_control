# Naming Conventions

Topic prefix:
- use `dog2_*` for system-specific topics, services, and debug channels

Frames:
- `world`
- `odom`
- `base_link`

Controllers:
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `rail_position_controller`
- `effort_controller`

Parameter files:
- `*_rates.yaml`
- `*_topics.yaml`
- `*_mpc.yaml`
- `*_estimator.yaml`

Launch files:
- `system.launch.py`
- `sim_base.launch.py`
- `control_stack.launch.py`
- `visualization.launch.py`
- `smoke_test.launch.py`
