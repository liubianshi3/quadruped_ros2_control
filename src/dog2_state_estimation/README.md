# dog2_state_estimation

`dog2_state_estimation` is the stage-1 estimator facade for the Dog2 stack.

Current behavior:
- consumes `/odom`, `/joint_states`, and optional `/imu/data`
- republishes normalized system-facing topics
- publishes `dog2_interfaces/RobotState` and `dog2_interfaces/ContactState`
- keeps the estimator contract stable while the internal implementation is still simulation-driven

Published topics:
- `/dog2/state_estimation/robot_state`
- `/dog2/state_estimation/contact_state`
- `/dog2/state_estimation/joint_states`
- `/dog2/state_estimation/odom`

Design notes:
- `contact_strategy=all_true` is the default stage-1 behavior
- the node is intentionally replaceable; future contact-aided estimation should keep the same outputs

Launch:

```bash
ros2 launch dog2_state_estimation state_estimation.launch.py
```
