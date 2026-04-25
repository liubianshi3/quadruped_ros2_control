# Dog2 Interface Contract

Primary system-facing topics:

- `/cmd_vel`
- `/dog2/state_estimation/robot_state`
- `/dog2/state_estimation/contact_state`
- `/dog2/state_estimation/odom`
- `/dog2/gait/contact_phase`
- `/dog2/gait/command`
- `/dog2/mpc/foot_forces`
- `/dog2/mpc/grf_reference`
- `/dog2/mpc/debug`
- `/dog2/wbc/joint_effort_command`
- `/dog2/wbc/debug`

Key services:

- `dog2_interfaces/srv/SetGait`
- `dog2_interfaces/srv/SetControllerMode`
- `dog2_interfaces/srv/SetRailMode`
- `dog2_interfaces/srv/TriggerAction`
