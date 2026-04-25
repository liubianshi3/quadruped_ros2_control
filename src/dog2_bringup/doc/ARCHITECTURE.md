# Dog2 Architecture

Stage-1 integrated architecture:

1. Simulation base:
   - Gazebo Fortress
   - `robot_state_publisher`
   - `ros2_control` controllers
   - stable position-mode or effort-mode launch from `dog2_motion_control`

2. Estimation facade:
   - `dog2_state_estimation`
   - wraps ground-truth odom + joint states into `dog2_interfaces/RobotState`

3. Gait / control / debug stack:
   - `dog2_gait_planner` publishes contact phase and gait command topics
   - existing `dog2_motion_control` controller remains the executable path for phase-1 stability
   - existing C++ `dog2_mpc` / `dog2_wbc` can be started as research/debug producers
   - adapter nodes normalize their debug outputs

4. Visualization:
   - RViz2
   - `dog2_visualization`
   - unified debug topics under `/dog2/*`
