# dog2_bringup

`dog2_bringup` is the unified integration layer for the Dog2 workspace.

It adds:
- a single top-level bringup package
- a flat-ground Gazebo Fortress world
- an obstacle-step Gazebo Fortress world
- a window-frame Gazebo Fortress world
- unified system launch entrypoints
- keyboard `/cmd_vel` teleop
- interface adapters for legacy MPC/WBC topics
- an automated smoke-check node
- a delayed crossing trigger for research-stack trials
- an automated window-crossing check
- integration-oriented docs

Main launches:
- `ros2 launch dog2_bringup system.launch.py`
- `ros2 launch dog2_bringup sim_base.launch.py`
- `ros2 launch dog2_bringup control_stack.launch.py`
- `ros2 launch dog2_bringup visualization.launch.py`
- `ros2 launch dog2_bringup smoke_test.launch.py`
- `ros2 launch dog2_bringup crossing_trial.launch.py`
- `ros2 launch dog2_bringup window_crossing_test.launch.py`

Research effort path:
- `ros2 launch dog2_bringup system.launch.py controller_mode:=effort research_stack:=true`

Recommended cold-start validation:
- `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true`

Complex-scene research entry:
- `ros2 launch dog2_bringup crossing_trial.launch.py`

Window-frame PASS/FAIL validation:
- `ros2 launch dog2_bringup window_crossing_test.launch.py ros_domain_id:=47 result_file:=/tmp/dog2_window_crossing_result.txt`

Current crossing development commands:
- build changed packages: `colcon build --packages-select dog2_mpc dog2_bringup --symlink-install --event-handlers console_direct+`
- flat smoke regression: `ros2 launch dog2_bringup smoke_test.launch.py controller_mode:=effort research_stack:=true expect_research_stack:=true ros_domain_id:=93 result_file:=/tmp/dog2_smoke_result.txt`
- window crossing validation: `ros2 launch dog2_bringup window_crossing_test.launch.py ros_domain_id:=91 timeout_sec:=150.0 result_file:=/tmp/dog2_window_crossing_result.txt`
- key logs: `grep -E "PASS|FAIL|MPC:|状态转换|crossing" /tmp/dog2_window_*.log /tmp/dog2_smoke_*.log`

Current window-crossing status:
- PASS currently means the trigger fired, the research stack stayed fresh, rails moved significantly, and odom reached the configured frame region.
- The current development boundary is `CROSSING:BODY_FORWARD_SHIFT`; rear-leg transit and recovery are still under stabilization.
- Prefer a fresh `ros_domain_id` for each run. The test launches pre-clean common stale ROS/Gazebo processes, but stale domains can still make debugging ambiguous.
