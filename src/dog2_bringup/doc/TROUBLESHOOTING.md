# Troubleshooting

Gazebo robot does not spawn:
- check `/robot_description`
- verify `GZ_SIM_RESOURCE_PATH`
- confirm the selected world file exists

Controllers do not activate:
- run `ros2 control list_controllers`
- make sure the correct launch path was chosen for `controller_mode`

`use_sim_time` mismatch:
- check `ros2 topic echo /clock`
- inspect node parameters for `use_sim_time`

Estimator has no output:
- verify `/odom` and `/joint_states`
- ensure `dog2_state_estimator` is running

MPC/WBC debug topics are missing:
- verify `research_stack:=true`
- inspect `/dog2/mpc/foot_forces` and `/dog2/wbc/joint_effort_command`

Smoke validation looks contaminated by old runs:
- prefer `ros2 launch dog2_bringup smoke_test.launch.py ros_domain_id:=44 ...`
- keep each validation run on its own `ROS_DOMAIN_ID`
- avoid reusing a domain with leftover Gazebo or ROS graph processes

Research stack starts but no effort reaches Gazebo:
- inspect `/effort_controller/commands`
- confirm `/dog2/gait/contact_phase` is fresh during smoke bringup
- verify `effort_controller` is active before motion stages begin

Window crossing fails before the body reaches the frame:
- inspect `/dog2/mpc/crossing_state`
- inspect `/tmp/dog2_window_crossing_result.txt`
- if the result shows large rail motion but `max_x` stays far below the frame, the controller likely armed crossing too early or the MPC QP is still going infeasible

Window crossing launch hangs after a PASS/FAIL result:
- prefer `ros2 launch dog2_bringup window_crossing_test.launch.py timeout_sec:=60.0 ...`
- check that `crossing_check` exited and wrote the result file
- if needed, stop leftover scene processes with `pkill -f dog2_window_frame`

`gz_pose_to_odom` keeps printing `no transform matched model/base hints`:
- inspect `/dog2/dynamic_pose_tf`
- verify `model_name:=dog2` and the selected `gz_world_name`
- avoid reusing a ROS domain that still has leftover Gazebo bridges from an old crossing run
