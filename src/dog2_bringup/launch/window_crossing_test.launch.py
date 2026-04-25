#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cleanup_pattern = (
        "[d]og2_window_frame|window_frame[.]sdf|/world/[d]og2_window_frame|"
        "[c]rossing_check|[c]rossing_trigger|[r]obot_state_publisher|"
        "[g]z_pose_to_odom|[s]im_state_estimator_node[.]py|[g]ait_scheduler_node[.]py|"
        "[m]pc_node_complete|[w]bc_node_complete|[w]bc_effort_mux|"
        "[m]pc_debug_adapter|[w]bc_debug_adapter|[v]isualization_node|"
        "[p]arameter_bridge|[i]gn gazebo|[g]z sim|"
        "[s]pawner_joint_state_broadcaster|[s]pawner_effort_controller"
    )
    preclean_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' "
                "2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' "
                "2>/dev/null || true"
            ),
        ],
        output="screen",
    )
    crossing_check = Node(
        package="dog2_bringup",
        executable="crossing_check",
        name="dog2_crossing_check",
        output="screen",
        parameters=[
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("timeout_sec"),
                    value_type=float,
                ),
                "window_x_position": ParameterValue(
                    LaunchConfiguration("crossing_window_x_position"),
                    value_type=float,
                ),
                "body_pass_margin": ParameterValue(
                    LaunchConfiguration("body_pass_margin"),
                    value_type=float,
                ),
                "rail_motion_threshold_m": ParameterValue(
                    LaunchConfiguration("rail_motion_threshold_m"),
                    value_type=float,
                ),
                "result_file": LaunchConfiguration("result_file"),
            }
        ],
    )
    cleanup_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' "
                "2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' "
                "2>/dev/null || true"
            ),
        ],
        output="screen",
    )
    crossing_trial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dog2_bringup"), "launch", "crossing_trial.launch.py"]
            )
        ),
        launch_arguments={
            "ros_domain_id": LaunchConfiguration("ros_domain_id"),
            "use_gui": LaunchConfiguration("use_gui"),
            "rviz": LaunchConfiguration("rviz"),
            "teleop": LaunchConfiguration("teleop"),
            "spawn_delay_sec": LaunchConfiguration("spawn_delay_sec"),
            "crossing_delay_sec": LaunchConfiguration("crossing_delay_sec"),
            "crossing_window_x_position": LaunchConfiguration("crossing_window_x_position"),
            "crossing_window_width": LaunchConfiguration("crossing_window_width"),
            "crossing_window_height": LaunchConfiguration("crossing_window_height"),
            "crossing_window_bottom_height": LaunchConfiguration("crossing_window_bottom_height"),
            "crossing_window_top_height": LaunchConfiguration("crossing_window_top_height"),
            "crossing_window_safety_margin": LaunchConfiguration("crossing_window_safety_margin"),
            "crossing_activation_distance": LaunchConfiguration("crossing_activation_distance"),
            "crossing_approach_speed": LaunchConfiguration("crossing_approach_speed"),
            "crossing_force_full_support": LaunchConfiguration("crossing_force_full_support"),
            "crossing_freeze_rail_targets": LaunchConfiguration("crossing_freeze_rail_targets"),
            "crossing_trigger_x_threshold": LaunchConfiguration("crossing_trigger_x_threshold"),
            "pre_crossing_cmd_vel_x": LaunchConfiguration("pre_crossing_cmd_vel_x"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ros_domain_id", default_value="47"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("teleop", default_value="false"),
            DeclareLaunchArgument("crossing_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("spawn_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("timeout_sec", default_value="150.0"),
            DeclareLaunchArgument("body_pass_margin", default_value="0.10"),
            DeclareLaunchArgument("rail_motion_threshold_m", default_value="0.025"),
            DeclareLaunchArgument(
                "result_file",
                default_value="/tmp/dog2_window_crossing_result.txt",
            ),
            DeclareLaunchArgument("crossing_window_x_position", default_value="1.55"),
            DeclareLaunchArgument("crossing_window_width", default_value="0.48"),
            DeclareLaunchArgument("crossing_window_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_bottom_height", default_value="0.0"),
            DeclareLaunchArgument("crossing_window_top_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_safety_margin", default_value="0.04"),
            DeclareLaunchArgument("crossing_activation_distance", default_value="0.75"),
            DeclareLaunchArgument("crossing_approach_speed", default_value="0.15"),
            DeclareLaunchArgument("crossing_force_full_support", default_value="false"),
            DeclareLaunchArgument("crossing_freeze_rail_targets", default_value="false"),
            DeclareLaunchArgument("crossing_trigger_x_threshold", default_value="0.35"),
            DeclareLaunchArgument("pre_crossing_cmd_vel_x", default_value="0.12"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=preclean_gazebo,
                    on_exit=[crossing_trial, crossing_check],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=crossing_check,
                    on_exit=[
                        cleanup_gazebo,
                        Shutdown(reason="dog2 window crossing check finished"),
                    ],
                )
            ),
            preclean_gazebo,
        ]
    )
