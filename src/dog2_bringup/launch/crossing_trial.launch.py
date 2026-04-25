#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    obstacle_world = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "worlds", "window_frame.sdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ros_domain_id", default_value="45"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("teleop", default_value="false"),
            DeclareLaunchArgument("crossing_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("spawn_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("trigger_crossing", default_value="true"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "system.launch.py"])
                ),
                launch_arguments={
                    "controller_mode": "effort",
                    "research_stack": "true",
                    "use_gui": LaunchConfiguration("use_gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "teleop": LaunchConfiguration("teleop"),
                    "spawn_delay_sec": LaunchConfiguration("spawn_delay_sec"),
                    "world": obstacle_world,
                    "odom_gz_topic": "/world/dog2_window_frame/model/dog2/odometry",
                    "dynamic_pose_gz_topic": "/world/dog2_window_frame/dynamic_pose/info",
                    "gz_world_name": "dog2_window_frame",
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
                }.items(),
            ),
            Node(
                package="dog2_bringup",
                executable="crossing_trigger",
                name="dog2_crossing_trigger",
                output="screen",
                parameters=[
                    {
                        "enabled": ParameterValue(
                            LaunchConfiguration("trigger_crossing"),
                            value_type=bool,
                        ),
                        "delay_sec": ParameterValue(
                            LaunchConfiguration("crossing_delay_sec"),
                            value_type=float,
                        ),
                        "approach_cmd_vel_x": ParameterValue(
                            LaunchConfiguration("pre_crossing_cmd_vel_x"),
                            value_type=float,
                        ),
                        "trigger_when_x_ge": ParameterValue(
                            LaunchConfiguration("crossing_trigger_x_threshold"),
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("trigger_crossing")),
            ),
        ]
    )
