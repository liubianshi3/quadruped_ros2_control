#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_mode", default_value="position"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dog2_motion_control"), "config", "gait_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("mass_scale", default_value="1.0"),
            DeclareLaunchArgument("p_gain", default_value="1.5"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=PathJoinSubstitution([FindPackageShare("dog2_bringup"), "worlds", "flat_ground.sdf"])),
            DeclareLaunchArgument("spawn_z", default_value="1.05"),
            DeclareLaunchArgument("spawn_z_margin", default_value="0.040"),
            DeclareLaunchArgument("spawn_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("controller_manager_name", default_value="/controller_manager"),
            DeclareLaunchArgument(
                "odom_gz_topic",
                default_value="/world/dog2_flat_ground/model/dog2/odometry",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("use_gz_odom_bridge", default_value="true"),
            DeclareLaunchArgument(
                "dynamic_pose_gz_topic",
                default_value="/world/dog2_flat_ground/dynamic_pose/info",
            ),
            DeclareLaunchArgument(
                "dynamic_pose_ros_topic", default_value="/dog2/dynamic_pose_tf"
            ),
            DeclareLaunchArgument("model_name", default_value="dog2"),
            DeclareLaunchArgument("gz_world_name", default_value="dog2_flat_ground"),
            DeclareLaunchArgument("bridge_foot_contact", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("teleop", default_value="true"),
            DeclareLaunchArgument("research_stack", default_value="false"),
            DeclareLaunchArgument("crossing_window_x_position", default_value="1.55"),
            DeclareLaunchArgument("crossing_window_width", default_value="0.48"),
            DeclareLaunchArgument("crossing_window_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_bottom_height", default_value="0.0"),
            DeclareLaunchArgument("crossing_window_top_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_safety_margin", default_value="0.04"),
            DeclareLaunchArgument("crossing_activation_distance", default_value="0.25"),
            DeclareLaunchArgument("crossing_approach_speed", default_value="0.15"),
            DeclareLaunchArgument("crossing_force_full_support", default_value="false"),
            DeclareLaunchArgument("crossing_freeze_rail_targets", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "sim_base.launch.py"])
                ),
                launch_arguments={
                    "controller_mode": LaunchConfiguration("controller_mode"),
                    "config_file": LaunchConfiguration("config_file"),
                    "mass_scale": LaunchConfiguration("mass_scale"),
                    "p_gain": LaunchConfiguration("p_gain"),
                    "use_gui": LaunchConfiguration("use_gui"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "world": LaunchConfiguration("world"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_z_margin": LaunchConfiguration("spawn_z_margin"),
                    "spawn_delay_sec": LaunchConfiguration("spawn_delay_sec"),
                    "controller_manager_name": LaunchConfiguration("controller_manager_name"),
                    "odom_gz_topic": LaunchConfiguration("odom_gz_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "use_gz_odom_bridge": LaunchConfiguration("use_gz_odom_bridge"),
                    "dynamic_pose_gz_topic": LaunchConfiguration("dynamic_pose_gz_topic"),
                    "dynamic_pose_ros_topic": LaunchConfiguration("dynamic_pose_ros_topic"),
                    "model_name": LaunchConfiguration("model_name"),
                    "gz_world_name": LaunchConfiguration("gz_world_name"),
                    "bridge_foot_contact": LaunchConfiguration("bridge_foot_contact"),
                    "research_stack": LaunchConfiguration("research_stack"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "control_stack.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "research_stack": LaunchConfiguration("research_stack"),
                    "controller_mode": LaunchConfiguration("controller_mode"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "visualization.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
            Node(
                package="dog2_bringup",
                executable="cmd_vel_teleop",
                name="dog2_cmd_vel_teleop",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("teleop")),
            ),
        ]
    )
