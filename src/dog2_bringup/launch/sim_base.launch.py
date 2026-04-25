#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world_default = PathJoinSubstitution([FindPackageShare("dog2_bringup"), "worlds", "flat_ground.sdf"])
    gait_config_default = PathJoinSubstitution(
        [FindPackageShare("dog2_motion_control"), "config", "gait_params.yaml"]
    )

    position_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dog2_motion_control"), "launch", "spider_gazebo_complete.launch.py"])
        ),
        launch_arguments={
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
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller_mode"), "' == 'position'"])
        ),
    )

    effort_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dog2_motion_control"), "launch", "spider_gazebo_mpc.launch.py"])
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "mass_scale": LaunchConfiguration("mass_scale"),
            "p_gain": LaunchConfiguration("p_gain"),
            "use_gui": LaunchConfiguration("use_gui"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world": LaunchConfiguration("world"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_z_margin": LaunchConfiguration("spawn_z_margin"),
            "controller_manager_name": LaunchConfiguration("controller_manager_name"),
            "odom_gz_topic": LaunchConfiguration("odom_gz_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "use_gz_odom_bridge": LaunchConfiguration("use_gz_odom_bridge"),
            "dynamic_pose_gz_topic": LaunchConfiguration("dynamic_pose_gz_topic"),
            "dynamic_pose_ros_topic": LaunchConfiguration("dynamic_pose_ros_topic"),
            "model_name": LaunchConfiguration("model_name"),
            "gz_world_name": LaunchConfiguration("gz_world_name"),
            "bridge_foot_contact": LaunchConfiguration("bridge_foot_contact"),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("controller_mode"),
                    "' == 'effort' and '",
                    LaunchConfiguration("research_stack"),
                    "' != 'true'",
                ]
            )
        ),
    )

    effort_research_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "effort_research_sim.launch.py"])
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "mass_scale": LaunchConfiguration("mass_scale"),
            "p_gain": LaunchConfiguration("p_gain"),
            "use_gui": LaunchConfiguration("use_gui"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world": LaunchConfiguration("world"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_z_margin": LaunchConfiguration("spawn_z_margin"),
            "controller_manager_name": LaunchConfiguration("controller_manager_name"),
            "odom_gz_topic": LaunchConfiguration("odom_gz_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "use_gz_odom_bridge": LaunchConfiguration("use_gz_odom_bridge"),
            "dynamic_pose_gz_topic": LaunchConfiguration("dynamic_pose_gz_topic"),
            "dynamic_pose_ros_topic": LaunchConfiguration("dynamic_pose_ros_topic"),
            "model_name": LaunchConfiguration("model_name"),
            "gz_world_name": LaunchConfiguration("gz_world_name"),
            "bridge_foot_contact": LaunchConfiguration("bridge_foot_contact"),
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("controller_mode"),
                    "' == 'effort' and '",
                    LaunchConfiguration("research_stack"),
                    "' == 'true'",
                ]
            )
        ),
    )

    position_odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            [LaunchConfiguration("odom_gz_topic"), "@nav_msgs/msg/Odometry[gz.msgs.Odometry"]
        ],
        remappings=[
            (LaunchConfiguration("odom_gz_topic"), LaunchConfiguration("external_odom_topic")),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("controller_mode"),
                    "' == 'position' and '",
                    LaunchConfiguration("use_gz_odom_bridge"),
                    "' == 'true'",
                ]
            )
        ),
    )

    position_dynamic_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            [LaunchConfiguration("dynamic_pose_gz_topic"), "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"]
        ],
        remappings=[
            (LaunchConfiguration("dynamic_pose_gz_topic"), LaunchConfiguration("dynamic_pose_ros_topic")),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller_mode"), "' == 'position'"])
        ),
    )

    position_pose_to_odom = Node(
        package="dog2_motion_control",
        executable="gz_pose_to_odom",
        name="gz_pose_to_odom",
        output="screen",
        parameters=[
            {
                "pose_topic": LaunchConfiguration("dynamic_pose_ros_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "external_odom_topic": LaunchConfiguration("external_odom_topic"),
                "model_name": LaunchConfiguration("model_name"),
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_only_when_no_external_odom": True,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("controller_mode"), "' == 'position'"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_mode", default_value="position"),
            DeclareLaunchArgument("config_file", default_value=gait_config_default),
            DeclareLaunchArgument("mass_scale", default_value="1.0"),
            DeclareLaunchArgument("p_gain", default_value="1.5"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument("spawn_z", default_value="1.05"),
            DeclareLaunchArgument("spawn_z_margin", default_value="0.040"),
            DeclareLaunchArgument("spawn_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("controller_manager_name", default_value="/controller_manager"),
            DeclareLaunchArgument(
                "odom_gz_topic",
                default_value="/world/dog2_flat_ground/model/dog2/odometry",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("external_odom_topic", default_value="/dog2/gazebo/odom"),
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
            DeclareLaunchArgument("research_stack", default_value="false"),
            position_launch,
            position_odom_bridge,
            position_dynamic_pose_bridge,
            position_pose_to_odom,
            effort_launch,
            effort_research_launch,
        ]
    )
