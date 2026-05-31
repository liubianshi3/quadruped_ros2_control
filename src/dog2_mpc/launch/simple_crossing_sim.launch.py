#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from dog2_motion_control.model_variant import get_urdf_xacro_filename


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    control_param_file = PathJoinSubstitution(
        [FindPackageShare("dog2_mpc"), "config", "dog2_ctrl_params.yaml"]
    )

    def _xacro_file(context):
        variant = LaunchConfiguration("model_variant").perform(context).strip() or "real"
        from dog2_motion_control.model_variant import normalize_model_variant
        variant = normalize_model_variant(variant)
        return PathJoinSubstitution(
            [FindPackageShare("dog2_description"), "urdf", get_urdf_xacro_filename(variant)]
        ).perform(context)

    robot_description = ParameterValue(Command(["xacro ", _xacro_file]), value_type=str)

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "dog2", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.5"],
        output="screen",
    )

    mpc_node_complete = Node(
        package="dog2_mpc",
        executable="mpc_node_complete",
        name="mpc_node_complete",
        output="screen",
        parameters=[
            control_param_file,
            {"use_sim_time": use_sim_time, "mode": "crossing", "robot_description": robot_description},
        ],
    )

    wbc_node_complete = Node(
        package="dog2_wbc",
        executable="wbc_node_complete",
        name="wbc_node_complete",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_variant", default_value="real"),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            mpc_node_complete,
            wbc_node_complete,
        ]
    )
