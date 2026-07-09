#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    model_variant = LaunchConfiguration("model_variant")
    software_rendering = LaunchConfiguration("software_rendering")
    control_param_file = PathJoinSubstitution(
        [FindPackageShare("dog2_mpc"), "config", "dog2_ctrl_params.yaml"]
    )

    xacro_filename = PythonExpression([
        "'dog2_symmetric.urdf.xacro' if '",
        model_variant,
        "' == 'symmetric' else 'dog2.urdf.xacro'",
    ])
    xacro_file = PathJoinSubstitution([FindPackageShare("dog2_description"), "urdf", xacro_filename])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"verbose": "false", "gui": "true", "pause": "false"}.items(),
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
            {"use_sim_time": use_sim_time, "mode": mode, "robot_description": robot_description},
        ],
    )

    wbc_node_complete = Node(
        package="dog2_wbc",
        executable="wbc_node_complete",
        name="wbc_node_complete",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="hover"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_variant", default_value="symmetric", choices=["real", "symmetric"]),
            DeclareLaunchArgument("software_rendering", default_value="1", choices=["0", "1"]),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", software_rendering),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            gazebo_launch,
            robot_state_publisher,
            spawn_entity,
            mpc_node_complete,
            wbc_node_complete,
        ]
    )
