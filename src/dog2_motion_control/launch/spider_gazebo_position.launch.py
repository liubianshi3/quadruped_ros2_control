#!/usr/bin/env python3
"""
Dog2 position-mode + Gazebo 完整仿真启动（双路架构中的 position 分支）。

保留你现有稳定链路：URDF command_interface=position + JTC/rail_position 控制器 + spider_controller。
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import xacro


def _resolve_world_name(world_path: str) -> str:
    world_path = str(world_path).strip()
    if world_path and os.path.isfile(world_path):
        try:
            root = ET.parse(world_path).getroot()
            world_elem = root.find("world")
            if world_elem is not None and world_elem.get("name"):
                return str(world_elem.get("name"))
        except Exception:
            pass

    if world_path:
        world_stem = os.path.splitext(os.path.basename(world_path))[0].strip()
        if world_stem:
            return world_stem

    return "empty"


def generate_launch_description():
    pkg_dog2_description = get_package_share_directory("dog2_description")
    pkg_dog2_motion_control = get_package_share_directory("dog2_motion_control")
    pkg_gazebo_ros = get_package_share_directory("ros_gz_sim")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("dog2_motion_control"), "config", "gait_params.yaml"]
        ),
        description="步态参数配置文件路径",
    )

    mass_scale_arg = DeclareLaunchArgument(
        "mass_scale",
        default_value="1.0",
        description="URDF inertial mass/inertia scaling for A/B testing",
    )

    p_gain_arg = DeclareLaunchArgument(
        "p_gain",
        default_value="1.5",
        description="Set /gz_ros2_control position_proportional_gain after startup",
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="是否启动Gazebo GUI",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="是否使用仿真时间",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf",
        description="Gazebo世界文件路径",
    )

    def launch_setup(context):
        controllers_yaml = os.path.join(pkg_dog2_description, "config", "ros2_controllers.yaml")
        mass_scale = LaunchConfiguration("mass_scale").perform(context)
        world_path = LaunchConfiguration("world").perform(context)
        world_name = _resolve_world_name(world_path)

        gazebo_model_path = os.path.join(pkg_dog2_description, "..")
        if "GZ_SIM_RESOURCE_PATH" in os.environ:
            gazebo_model_path = os.environ["GZ_SIM_RESOURCE_PATH"] + ":" + gazebo_model_path

        set_gazebo_model_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gazebo_model_path,
        )

        xacro_file = os.path.join(pkg_dog2_description, "urdf", "dog2.urdf.xacro")
        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={
                "controllers_yaml": controllers_yaml,
                "mass_scale": mass_scale,
                "control_mode": "position",
            },
        )
        robot_description = {"robot_description": robot_description_config.toxml()}

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": [f"{world_path} "]}.items(),
            condition=IfCondition(LaunchConfiguration("use_gui")),
        )

        gazebo_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": [f"-s {world_path} "]}.items(),
            condition=UnlessCondition(LaunchConfiguration("use_gui")),
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        )

        clock_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )

        world_control_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"/world/{world_name}/control@ros_gz_interfaces/srv/ControlWorld"],
            output="screen",
        )

        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "/robot_description",
                "-name",
                "dog2",
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                "0.8",
            ],
            output="screen",
        )

        load_joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
            output="screen",
        )

        load_joint_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
            output="screen",
        )

        load_rail_position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rail_position_controller", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
            output="screen",
        )

        spider_controller_node = Node(
            package="dog2_motion_control",
            executable="spider_controller",
            name="spider_robot_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time"), "debug_mode": True},
            ],
        )

        gz_gain_setter = Node(
            package="dog2_motion_control",
            executable="gz_gain_setter",
            name="gz_gain_setter",
            output="screen",
            parameters=[{"target_node": "/gz_ros2_control", "gain": LaunchConfiguration("p_gain")}],
        )

        startup_gate = Node(
            package="dog2_motion_control",
            executable="gz_startup_gate",
            name="gz_startup_gate",
            output="screen",
            parameters=[
                {
                    "controller_manager_name": "/controller_manager",
                    "required_controllers": [
                        "joint_state_broadcaster",
                        "joint_trajectory_controller",
                        "rail_position_controller",
                    ],
                    "ready_topic": "/spider_startup_ready",
                    "world_name": world_name,
                }
            ],
        )

        start_joint_trajectory = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        )

        start_rail_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_rail_position_controller],
            )
        )

        start_remaining_nodes = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rail_position_controller,
                on_exit=[spider_controller_node],
            )
        )

        wait_for_spawn = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        )

        return [
            set_gazebo_model_path,
            gazebo,
            gazebo_headless,
            clock_bridge,
            world_control_bridge,
            robot_state_publisher,
            spawn_entity,
            wait_for_spawn,
            start_joint_trajectory,
            start_rail_controller,
            start_remaining_nodes,
            gz_gain_setter,
            startup_gate,
        ]

    return LaunchDescription(
        [
            config_file_arg,
            mass_scale_arg,
            p_gain_arg,
            use_gui_arg,
            use_sim_time_arg,
            world_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
