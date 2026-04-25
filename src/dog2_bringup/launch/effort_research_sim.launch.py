#!/usr/bin/env python3

import os
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _prefer_workspace_package_dir(package_name: str, installed_share_dir: str) -> str:
    share_path = Path(installed_share_dir).resolve()
    try:
        workspace_root = share_path.parents[3]
    except IndexError:
        return installed_share_dir

    source_dir = workspace_root / "src" / package_name
    if source_dir.is_dir():
        return str(source_dir)
    return installed_share_dir


def _collect_gz_resource_roots(*package_dirs: str) -> str:
    roots = []
    for package_dir in package_dirs:
        if not package_dir:
            continue
        resource_root = str(Path(package_dir).resolve().parent)
        if resource_root not in roots:
            roots.append(resource_root)

    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    for resource_root in existing.split(":"):
        if resource_root and resource_root not in roots:
            roots.append(resource_root)

    return ":".join(roots)


def _load_standing_pose(config_file: str) -> dict[str, dict[str, float]]:
    standing_pose = {
        "lf": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
        "lh": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
        "rh": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
        "rf": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
    }

    try:
        with open(config_file, "r", encoding="utf-8") as handle:
            raw_cfg = yaml.safe_load(handle) or {}
        node_cfg = raw_cfg.get("spider_robot_controller", {})
        ros_params = node_cfg.get("ros__parameters", {}) if isinstance(node_cfg, dict) else {}
        cfg_pose = ros_params.get("standing_pose", {})
        if isinstance(cfg_pose, dict):
            for leg in ("lf", "lh", "rh", "rf"):
                leg_cfg = cfg_pose.get(leg, {})
                if isinstance(leg_cfg, dict):
                    standing_pose[leg].update(leg_cfg)
    except Exception as exc:
        print(f"[effort_research_sim.launch] Failed to load standing_pose from {config_file}: {exc}")

    return standing_pose


def generate_launch_description() -> LaunchDescription:
    world_default = PathJoinSubstitution([FindPackageShare("dog2_bringup"), "worlds", "flat_ground.sdf"])
    gait_config_default = PathJoinSubstitution(
        [FindPackageShare("dog2_motion_control"), "config", "gait_params.yaml"]
    )

    pkg_dog2_description_install = get_package_share_directory("dog2_description")
    pkg_dog2_motion_control_install = get_package_share_directory("dog2_motion_control")
    pkg_gazebo_ros = get_package_share_directory("ros_gz_sim")
    pkg_dog2_description = _prefer_workspace_package_dir("dog2_description", pkg_dog2_description_install)
    pkg_dog2_motion_control = _prefer_workspace_package_dir("dog2_motion_control", pkg_dog2_motion_control_install)

    def launch_setup(context):
        controllers_yaml = os.path.join(pkg_dog2_motion_control, "config", "effort_controllers.yaml")
        mass_scale = LaunchConfiguration("mass_scale").perform(context)
        p_gain = LaunchConfiguration("p_gain").perform(context)
        controller_manager_name = LaunchConfiguration("controller_manager_name").perform(context)
        odom_gz_topic = LaunchConfiguration("odom_gz_topic").perform(context)
        odom_topic = LaunchConfiguration("odom_topic").perform(context)
        external_odom_topic = f"{odom_topic}_external"
        dynamic_pose_gz_topic = LaunchConfiguration("dynamic_pose_gz_topic").perform(context)
        dynamic_pose_ros_topic = LaunchConfiguration("dynamic_pose_ros_topic").perform(context)
        model_name = LaunchConfiguration("model_name").perform(context)
        gz_world_name = LaunchConfiguration("gz_world_name").perform(context)
        bridge_foot_contact = LaunchConfiguration("bridge_foot_contact").perform(context).lower() in (
            "1",
            "true",
            "yes",
        )
        config_file = LaunchConfiguration("config_file").perform(context)
        spawn_z = LaunchConfiguration("spawn_z").perform(context)
        try:
            spawn_z_value = float(spawn_z)
        except ValueError:
            spawn_z_value = 1.05

        # `system.launch.py` keeps a conservative high spawn for the legacy
        # position-mode branch. In the effort research path that height causes
        # an unnecessary free-fall window before the effort controller takes over.
        if spawn_z_value > 0.75:
            spawn_z_value = 0.45

        set_gazebo_model_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=_collect_gz_resource_roots(pkg_dog2_description, pkg_dog2_description_install),
        )

        xacro_file = os.path.join(pkg_dog2_description, "urdf", "dog2.urdf.xacro")
        robot_description_xml = xacro.process_file(
            xacro_file,
            mappings={
                "controllers_yaml": controllers_yaml,
                "mass_scale": mass_scale,
                "control_mode": "effort",
            },
        ).toxml()

        gain_tag = "<position_proportional_gain>1.0</position_proportional_gain>"
        if gain_tag in robot_description_xml:
            robot_description_xml = robot_description_xml.replace(
                gain_tag,
                f"<position_proportional_gain>{float(p_gain):.6f}</position_proportional_gain>",
                1,
            )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description_xml},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        )

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r ", LaunchConfiguration("world"), " "],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gui")),
        )

        gazebo_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r -s ", LaunchConfiguration("world")],
                "on_exit_shutdown": "true",
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("use_gui")),
        )

        clock_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )

        odom_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{odom_gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            remappings=[(odom_gz_topic, external_odom_topic)],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_gz_odom_bridge")),
        )

        dynamic_pose_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{dynamic_pose_gz_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            remappings=[(dynamic_pose_gz_topic, dynamic_pose_ros_topic)],
            output="screen",
        )

        pose_to_odom_node = Node(
            package="dog2_motion_control",
            executable="gz_pose_to_odom",
            name="gz_pose_to_odom",
            output="screen",
            parameters=[
                {
                    "pose_topic": dynamic_pose_ros_topic,
                    "odom_topic": odom_topic,
                    "external_odom_topic": external_odom_topic,
                    "model_name": model_name,
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "publish_only_when_no_external_odom": True,
                }
            ],
        )

        gz_gain_setter = Node(
            package="dog2_motion_control",
            executable="gz_gain_setter",
            name="gz_gain_setter",
            output="screen",
            parameters=[
                {
                    "target_node": "/gz_ros2_control",
                    "gain": LaunchConfiguration("p_gain"),
                    "timeout_s": 12.0,
                }
            ],
        )

        standing_pose = _load_standing_pose(config_file)
        spawn_joint_args = []
        for leg_prefix in ("lf", "lh", "rh", "rf"):
            leg_pose = standing_pose[leg_prefix]
            spawn_joint_args.extend(
                [
                    "-J",
                    f"{leg_prefix}_rail_joint",
                    str(float(leg_pose["rail_m"])),
                    "-J",
                    f"{leg_prefix}_coxa_joint",
                    str(float(leg_pose["hip_roll_rad"])),
                    "-J",
                    f"{leg_prefix}_femur_joint",
                    str(float(leg_pose["hip_pitch_rad"])),
                    "-J",
                    f"{leg_prefix}_tibia_joint",
                    str(float(leg_pose["knee_pitch_rad"])),
                ]
            )

        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "/robot_description",
                "-name",
                model_name,
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                str(spawn_z_value),
            ]
            + spawn_joint_args,
            output="screen",
        )

        load_joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
                "--switch-timeout",
                "30",
            ],
            output="screen",
        )

        load_effort_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "effort_controller",
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
                "--switch-timeout",
                "30",
            ],
            output="screen",
        )

        foot_contact_bridges = []
        if bridge_foot_contact:
            for leg in ("lf", "lh", "rh", "rf"):
                gz_topic = (
                    f"/world/{gz_world_name}/model/{model_name}/link/{leg}_foot_link/"
                    f"sensor/{leg}_foot_contact/contacts"
                )
                ros_topic = f"/dog2/foot_contact/{leg}"
                foot_contact_bridges.append(
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        arguments=[
                            f"{gz_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                            "--ros-args",
                            "-r",
                            f"{gz_topic}:={ros_topic}",
                        ],
                        output="screen",
                    )
                )

        delayed_spawn_entity = TimerAction(
            period=LaunchConfiguration("spawn_delay_sec"),
            actions=[spawn_entity],
        )
        start_joint_state_broadcaster_after_spawn = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])
        )
        start_gain_setter_after_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[gz_gain_setter])
        )
        start_effort_controller_after_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_effort_controller])
        )

        return [
            set_gazebo_model_path,
            gazebo,
            gazebo_headless,
            robot_state_publisher,
            clock_bridge,
            odom_bridge,
            dynamic_pose_bridge,
            pose_to_odom_node,
            start_joint_state_broadcaster_after_spawn,
            start_gain_setter_after_jsb,
            start_effort_controller_after_jsb,
            delayed_spawn_entity,
            *foot_contact_bridges,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=gait_config_default),
            DeclareLaunchArgument("mass_scale", default_value="1.0"),
            DeclareLaunchArgument("p_gain", default_value="1.5"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument("spawn_delay_sec", default_value="4.0"),
            DeclareLaunchArgument("spawn_z", default_value="1.05"),
            DeclareLaunchArgument("spawn_z_margin", default_value="0.040"),
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
            DeclareLaunchArgument("dynamic_pose_ros_topic", default_value="/dog2/dynamic_pose_tf"),
            DeclareLaunchArgument("model_name", default_value="dog2"),
            DeclareLaunchArgument("gz_world_name", default_value="dog2_flat_ground"),
            DeclareLaunchArgument("bridge_foot_contact", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
