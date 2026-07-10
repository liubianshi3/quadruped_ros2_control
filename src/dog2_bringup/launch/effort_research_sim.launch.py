#!/usr/bin/env python3

import os
import tempfile
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


def _make_effort_controllers_yaml(base_yaml: str, p_gain: str) -> str:
    with open(base_yaml, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    gain = float(p_gain)
    for node_name in ("gz_ros_control", "gz_ros2_control"):
        node_cfg = config.setdefault(node_name, {})
        ros_params = node_cfg.setdefault("ros__parameters", {})
        ros_params["hold_joints"] = True
        ros_params["position_proportional_gain"] = gain

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="dog2_effort_controllers_",
        suffix=".yaml",
        delete=False,
    )
    with tmp:
        yaml.safe_dump(config, tmp, sort_keys=False)
    return tmp.name


def _estimate_spawn_z_from_standing_pose(
    robot_description_xml: str,
    standing_pose: dict[str, dict[str, float]],
    margin: float,
) -> float:
    try:
        import numpy as np
        import pinocchio as pin

        model = pin.buildModelFromXML(robot_description_xml)
        data = model.createData()
        q = np.zeros(model.nq)

        for leg_prefix in ("lf", "lh", "rh", "rf"):
            leg_pose = standing_pose[leg_prefix]
            joint_values = {
                f"{leg_prefix}_rail_joint": float(leg_pose["rail_m"]),
                f"{leg_prefix}_coxa_joint": float(leg_pose["hip_roll_rad"]),
                f"{leg_prefix}_femur_joint": float(leg_pose["hip_pitch_rad"]),
                f"{leg_prefix}_tibia_joint": float(leg_pose["knee_pitch_rad"]),
            }
            for joint_name, value in joint_values.items():
                joint_id = model.getJointId(joint_name)
                q[model.idx_qs[joint_id]] = value

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        foot_z_values = [
            float(data.oMf[model.getFrameId(f"{leg_prefix}_foot_link")].translation[2])
            for leg_prefix in ("lf", "lh", "rh", "rf")
        ]
        return max(0.05, -min(foot_z_values) + margin)
    except Exception as exc:
        fallback_spawn_z = 0.30
        print(
            "[effort_research_sim.launch] Failed to estimate standing spawn_z "
            f"from URDF/standing_pose ({exc}); using fallback {fallback_spawn_z:.3f}"
        )
        return fallback_spawn_z


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
        mass_scale = LaunchConfiguration("mass_scale").perform(context)
        p_gain = LaunchConfiguration("p_gain").perform(context)
        base_controllers_yaml = os.path.join(pkg_dog2_motion_control, "config", "effort_controllers.yaml")
        controllers_yaml = _make_effort_controllers_yaml(base_controllers_yaml, p_gain)
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
        enable_acceptance_contact_sensors = (
            LaunchConfiguration("enable_acceptance_contact_sensors").perform(context).lower()
            in ("1", "true", "yes")
        )
        config_file = LaunchConfiguration("config_file").perform(context)
        spawn_z = LaunchConfiguration("spawn_z").perform(context)
        spawn_z_margin = LaunchConfiguration("spawn_z_margin").perform(context)
        try:
            requested_spawn_z_value = float(spawn_z)
        except ValueError:
            requested_spawn_z_value = 1.05
        try:
            spawn_z_margin_value = float(spawn_z_margin)
        except ValueError:
            spawn_z_margin_value = 0.04

        set_gazebo_model_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=_collect_gz_resource_roots(pkg_dog2_description, pkg_dog2_description_install),
        )

        model_variant_raw = LaunchConfiguration("model_variant").perform(context).strip()
        from dog2_motion_control.model_variant import normalize_model_variant, get_urdf_xacro_filename
        variant = normalize_model_variant(model_variant_raw or "symmetric")
        xacro_file = os.path.join(pkg_dog2_description, "urdf", get_urdf_xacro_filename(variant))
        robot_description_xml = xacro.process_file(
            xacro_file,
            mappings={
                "controllers_yaml": controllers_yaml,
                "robot_param_node": "gz_robot_description_server",
                "mass_scale": mass_scale,
                "control_mode": "effort",
                "enable_acceptance_contact_sensors": (
                    "true" if enable_acceptance_contact_sensors else "false"
                ),
            },
        ).toxml()

        gain_tag = "<position_proportional_gain>1.0</position_proportional_gain>"
        if gain_tag in robot_description_xml:
            robot_description_xml = robot_description_xml.replace(
                gain_tag,
                f"<position_proportional_gain>{float(p_gain):.6f}</position_proportional_gain>",
                1,
            )
        gz_robot_description_server = Node(
            package="dog2_bringup",
            executable="robot_description_server",
            name="gz_robot_description_server",
            output="screen",
            parameters=[
                {"robot_description": robot_description_xml},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
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

        standing_pose = _load_standing_pose(config_file)
        # `system.launch.py` keeps a conservative high spawn for the legacy
        # position-mode branch. In the effort research path, derive the spawn
        # height from the actual standing foot height so Gazebo does not start
        # with a large free-fall before the effort controller takes over.
        if requested_spawn_z_value > 0.75:
            spawn_z_value = _estimate_spawn_z_from_standing_pose(
                robot_description_xml,
                standing_pose,
                spawn_z_margin_value,
            )
            print(
                "[effort_research_sim.launch] Auto effort spawn_z="
                f"{spawn_z_value:.3f} from standing_pose + margin {spawn_z_margin_value:.3f}"
            )
        else:
            spawn_z_value = requested_spawn_z_value

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
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
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

        load_rail_position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "rail_position_controller",
                "--controller-manager",
                controller_manager_name,
                "--controller-manager-timeout",
                "120",
                "--switch-timeout",
                "30",
            ],
            output="screen",
        )

        rail_lock_commander = Node(
            package="dog2_bringup",
            executable="rail_lock_commander",
            name="rail_lock_commander",
            output="screen",
            parameters=[
                {
                    "output_topic": "/rail_position_controller/commands",
                    "rail_targets": [0.0, 0.0, 0.0, 0.0],
                    "publish_rate_hz": 20.0,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )

        foot_contact_bridges = []
        if bridge_foot_contact:
            for leg in ("lf", "lh", "rh", "rf"):
                # The sensor owns an explicit transport topic.  This avoids
                # coupling the bridge to libsdformat's fixed-joint-lumped link
                # and sensor topic names.
                gz_topic = f"/dog2/gz_contact/{leg}_foot"
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

        acceptance_contact_bridges = []
        if enable_acceptance_contact_sensors:
            acceptance_contacts = [
                ("/dog2/gz_contact/base", "/dog2/contact/base"),
                *[
                    (
                        f"/dog2/gz_contact/{leg}_tibia",
                        f"/dog2/contact/{leg}_tibia",
                    )
                    for leg in ("lf", "lh", "rh", "rf")
                ],
            ]
            for gz_topic, ros_topic in acceptance_contacts:
                acceptance_contact_bridges.append(
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
        # Claim the rail position interfaces as early as possible: right after
        # the joint_state_broadcaster, while the rails are still at their
        # spawn positions (~0). Activating the lock later would let landing
        # impacts drift the rails first and then snap them back violently.
        start_rail_controller_after_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_rail_position_controller, rail_lock_commander],
            )
        )
        # The 12 rotational joints are 0 N (limp) until the effort controller
        # activates, so it must come up immediately after the rail lock; every
        # second here is a second of the robot collapsing under gravity. The
        # gain setter is not on the critical path (on gz_ros2_control 0.7.x it
        # cannot change the cached gain anyway) and runs last.
        start_effort_controller_after_rail = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rail_position_controller,
                on_exit=[load_effort_controller],
            )
        )
        start_gain_setter_after_effort = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_effort_controller,
                on_exit=[gz_gain_setter],
            )
        )

        return [
            set_gazebo_model_path,
            gz_robot_description_server,
            gazebo,
            gazebo_headless,
            robot_state_publisher,
            clock_bridge,
            odom_bridge,
            dynamic_pose_bridge,
            pose_to_odom_node,
            start_joint_state_broadcaster_after_spawn,
            start_rail_controller_after_jsb,
            start_effort_controller_after_rail,
            start_gain_setter_after_effort,
            delayed_spawn_entity,
            *foot_contact_bridges,
            *acceptance_contact_bridges,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=gait_config_default),
            DeclareLaunchArgument("model_variant", default_value="symmetric", choices=["real", "symmetric"]),
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
            DeclareLaunchArgument("enable_acceptance_contact_sensors", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
