#!/usr/bin/env python3
"""
Dog2 effort-mode + MPC 架构分支启动文件（双路架构中的 mpc 分支）。

关键差异（按你的要求）：
1) URDF xacro 生成 robot_description 时传入 control_mode:=effort
2) controller_manager 参数文件从 ros2_controllers.yaml 替换为 effort_controllers.yaml
3) 仅加载 joint_state_broadcaster + effort_controller
4) 启动新增的 mpc_controller 节点（此处为算法骨架）
"""

import os
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

import xacro
import pinocchio as pin
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


def _parse_xyz_triplet(text, default):
    values = (text or default).split()
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got: {text!r}")
    return np.array([float(value) for value in values], dtype=float)


def _rpy_to_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rot_z @ rot_y @ rot_x


def _resolve_mesh_path(mesh_filename, urdf_dir, package_share_cache):
    if not mesh_filename:
        raise ValueError("Collision mesh filename is empty")

    if mesh_filename.startswith("package://"):
        package_path = mesh_filename[len("package://") :]
        package_name, relative_path = package_path.split("/", 1)
        package_share = package_share_cache.get(package_name)
        if package_share is None:
            package_share = get_package_share_directory(package_name)
            package_share_cache[package_name] = package_share
        return os.path.join(package_share, relative_path)

    if os.path.isabs(mesh_filename):
        return mesh_filename

    return os.path.normpath(os.path.join(urdf_dir, mesh_filename))


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


def _load_stl_vertices(mesh_path, mesh_vertices_cache):
    cached_vertices = mesh_vertices_cache.get(mesh_path)
    if cached_vertices is not None:
        return cached_vertices

    if os.path.splitext(mesh_path)[1].lower() != ".stl":
        raise ValueError(f"Unsupported collision mesh format: {mesh_path}")

    with open(mesh_path, "rb") as mesh_file:
        mesh_data = mesh_file.read()

    vertices = None
    if len(mesh_data) >= 84:
        triangle_count = struct.unpack_from("<I", mesh_data, 80)[0]
        expected_size = 84 + triangle_count * 50
        if expected_size == len(mesh_data):
            vertices = np.empty((triangle_count * 3, 3), dtype=np.float32)
            offset = 84
            vertex_index = 0
            for _ in range(triangle_count):
                offset += 12
                for _ in range(3):
                    vertices[vertex_index] = struct.unpack_from("<3f", mesh_data, offset)
                    vertex_index += 1
                    offset += 12
                offset += 2

    if vertices is None:
        ascii_vertices = []
        for raw_line in mesh_data.decode("utf-8", errors="ignore").splitlines():
            line = raw_line.strip()
            if line.startswith("vertex "):
                _, x_value, y_value, z_value = line.split()
                ascii_vertices.append((float(x_value), float(y_value), float(z_value)))
        if not ascii_vertices:
            raise ValueError(f"Failed to parse STL mesh vertices from {mesh_path}")
        vertices = np.asarray(ascii_vertices, dtype=float)
    else:
        vertices = vertices.astype(float)

    mesh_vertices_cache[mesh_path] = vertices
    return vertices


def _estimate_min_collision_z(robot_description_xml, pin_model, pin_data, urdf_dir, package_share_cache):
    robot_root = ET.fromstring(robot_description_xml)
    mesh_vertices_cache = {}
    min_collision_z = None
    collision_geom_count = 0

    for link in robot_root.findall("link"):
        link_name = link.attrib.get("name")
        if not link_name:
            continue

        link_frame_id = pin_model.getFrameId(link_name)
        link_pose = pin_data.oMf[link_frame_id]
        link_rotation = np.asarray(link_pose.rotation, dtype=float)
        link_translation = np.asarray(link_pose.translation, dtype=float).reshape(3, 1)

        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None:
                continue

            collision_origin = collision.find("origin")
            collision_xyz = _parse_xyz_triplet(
                collision_origin.attrib.get("xyz") if collision_origin is not None else None,
                "0 0 0",
            ).reshape(3, 1)
            collision_rpy = _parse_xyz_triplet(
                collision_origin.attrib.get("rpy") if collision_origin is not None else None,
                "0 0 0",
            )
            collision_rotation = _rpy_to_matrix(collision_rpy)

            mesh = geometry.find("mesh")
            box = geometry.find("box")
            sphere = geometry.find("sphere")

            if mesh is not None:
                mesh_path = _resolve_mesh_path(mesh.attrib.get("filename"), urdf_dir, package_share_cache)
                mesh_vertices = _load_stl_vertices(mesh_path, mesh_vertices_cache)
                mesh_scale = _parse_xyz_triplet(mesh.attrib.get("scale"), "1 1 1").reshape(1, 3)
                scaled_vertices = mesh_vertices * mesh_scale
                world_vertices = (
                    link_rotation @ (collision_rotation @ scaled_vertices.T + collision_xyz) + link_translation
                )
                link_min_collision_z = float(np.min(world_vertices[2]))
            elif box is not None:
                sx, sy, sz = [float(v) for v in box.attrib.get("size", "0 0 0").split()]
                hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
                corners = np.array(
                    [
                        [-hx, -hy, -hz],
                        [-hx, -hy, hz],
                        [-hx, hy, -hz],
                        [-hx, hy, hz],
                        [hx, -hy, -hz],
                        [hx, -hy, hz],
                        [hx, hy, -hz],
                        [hx, hy, hz],
                    ],
                    dtype=float,
                ).T
                world_vertices = link_rotation @ (collision_rotation @ corners + collision_xyz) + link_translation
                link_min_collision_z = float(np.min(world_vertices[2]))
            elif sphere is not None:
                r = float(sphere.attrib.get("radius", "0"))
                center_world = (link_rotation @ (collision_rotation @ np.zeros((3, 1)) + collision_xyz) + link_translation).reshape(
                    3,
                )
                link_min_collision_z = float(center_world[2] - r)
            else:
                continue

            min_collision_z = (
                link_min_collision_z if min_collision_z is None else min(min_collision_z, link_min_collision_z)
            )
            collision_geom_count += 1

    if collision_geom_count == 0 or min_collision_z is None:
        raise ValueError("No collision geometry (mesh/box/sphere) found in expanded URDF")

    return min_collision_z


def generate_launch_description():
    pkg_dog2_description_install = get_package_share_directory("dog2_description")
    pkg_dog2_motion_control_install = get_package_share_directory("dog2_motion_control")
    pkg_dog2_description = _prefer_workspace_package_dir("dog2_description", pkg_dog2_description_install)
    pkg_dog2_motion_control = _prefer_workspace_package_dir("dog2_motion_control", pkg_dog2_motion_control_install)
    pkg_gazebo_ros = get_package_share_directory("ros_gz_sim")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("dog2_motion_control"),
                "config",
                "gait_params.yaml",
            ]
        ),
        description="步态参数配置文件路径（当前 MPC 骨架暂无强依赖）",
    )

    mass_scale_arg = DeclareLaunchArgument(
        "mass_scale",
        default_value="1.0",
        description="URDF inertial mass/inertia scaling for A/B testing",
    )

    p_gain_arg = DeclareLaunchArgument(
        "p_gain",
        default_value="1.5",
        description="(position interface only) kept for parity with existing launch",
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
    spawn_z_margin_arg = DeclareLaunchArgument(
        "spawn_z_margin",
        default_value="0.040",
        description="spawn_z 的额外安全裕度(单位m)，用于避免脚端在初始碰撞约束下抽搐",
    )

    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value="/controller_manager",
        description="ros2_control controller_manager node name/namespace",
    )

    odom_gz_topic_arg = DeclareLaunchArgument(
        "odom_gz_topic",
        default_value="/world/empty/model/dog2/odometry",
        description="Gazebo odometry topic (gz transport side)",
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="ROS odometry topic consumed by mpc_controller",
    )

    use_gz_odom_bridge_arg = DeclareLaunchArgument(
        "use_gz_odom_bridge",
        default_value="false",
        description="是否启用 Gazebo 原生 /model/.../odometry 桥接；默认关闭，改用 dynamic_pose 回退链路",
    )

    dynamic_pose_gz_topic_arg = DeclareLaunchArgument(
        "dynamic_pose_gz_topic",
        default_value="/world/empty/dynamic_pose/info",
        description="Gazebo dynamic pose topic (Pose_V) for odom fallback",
    )

    dynamic_pose_ros_topic_arg = DeclareLaunchArgument(
        "dynamic_pose_ros_topic",
        default_value="/dog2/dynamic_pose_tf",
        description="ROS TFMessage topic bridged from Gazebo dynamic pose",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="dog2",
        description="Model name used by pose->odom fallback node",
    )

    gz_world_name_arg = DeclareLaunchArgument(
        "gz_world_name",
        default_value="empty",
        description="Gazebo world 名（与 /world/<name>/... 话题一致，默认 empty.sdf）",
    )

    bridge_foot_contact_arg = DeclareLaunchArgument(
        "bridge_foot_contact",
        default_value="true",
        description="是否桥接四足 foot_link 上 gz contact sensor 到 /dog2/foot_contact/*",
    )

    def launch_setup(context):
        # === 关键 1：替换 controllers YAML 到 effort 模式 ===
        controllers_yaml = os.path.join(
            pkg_dog2_motion_control,
            "config",
            "effort_controllers.yaml",
        )

        mass_scale = LaunchConfiguration("mass_scale").perform(context)
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
        p_gain = float(LaunchConfiguration("p_gain").perform(context))
        spawn_z_margin = float(LaunchConfiguration("spawn_z_margin").perform(context))

        # 设置 Gazebo 模型路径环境变量
        set_gazebo_model_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=_collect_gz_resource_roots(pkg_dog2_description, pkg_dog2_description_install),
        )

        # === 关键 2：xacro 生成 robot_description 传入 control_mode:=effort ===
        xacro_file = os.path.join(pkg_dog2_description, "urdf", "dog2.urdf.xacro")
        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={
                "controllers_yaml": controllers_yaml,
                "mass_scale": mass_scale,
                "control_mode": "effort",
            },
        )
        robot_description_xml = robot_description_config.toxml()
        gain_tag = "<position_proportional_gain>1.0</position_proportional_gain>"
        if gain_tag in robot_description_xml:
            robot_description_xml = robot_description_xml.replace(
                gain_tag,
                f"<position_proportional_gain>{p_gain:.6f}</position_proportional_gain>",
                1,
            )
        else:
            print(
                "[spider_gazebo_mpc.launch] Warning: position_proportional_gain tag not found in robot_description"
            )
        robot_description = {"robot_description": robot_description_xml}

        standing_pose_defaults = {
            "lf": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
            "lh": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
            "rh": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
            "rf": {"rail_m": 0.0, "hip_roll_rad": 0.0, "hip_pitch_rad": -0.3, "knee_pitch_rad": -0.6},
        }
        standing_pose = standing_pose_defaults
        try:
            with open(config_file, "r", encoding="utf-8") as f:
                raw_cfg = yaml.safe_load(f) or {}
            node_cfg = raw_cfg.get("spider_robot_controller", {})
            ros_params = node_cfg.get("ros__parameters", {}) if isinstance(node_cfg, dict) else {}
            cfg_pose = ros_params.get("standing_pose", {})
            if isinstance(cfg_pose, dict):
                merged_pose = {}
                for leg in ("lf", "lh", "rh", "rf"):
                    leg_defaults = dict(standing_pose_defaults[leg])
                    leg_cfg = cfg_pose.get(leg, {})
                    if isinstance(leg_cfg, dict):
                        leg_defaults.update(leg_cfg)
                    merged_pose[leg] = leg_defaults
                standing_pose = merged_pose
        except Exception as exc:
            print(f"[spider_gazebo_mpc.launch] Failed to load standing_pose from {config_file}: {exc}")

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

        spawn_z = 0.45
        fallback_spawn_z = None
        pin_model = None
        pin_data = None
        try:
            pin_model = pin.buildModelFromXML(robot_description_xml, pin.JointModelFreeFlyer())
            pin_data = pin_model.createData()
            q = pin.neutral(pin_model)
            q[:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            for leg_prefix in ("lf", "lh", "rh", "rf"):
                leg_pose = standing_pose[leg_prefix]
                q[pin_model.joints[pin_model.getJointId(f"{leg_prefix}_rail_joint")].idx_q] = float(leg_pose["rail_m"])
                q[pin_model.joints[pin_model.getJointId(f"{leg_prefix}_coxa_joint")].idx_q] = float(
                    leg_pose["hip_roll_rad"]
                )
                q[pin_model.joints[pin_model.getJointId(f"{leg_prefix}_femur_joint")].idx_q] = float(
                    leg_pose["hip_pitch_rad"]
                )
                q[pin_model.joints[pin_model.getJointId(f"{leg_prefix}_tibia_joint")].idx_q] = float(
                    leg_pose["knee_pitch_rad"]
                )

            pin.forwardKinematics(pin_model, pin_data, q)
            pin.updateFramePlacements(pin_model, pin_data)
            min_foot_z = min(
                float(pin_data.oMf[pin_model.getFrameId(f"{leg_prefix}_foot_link")].translation[2])
                for leg_prefix in ("lf", "lh", "rh", "rf")
            )
            fallback_spawn_z = max(0.25, -min_foot_z + spawn_z_margin)
            spawn_z = fallback_spawn_z
        except Exception as exc:
            print(f"[spider_gazebo_mpc.launch] Failed to estimate spawn_z from standing_pose: {exc}")

        if pin_model is not None and pin_data is not None:
            try:
                min_collision_z = _estimate_min_collision_z(
                    robot_description_xml=robot_description_xml,
                    pin_model=pin_model,
                    pin_data=pin_data,
                    urdf_dir=os.path.dirname(xacro_file),
                    package_share_cache={"dog2_description": pkg_dog2_description},
                )
                spawn_z = max(0.25, -min_collision_z + spawn_z_margin)
                print(
                    "[spider_gazebo_mpc.launch] collision-aware spawn_z="
                    f"{spawn_z:.3f} m (min_collision_z={min_collision_z:.3f} m)"
                )
            except Exception as exc:
                if fallback_spawn_z is not None:
                    print(
                        "[spider_gazebo_mpc.launch] Failed to estimate collision-aware spawn_z; "
                        f"keeping standing_pose fallback {fallback_spawn_z:.3f} m: {exc}"
                    )
                else:
                    print(f"[spider_gazebo_mpc.launch] Failed to estimate collision-aware spawn_z: {exc}")

        # 启动 Gazebo Fortress
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": [
                    "-r ",
                    LaunchConfiguration("world"),
                    " ",
                ],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gui")),
        )

        # 无GUI模式
        gazebo_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": [
                    "-r -s ",
                    LaunchConfiguration("world"),
                ],
                "on_exit_shutdown": "true",
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("use_gui")),
        )

        # Robot State Publisher
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

        # Clock bridge（Gazebo -> ROS）
        clock_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )

        # Odometry bridge（Gazebo -> ROS）
        odom_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"{odom_gz_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            remappings=[
                (odom_gz_topic, external_odom_topic),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_gz_odom_bridge")),
        )

        # Dynamic pose bridge (Pose_V -> TFMessage), used as odom fallback source.
        dynamic_pose_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"{dynamic_pose_gz_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            remappings=[
                (dynamic_pose_gz_topic, dynamic_pose_ros_topic),
            ],
            output="screen",
        )

        # Fallback adapter: build /odom from bridged dynamic pose stream.
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

        # 在 Gazebo 中生成机器人
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
                str(spawn_z),
            ]
            + spawn_joint_args,
            output="screen",
        )

        # === 关键 3：仅加载 joint_state_broadcaster + effort_controller ===
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

        # MPC controller node（算法骨架）
        # 提前启动，让节点先完成订阅/初始化；它会自行等待 /joint_states。
        # 这样可以在 effort_controller 激活后更快进入关节保持，减少出生后的自由下落窗口。
        mpc_controller_node = Node(
            package="dog2_motion_control",
            executable="mpc_controller",
            name="mpc_robot_controller",
            output="screen",
            parameters=[
                {
                    "config_file": LaunchConfiguration("config_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "debug_mode": True,
                    "odom_topic": odom_topic,
                    "use_gz_foot_contact": bridge_foot_contact,
                    "gz_contact_topic_lf": "/dog2/foot_contact/lf",
                    "gz_contact_topic_lh": "/dog2/foot_contact/lh",
                    "gz_contact_topic_rh": "/dog2/foot_contact/rh",
                    "gz_contact_topic_rf": "/dog2/foot_contact/rf",
                },
            ],
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

        start_joint_state_broadcaster_after_gain = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_gain_setter,
                on_exit=[load_joint_state_broadcaster],
            )
        )

        start_effort_controller_after_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_effort_controller],
            )
        )

        start_gain_setter_after_spawn = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[gz_gain_setter],
            )
        )

        return [
            set_gazebo_model_path,
            gazebo,
            gazebo_headless,
            clock_bridge,
            odom_bridge,
            dynamic_pose_bridge,
            robot_state_publisher,
            *foot_contact_bridges,
            mpc_controller_node,
            spawn_entity,
            start_gain_setter_after_spawn,
            start_joint_state_broadcaster_after_gain,
            start_effort_controller_after_jsb,
            pose_to_odom_node,
        ]

    return LaunchDescription(
        [
            config_file_arg,
            mass_scale_arg,
            p_gain_arg,
            use_gui_arg,
            use_sim_time_arg,
            world_arg,
            spawn_z_margin_arg,
            controller_manager_name_arg,
            odom_gz_topic_arg,
            odom_topic_arg,
            use_gz_odom_bridge_arg,
            dynamic_pose_gz_topic_arg,
            dynamic_pose_ros_topic_arg,
            model_name_arg,
            gz_world_name_arg,
            bridge_foot_contact_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
