#!/usr/bin/env python3
"""
蜘蛛机器人完整仿真启动文件

功能：
1. 启动 Gazebo Fortress 仿真环境
2. 加载 Dog2 机器人模型
3. 启动 ros2_control 控制器
4. 启动蜘蛛机器人运动控制节点

使用方法：
ros2 launch dog2_motion_control spider_gazebo_complete.launch.py

可选参数：
- config_file: 步态参数配置文件路径
- use_gui: 是否启动Gazebo GUI (默认: true)
- world: 世界文件路径 (默认: empty.sdf)
- spawn_z: 生成高度 m（默认 1.05；过低易穿地、起立阶段塌倒）
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    DeclareLaunchArgument,
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


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_dog2_description_install = get_package_share_directory('dog2_description')
    pkg_dog2_motion_control_install = get_package_share_directory('dog2_motion_control')
    pkg_dog2_description = _prefer_workspace_package_dir('dog2_description', pkg_dog2_description_install)
    pkg_dog2_motion_control = _prefer_workspace_package_dir('dog2_motion_control', pkg_dog2_motion_control_install)
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('dog2_motion_control'),
            'config',
            'gait_params.yaml'
        ]),
        description='步态参数配置文件路径'
    )

    mass_scale_arg = DeclareLaunchArgument(
        'mass_scale',
        default_value='1.0',
        description='URDF inertial mass/inertia scaling for A/B testing'
    )

    p_gain_arg = DeclareLaunchArgument(
        'p_gain',
        default_value='1.5',
        description='Set /gz_ros2_control position_proportional_gain after startup'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf',
        description='Gazebo世界文件路径'
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='1.05',
        description='Spawn dog2 base height (m); higher reduces foot penetration / birth collapse'
    )
    def launch_setup(context):
        # 配置文件路径（不依赖 launch context）
        controllers_yaml = os.path.join(pkg_dog2_description, 'config', 'ros2_controllers.yaml')

        # URDF xacro 参数：mass_scale
        mass_scale = LaunchConfiguration('mass_scale').perform(context)
        world_path = LaunchConfiguration('world').perform(context)
        spawn_z = LaunchConfiguration('spawn_z').perform(context)
        world_name = _resolve_world_name(world_path)

        # 设置Gazebo模型路径环境变量
        set_gazebo_model_path = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=_collect_gz_resource_roots(pkg_dog2_description, pkg_dog2_description_install)
        )

        # 处理xacro文件生成URDF
        xacro_file = os.path.join(pkg_dog2_description, 'urdf', 'dog2.urdf.xacro')
        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={
                'controllers_yaml': controllers_yaml,
                'mass_scale': mass_scale,
            },
        )
        robot_description = {'robot_description': robot_description_config.toxml()}

        # 启动 Gazebo Fortress（必须带 -r 自动播放）
        # 若仿真从暂停开始：gz_ros2_control 的仿真步长为 0，controller switch 会 5s 超时；
        # gz_startup_gate 又要求控制器 active 后才 unpause → 死锁。与 spider_gazebo_mpc 一致使用 -r。
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [
                    '-r ',
                    LaunchConfiguration('world'),
                    ' ',
                ],
                'on_exit_shutdown': 'true'
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_gui'))
        )

        # 无 GUI：-r 播放 + -s 无图形（顺序与 mpc launch 一致）
        gazebo_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [
                    '-r -s ',
                    LaunchConfiguration('world'),
                ],
                'on_exit_shutdown': 'true'
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('use_gui'))
        )

        # Robot State Publisher节点
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

        # 显式桥接Gazebo时钟到ROS，避免控制节点出现“sim time not advancing”
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )

        world_control_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/world/{world_name}/control@ros_gz_interfaces/srv/ControlWorld'],
            output='screen'
        )

        # 在Gazebo中生成机器人
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'dog2',
                '-x', '0.0',
                '-y', '0.0',
                '-z', spawn_z,
            ],
            output='screen'
        )

        # 激活已在 controller_manager 参数中声明的控制器
        load_joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120',
            ],
            output='screen'
        )

        # 加载Joint Trajectory Controller（控制12个旋转关节）
        load_joint_trajectory_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_trajectory_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120',
            ],
            output='screen'
        )

        # 加载Rail Position Controller（控制4个导轨关节）
        load_rail_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'rail_position_controller',
                '-c', '/controller_manager',
                '--controller-manager-timeout', '120',
            ],
            output='screen'
        )

        # 启动蜘蛛机器人运动控制节点
        spider_controller_node = Node(
            package='dog2_motion_control',
            executable='spider_controller',
            name='spider_robot_controller',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time'), 'debug_mode': True}
            ],
        )

        # After Gazebo plugin node appears, set its position_proportional_gain once.
        gz_gain_setter = Node(
            package='dog2_motion_control',
            executable='gz_gain_setter',
            name='gz_gain_setter',
            output='screen',
            parameters=[
                {'target_node': '/gz_ros2_control', 'gain': LaunchConfiguration('p_gain')},
            ],
        )

        startup_gate = Node(
            package='dog2_motion_control',
            executable='gz_startup_gate',
            name='gz_startup_gate',
            output='screen',
            parameters=[
                {
                    'controller_manager_name': '/controller_manager',
                    'required_controllers': [
                        'joint_state_broadcaster',
                        'joint_trajectory_controller',
                        'rail_position_controller',
                    ],
                    'ready_topic': '/spider_startup_ready',
                    'world_name': world_name,
                },
            ],
        )

        # Joint State Broadcaster 启动后再启动轨迹控制器
        start_joint_trajectory = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        )

        # 轨迹控制器启动后再启动导轨控制器
        start_rail_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_rail_position_controller],
            )
        )

        # 导轨控制器启动后再启动主控制节点
        start_remaining_nodes = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rail_position_controller,
                on_exit=[
                    spider_controller_node,
                ],
            )
        )

        # 等待机器人生成后再加载控制器和运动控制节点
        wait_for_spawn = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                ],
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

    return LaunchDescription([
        config_file_arg,
        mass_scale_arg,
        p_gain_arg,
        use_gui_arg,
        use_sim_time_arg,
        world_arg,
        spawn_z_arg,
        OpaqueFunction(function=launch_setup),
    ])
