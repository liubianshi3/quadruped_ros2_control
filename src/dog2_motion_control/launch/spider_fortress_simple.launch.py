#!/usr/bin/env python3
"""
蜘蛛机器人Gazebo Fortress简化启动文件

这是一个简化版本，专注于让机器人正确显示在Gazebo中

使用方法：
ros2 launch dog2_motion_control spider_fortress_simple.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_dog2_description = get_package_share_directory('dog2_description')
    pkg_dog2_motion_control = get_package_share_directory('dog2_motion_control')
    
    # 配置文件路径
    controllers_yaml = os.path.join(pkg_dog2_description, 'config', 'ros2_controllers.yaml')
    config_file = os.path.join(pkg_dog2_motion_control, 'config', 'gait_params.yaml')

    def _resolve_xacro_file(context):
        variant = LaunchConfiguration('model_variant').perform(context).strip()
        from dog2_motion_control.model_variant import normalize_model_variant, get_urdf_xacro_filename
        variant = normalize_model_variant(variant or "real")
        return os.path.join(pkg_dog2_description, 'urdf', get_urdf_xacro_filename(variant))
    
    # 处理xacro文件生成URDF
    robot_description_config = xacro.process_file(
        _resolve_xacro_file,
        mappings={'controllers_yaml': controllers_yaml},
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # 启动Gazebo Fortress
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # 创建机器人实体（使用gz命令行工具）
    # 等待Gazebo启动后再spawn机器人
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-topic', '/robot_description',
                    '-name', 'dog2',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.5',
                ],
                output='screen'
            )
        ]
    )
    
    # 桥接Gazebo和ROS 2话题
    # 桥接joint_states
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/dog2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '--ros-args', '-r', '/world/empty/model/dog2/joint_state:=/joint_states'
        ],
        output='screen'
    )
    
    # 桥接clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # 加载控制器（延迟启动，等待机器人spawn完成）
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    load_joint_trajectory_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller'],
                output='screen'
            )
        ]
    )
    
    load_rail_position_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['rail_position_controller'],
                output='screen'
            )
        ]
    )
    
    # 启动蜘蛛机器人运动控制节点
    spider_controller_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='dog2_motion_control',
                executable='spider_controller',
                name='spider_robot_controller',
                output='screen',
                parameters=[
                    config_file,
                    {'use_sim_time': True}
                ],
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("model_variant", default_value="real"),
        # 启动Robot State Publisher
        robot_state_publisher,
        
        # 启动Gazebo
        gazebo,
        
        # 桥接话题
        bridge_clock,
        bridge_joint_states,
        
        # Spawn机器人
        spawn_robot,
        
        # 加载控制器
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        load_rail_position_controller,
        
        # 启动运动控制节点
        spider_controller_node,
    ])
