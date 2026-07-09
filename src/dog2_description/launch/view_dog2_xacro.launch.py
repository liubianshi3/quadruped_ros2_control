#!/usr/bin/env python3
"""
Dog2 Xacro 可视化 Launch 文件
直接从 dog2.urdf.xacro 加载并在 RViz2 中显示

使用方法:
    ros2 launch dog2_description view_dog2_xacro.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_variant = LaunchConfiguration('model_variant')
    xacro_filename = PythonExpression([
        "'dog2_symmetric.urdf.xacro' if '",
        model_variant,
        "' == 'symmetric' else 'dog2.urdf.xacro'",
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('dog2_description'),
        'urdf',
        xacro_filename,
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('dog2_description'),
        'rviz',
        'dog2.rviz',
    ])
    
    # 使用 xacro 命令处理 xacro 文件
    # Command 会在运行时执行 xacro 命令并返回结果
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # 关节状态发布 GUI 节点
    # 允许手动控制关节位置
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('model_variant', default_value='real', choices=['real', 'symmetric']),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
