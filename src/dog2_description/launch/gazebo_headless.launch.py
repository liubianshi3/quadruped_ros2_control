#!/usr/bin/env python3
"""
Gazebo Headless Launch - 只启动物理引擎，不启动图形界面
用于测试物理仿真
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dog2_description = get_package_share_directory('dog2_description')
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('dog2_description'),
        'config',
        'ros2_controllers.yaml',
    ])
    
    # 设置环境变量
    gazebo_model_path = os.path.join(pkg_dog2_description, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + gazebo_model_path
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_model_path
    )
    
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
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' controllers_yaml:=', controllers_yaml]),
            value_type=str,
        )
    }
    
    # 世界文件路径（使用空世界）
    world_file = '/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'
    
    # 只启动 gzserver（无图形界面）
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Spawn 机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dog2',
            '-z', '1.0'  # Increased height to prevent initial penetration
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('model_variant', default_value='real', choices=['real', 'symmetric']),
        set_gazebo_model_path,
        gzserver,
        robot_state_publisher,
        spawn_entity,
    ])
