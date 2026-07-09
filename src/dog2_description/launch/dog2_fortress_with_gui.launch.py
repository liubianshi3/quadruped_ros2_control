#!/usr/bin/env python3
"""
Dog2 Gazebo Fortress 带 GUI 的启动文件

功能：
1. 启动 Gazebo Fortress 仿真环境（带 GUI）
2. 加载 Dog2 机器人模型
3. 启动 ros2_control 控制器
4. 准备接收关节轨迹命令

使用方法：
ros2 launch dog2_description dog2_fortress_with_gui.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    
    # 设置环境变量
    gazebo_model_path = os.path.join(pkg_dog2_description, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + gazebo_model_path
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_model_path
    )
    
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' controllers_yaml:=', controllers_yaml]),
            value_type=str,
        )
    }
    
    # 世界文件路径
    world_file = '/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'
    
    # 启动 Gazebo Fortress 带 GUI
    # 移除 -r 参数，这样会显示 GUI
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file}',  # 不使用 -r，这样会显示 GUI
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
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # 加载 Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载 Joint Trajectory Controller（控制 12 个旋转关节）
    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )
    
    # 加载 Rail Position Controller（控制 4 个移动关节，锁定导轨）
    load_rail_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rail_position_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('model_variant', default_value='real', choices=['real', 'symmetric']),
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        
        # 等待机器人生成后再加载控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_joint_trajectory_controller,
                    load_rail_position_controller,
                ],
            )
        ),
    ])
