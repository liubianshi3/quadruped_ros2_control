#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    acceptance_config = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "config", "locomotion_acceptance.yaml"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "worlds", "flat_ground.sdf"]
    )
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dog2_bringup"), "launch", "system.launch.py"]
            )
        ),
        launch_arguments={
            "controller_mode": "effort",
            "model_variant": LaunchConfiguration("model_variant"),
            "research_stack": "true",
            "use_gui": LaunchConfiguration("use_gui"),
            "rviz": "false",
            "teleop": "false",
            "world": LaunchConfiguration("world"),
            "odom_gz_topic": LaunchConfiguration("odom_gz_topic"),
            "dynamic_pose_gz_topic": LaunchConfiguration("dynamic_pose_gz_topic"),
            "gz_world_name": LaunchConfiguration("gz_world_name"),
            "bridge_foot_contact": "true",
            "enable_acceptance_contact_sensors": "true",
        }.items(),
    )
    checker = Node(
        package="dog2_bringup",
        executable="locomotion_acceptance",
        name="dog2_locomotion_acceptance",
        output="screen",
        parameters=[
            acceptance_config,
            {
                "trial_id": LaunchConfiguration("trial_id"),
                "run_uuid": LaunchConfiguration("run_uuid"),
                "model_variant": LaunchConfiguration("model_variant"),
                "result_json": LaunchConfiguration("result_json"),
                "samples_csv": LaunchConfiguration("samples_csv"),
                "junit_xml": LaunchConfiguration("junit_xml"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_variant", default_value="symmetric", choices=["real", "symmetric"]
            ),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("ros_domain_id", default_value="64"),
            DeclareLaunchArgument("trial_id", default_value="trial_001"),
            DeclareLaunchArgument("run_uuid", default_value="manual"),
            DeclareLaunchArgument(
                "transport_partition", default_value="dog2_lav1_manual"
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument(
                "odom_gz_topic",
                default_value="/world/dog2_flat_ground/model/dog2/odometry",
            ),
            DeclareLaunchArgument(
                "dynamic_pose_gz_topic",
                default_value="/world/dog2_flat_ground/dynamic_pose/info",
            ),
            DeclareLaunchArgument("gz_world_name", default_value="dog2_flat_ground"),
            DeclareLaunchArgument(
                "result_json",
                default_value="/tmp/dog2_locomotion_acceptance/trial_001.json",
            ),
            DeclareLaunchArgument(
                "samples_csv",
                default_value="/tmp/dog2_locomotion_acceptance/trial_001_samples.csv",
            ),
            DeclareLaunchArgument(
                "junit_xml",
                default_value="/tmp/dog2_locomotion_acceptance/trial_001.junit.xml",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            SetEnvironmentVariable(
                "GZ_PARTITION", LaunchConfiguration("transport_partition")
            ),
            SetEnvironmentVariable(
                "IGN_PARTITION", LaunchConfiguration("transport_partition")
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=[Shutdown(reason="Dog2 LAV1 checker finished")],
                )
            ),
            system_launch,
            checker,
        ]
    )
