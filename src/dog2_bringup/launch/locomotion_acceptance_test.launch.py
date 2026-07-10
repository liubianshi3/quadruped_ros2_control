#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
    cleanup_pattern = (
        "[d]og2_flat_ground|flat_ground[.]sdf|/world/[d]og2_flat_ground|"
        "[r]obot_state_publisher|[g]z_pose_to_odom|[s]im_state_estimator_node[.]py|"
        "[g]ait_scheduler_node[.]py|[m]pc_node_complete|[w]bc_node_complete|"
        "[w]bc_effort_mux|[m]pc_debug_adapter|[w]bc_debug_adapter|"
        "[r]ail_lock_commander|[d]og2_locomotion_acceptance|"
        "[v]isualization_node|[p]arameter_bridge|[i]gn gazebo|[g]z sim|"
        "[s]pawner_joint_state_broadcaster|[s]pawner_effort_controller|"
        "[s]pawner_rail_position_controller"
    )
    preclean = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' 2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' 2>/dev/null || true"
            ),
        ],
        output="screen",
    )
    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' 2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' 2>/dev/null || true"
            ),
        ],
        output="screen",
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action=preclean,
                    on_exit=[system_launch, checker],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=[
                        cleanup,
                        Shutdown(reason="Dog2 LAV1 checker finished"),
                    ],
                )
            ),
            preclean,
        ]
    )
