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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    smoke_config = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "config", "smoke_test.yaml"]
    )
    default_world = PathJoinSubstitution([FindPackageShare("dog2_bringup"), "worlds", "flat_ground.sdf"])
    cleanup_pattern = (
        "[d]og2_flat_ground|flat_ground[.]sdf|/world/[d]og2_flat_ground|"
        "[r]obot_state_publisher|[g]z_pose_to_odom|[s]im_state_estimator_node[.]py|"
        "[g]ait_scheduler_node[.]py|[m]pc_node_complete|[w]bc_node_complete|"
        "[w]bc_effort_mux|[m]pc_debug_adapter|[w]bc_debug_adapter|"
        "[v]isualization_node|[p]arameter_bridge|[i]gn gazebo|[g]z sim|"
        "[s]pawner_joint_state_broadcaster|[s]pawner_effort_controller"
    )
    preclean_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' "
                "2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' "
                "2>/dev/null || true"
            ),
        ],
        output="screen",
    )
    smoke_check = Node(
        package="dog2_bringup",
        executable="smoke_check",
        name="dog2_smoke_check",
        output="screen",
        parameters=[
            smoke_config,
            {
                "expect_research_stack": ParameterValue(
                    LaunchConfiguration("expect_research_stack"),
                    value_type=bool,
                )
            },
            {
                "exercise_motion": ParameterValue(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("exercise_motion"),
                            "' == 'true' or '",
                            LaunchConfiguration("expect_research_stack"),
                            "' == 'true'",
                        ]
                    ),
                    value_type=bool,
                )
            },
            {
                "result_file": LaunchConfiguration("result_file"),
            },
        ],
    )
    cleanup_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                f"pkill -TERM -f '{cleanup_pattern}' "
                "2>/dev/null || true; "
                "sleep 1; "
                f"pkill -KILL -f '{cleanup_pattern}' "
                "2>/dev/null || true"
            ),
        ],
        output="screen",
    )
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dog2_bringup"), "launch", "system.launch.py"])
        ),
        launch_arguments={
            "controller_mode": LaunchConfiguration("controller_mode"),
            "research_stack": LaunchConfiguration("research_stack"),
            "use_gui": "false",
            "rviz": "false",
            "teleop": "false",
            "world": LaunchConfiguration("world"),
            "odom_gz_topic": LaunchConfiguration("odom_gz_topic"),
            "dynamic_pose_gz_topic": LaunchConfiguration("dynamic_pose_gz_topic"),
            "gz_world_name": LaunchConfiguration("gz_world_name"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_mode", default_value="position"),
            DeclareLaunchArgument("research_stack", default_value="false"),
            DeclareLaunchArgument("expect_research_stack", default_value="false"),
            DeclareLaunchArgument("exercise_motion", default_value="false"),
            DeclareLaunchArgument("ros_domain_id", default_value="44"),
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
            DeclareLaunchArgument("result_file", default_value="/tmp/dog2_smoke_result.txt"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=preclean_gazebo,
                    on_exit=[system_launch, smoke_check],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=smoke_check,
                    on_exit=[
                        cleanup_gazebo,
                        Shutdown(reason="dog2 smoke check finished"),
                    ],
                )
            ),
            preclean_gazebo,
        ]
    )
