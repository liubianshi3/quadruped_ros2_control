#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from dog2_motion_control.model_variant import get_urdf_xacro_filename


def generate_launch_description() -> LaunchDescription:
    estimator_config = PathJoinSubstitution(
        [FindPackageShare("dog2_state_estimation"), "config", "estimator.yaml"]
    )
    research_mpc_config = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "config", "research_mpc.yaml"]
    )
    control_stack_use_sim_time = PythonExpression(
        [
            "'false' if '",
            LaunchConfiguration("research_stack"),
            "' == 'true' else '",
            LaunchConfiguration("use_sim_time"),
            "'",
        ]
    )

    def _xacro_file(context):
        variant = LaunchConfiguration("model_variant").perform(context).strip() or "real"
        from dog2_motion_control.model_variant import normalize_model_variant
        variant = normalize_model_variant(variant)
        return PathJoinSubstitution(
            [FindPackageShare("dog2_description"), "urdf", get_urdf_xacro_filename(variant)]
        ).perform(context)

    robot_description = ParameterValue(Command(["xacro ", _xacro_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_variant", default_value="real"),
            DeclareLaunchArgument("research_stack", default_value="true"),
            DeclareLaunchArgument("controller_mode", default_value="position"),
            DeclareLaunchArgument("estimator_config", default_value=estimator_config),
            DeclareLaunchArgument("research_mpc_config", default_value=research_mpc_config),
            DeclareLaunchArgument("crossing_window_x_position", default_value="1.55"),
            DeclareLaunchArgument("crossing_window_width", default_value="0.48"),
            DeclareLaunchArgument("crossing_window_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_bottom_height", default_value="0.0"),
            DeclareLaunchArgument("crossing_window_top_height", default_value="0.62"),
            DeclareLaunchArgument("crossing_window_safety_margin", default_value="0.04"),
            DeclareLaunchArgument("crossing_activation_distance", default_value="0.25"),
            DeclareLaunchArgument("crossing_approach_speed", default_value="0.05"),
            DeclareLaunchArgument("crossing_force_full_support", default_value="true"),
            DeclareLaunchArgument("crossing_freeze_rail_targets", default_value="false"),
            DeclareLaunchArgument("rail_hold_enabled", default_value="false"),
            DeclareLaunchArgument("rail_hold_hover_enabled", default_value="false"),
            DeclareLaunchArgument("rail_hold_crossing_staging_enabled", default_value="false"),
            DeclareLaunchArgument("freeze_rail_effort_in_hover", default_value="false"),
            DeclareLaunchArgument("freeze_rail_effort_in_crossing_staging", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("dog2_state_estimation"), "launch", "state_estimation.launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("estimator_config"),
                    # The current research stack is driven by wall-timer loops.
                    # Keep it off sim time so a stalled /clock bridge does not
                    # freeze state_estimation -> mpc -> wbc in Gazebo.
                    "use_sim_time": control_stack_use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("dog2_gait_planner"), "launch", "gait_scheduler.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": control_stack_use_sim_time}.items(),
            ),
            Node(
                package="dog2_mpc",
                executable="mpc_node_complete",
                name="mpc_node_complete",
                output="screen",
                parameters=[
                    LaunchConfiguration("research_mpc_config"),
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        ),
                        "robot_description": robot_description
                    },
                    {
                        "crossing_window_x_position": ParameterValue(
                            LaunchConfiguration("crossing_window_x_position"),
                            value_type=float,
                        ),
                        "crossing_window_width": ParameterValue(
                            LaunchConfiguration("crossing_window_width"),
                            value_type=float,
                        ),
                        "crossing_window_height": ParameterValue(
                            LaunchConfiguration("crossing_window_height"),
                            value_type=float,
                        ),
                        "crossing_window_bottom_height": ParameterValue(
                            LaunchConfiguration("crossing_window_bottom_height"),
                            value_type=float,
                        ),
                        "crossing_window_top_height": ParameterValue(
                            LaunchConfiguration("crossing_window_top_height"),
                            value_type=float,
                        ),
                        "crossing_window_safety_margin": ParameterValue(
                            LaunchConfiguration("crossing_window_safety_margin"),
                            value_type=float,
                        ),
                        "crossing_activation_distance": ParameterValue(
                            LaunchConfiguration("crossing_activation_distance"),
                            value_type=float,
                        ),
                        "crossing_approach_speed": ParameterValue(
                            LaunchConfiguration("crossing_approach_speed"),
                            value_type=float,
                        ),
                        "crossing_force_full_support": ParameterValue(
                            LaunchConfiguration("crossing_force_full_support"),
                            value_type=bool,
                        ),
                        "crossing_freeze_rail_targets": ParameterValue(
                            LaunchConfiguration("crossing_freeze_rail_targets"),
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[("/dog2/odom", "/dog2/state_estimation/odom")],
                condition=IfCondition(LaunchConfiguration("research_stack")),
            ),
            Node(
                package="dog2_wbc",
                executable="wbc_node_complete",
                name="wbc_node_complete",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        ),
                        "freeze_rail_effort": ParameterValue(
                            LaunchConfiguration("crossing_freeze_rail_targets"),
                            value_type=bool,
                        ),
                        "rail_hold_enabled": ParameterValue(
                            LaunchConfiguration("rail_hold_enabled"),
                            value_type=bool,
                        ),
                        "rail_hold_hover_enabled": ParameterValue(
                            LaunchConfiguration("rail_hold_hover_enabled"),
                            value_type=bool,
                        ),
                        "rail_hold_crossing_staging_enabled": ParameterValue(
                            LaunchConfiguration("rail_hold_crossing_staging_enabled"),
                            value_type=bool,
                        ),
                    }
                ],
                remappings=[
                    ("/joint_group_effort_controller/commands", "/dog2/wbc/joint_effort_command"),
                    ("/sliding_joint_effort_controller/commands", "/dog2/wbc/rail_effort_command"),
                ],
                condition=IfCondition(LaunchConfiguration("research_stack")),
            ),
            Node(
                package="dog2_bringup",
                executable="mpc_debug_adapter",
                name="dog2_mpc_debug_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        )
                    }
                ],
            ),
            Node(
                package="dog2_bringup",
                executable="wbc_debug_adapter",
                name="dog2_wbc_debug_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        )
                    }
                ],
            ),
            Node(
                package="dog2_bringup",
                executable="wbc_effort_mux",
                name="dog2_wbc_effort_mux",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        ),
                        "freeze_rail_effort": ParameterValue(
                            LaunchConfiguration("crossing_freeze_rail_targets"),
                            value_type=bool,
                        ),
                        "freeze_rail_effort_in_hover": ParameterValue(
                            LaunchConfiguration("freeze_rail_effort_in_hover"),
                            value_type=bool,
                        ),
                        "freeze_rail_effort_in_crossing_staging": ParameterValue(
                            LaunchConfiguration("freeze_rail_effort_in_crossing_staging"),
                            value_type=bool,
                        ),
                        "publish_safe_zero_on_stale": False,
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("research_stack"),
                            "' == 'true' and '",
                            LaunchConfiguration("controller_mode"),
                            "' == 'effort'",
                        ]
                    )
                ),
            ),
        ]
    )
