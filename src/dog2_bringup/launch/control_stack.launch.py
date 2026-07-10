#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    estimator_config = PathJoinSubstitution(
        [FindPackageShare("dog2_state_estimation"), "config", "estimator.yaml"]
    )
    research_mpc_config = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "config", "research_mpc.yaml"]
    )
    flat_locomotion_config = PathJoinSubstitution(
        [FindPackageShare("dog2_bringup"), "config", "flat_locomotion.yaml"]
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

    model_variant = LaunchConfiguration("model_variant")
    xacro_filename = PythonExpression([
        "'dog2_symmetric.urdf.xacro' if '",
        model_variant,
        "' == 'symmetric' else 'dog2.urdf.xacro'",
    ])
    xacro_file = PathJoinSubstitution([FindPackageShare("dog2_description"), "urdf", xacro_filename])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    flat_locomotion_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("research_stack"),
                "' == 'true' and '",
                LaunchConfiguration("flat_locomotion"),
                "' == 'true'",
            ]
        )
    )
    legacy_research_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("research_stack"),
                "' == 'true' and '",
                LaunchConfiguration("flat_locomotion"),
                "' != 'true'",
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_variant", default_value="symmetric", choices=["real", "symmetric"]),
            DeclareLaunchArgument("research_stack", default_value="true"),
            DeclareLaunchArgument("flat_locomotion", default_value="true"),
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
            # -1.0: MPC publishes fz as the UPWARD ground reaction on the
            # foot; static equilibrium is tau = g(q) - J^T f_ext (verified
            # against pinocchio rnea with external forces). +1.0 made every
            # stance leg ANTI-supporting -- the WBC folded the knees harder
            # the more support the MPC asked for (tibia slam to -2.8 rad at
            # handoff, h bouncing 0.05-0.17 in runs 20-27).
            DeclareLaunchArgument("wbc_foot_force_sign", default_value="-1.0"),
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
                launch_arguments={
                    "use_sim_time": control_stack_use_sim_time,
                    "model_variant": model_variant,
                }.items(),
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
                condition=legacy_research_condition,
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
                        "foot_force_sign": ParameterValue(
                            LaunchConfiguration("wbc_foot_force_sign"),
                            value_type=float,
                        ),
                        # MPC forces are world-frame; rotate them into the
                        # (possibly tilted) base frame before J^T, and use
                        # the tilted gravity in g(q). Without this, a
                        # pitched trunk converts vertical support into a
                        # horizontal shove (sin(tilt) leakage; run15 flip).
                        "rotate_forces_to_base": True,
                        "odom_topic": "/dog2/state_estimation/odom",
                        "odom_timeout_sec": 0.5,
                        # Pure J^T*f stance has zero joint-space stiffness:
                        # quiet standing collapses without a posture PD. The
                        # OFF: with four feet planted and the rails locked,
                        # the leg joints are FULLY determined by the body
                        # pose (16 DOF - 12 contact constraints - 4 rail
                        # locks = 0 free DOF). A joint-space posture PD is
                        # therefore a second, redundant position regulator
                        # for the same DOF the MPC height loop already
                        # owns; unless the two geometries agree exactly it
                        # produces internal forces and foot scrub. run30:
                        # the hold was perfectly quiet at h=0.184, and the
                        # posture pull (tibia target -1.40 vs measured
                        # -1.62, ~+4.4 Nm/leg) kicked the trunk up 5 cm at
                        # handoff and seeded the hop cycle. Never re-enable
                        # without null-space projection.
                        "posture_pd_enabled": False,
                        # Kept for reference / A-B experiments only. Target
                        # matches the MPC height loop pose (femur 1.05 /
                        # tibia -1.40 <-> body 0.205 m).
                        "posture_target_coxa": 0.0,
                        "posture_target_femur": 1.05,
                        "posture_target_tibia": -1.40,
                        "posture_kp_coxa": 15.0,
                        "posture_kp_femur": 20.0,
                        "posture_kp_tibia": 20.0,
                        "posture_kd_coxa": 3.0,
                        "posture_kd_femur": 5.0,
                        "posture_kd_tibia": 5.0,
                        "posture_max_torque": 18.0,
                        "robot_description": robot_description,
                    }
                ],
                remappings=[
                    ("/joint_group_effort_controller/commands", "/dog2/wbc/joint_effort_command"),
                    ("/sliding_joint_effort_controller/commands", "/dog2/wbc/rail_effort_command"),
                ],
                condition=legacy_research_condition,
            ),
            Node(
                package="dog2_mpc",
                executable="flat_mpc_node",
                name="flat_mpc_node",
                output="screen",
                parameters=[
                    flat_locomotion_config,
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        ),
                        "robot_description": robot_description,
                    },
                ],
                condition=flat_locomotion_condition,
            ),
            Node(
                package="dog2_wbc",
                executable="flat_wbc_node",
                name="flat_wbc_node",
                output="screen",
                parameters=[
                    flat_locomotion_config,
                    {
                        "use_sim_time": ParameterValue(
                            control_stack_use_sim_time,
                            value_type=bool,
                        ),
                        "robot_description": robot_description,
                    },
                ],
                condition=flat_locomotion_condition,
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
                        # Rails are locked by rail_position_controller; the
                        # effort_controller only takes the 12 rotational joints.
                        "include_rail_in_output": False,
                        "publish_safe_zero_on_stale": False,
                        "startup_hold_enabled": True,
                        # Stand-up PD window: the robot spends several limp
                        # seconds collapsing while the controller chain comes
                        # up, so give the PD time to fold back to the stance
                        # (3 s quasi-static pose ramp) plus a settle margin
                        # before blending into WBC output.
                        "startup_hold_sec": 6.0,
                        # 5 s quasi-static authority blend: at handoff the
                        # WBC posture PD carries a 0.45 rad tibia error
                        # (crouch -1.55 vs stance target -1.10, ~27 N*m
                        # step) plus the MPC height loop step (0.166 ->
                        # 0.26 m). Blending over 2 s injected both steps
                        # nearly at once and rang the stand-up loop into a
                        # growing 1 Hz wobble (run20).
                        "startup_ramp_sec": 5.0,
                        # Per-leg static feed-forward, deliberately MODEST
                        # (stance-pose values, not crouch-pose): a point-
                        # exact crouch FF has hind tibia at -13.3 N*m
                        # (76% hind load there), which acts as a folding
                        # bias at every intermediate pose and dragged the
                        # rise to a halt 0.22 rad below target (runs 19-24
                        # never actually reached the crouch, h stuck at
                        # 0.08). The PD owns tracking; FF only trims.
                        # Order [lf, lh, rh, rf] x [coxa, femur, tibia].
                        "startup_hold_joint_effort": [
                            0.0, 2.109, -6.469,
                            0.0, 1.683, -4.201,
                            0.0, 1.683, -4.201,
                            0.0, 2.109, -6.469,
                        ],
                        "startup_standup_pd_enabled": True,
                        # Stand-up target is a CROUCH, not the full stance:
                        # runs 16-18 showed the joint-space PD rise stays
                        # level only up to tibia ~-1.5 and then pitch-runs
                        # away into a stable 0.64 rad "sit" (attitude-blind
                        # PD cannot recover it). The PD now stops in the
                        # level regime; the attitude-aware WBC+MPC stack
                        # (force rotation + HOVER attitude support + height
                        # loop to 0.26 m) completes the rise after handoff.
                        "startup_standup_pose": [0.0, 1.05, -1.55],
                        "startup_standup_pose_ramp_sec": 4.0,
                        "startup_standup_kp": [25.0, 60.0, 60.0],
                        "startup_standup_kd": [1.0, 2.0, 2.0],
                        "startup_standup_max_torque": 33.0,
                        # Start the hold timer only when the effort
                        # controller itself subscribes; smoke_check also
                        # listens on the output topic and must not trigger
                        # the timer early (run15: 2.6 s of the pose ramp
                        # burned before the torque path existed).
                        "output_controller_node": "effort_controller",
                        # Gate the hold->WBC ramp on actual quiescence.
                        # run15 flipped at the fixed-timer handoff with the
                        # trunk still pitched ~0.4 rad; the HOVER stack has
                        # no attitude authority to absorb that.
                        "startup_handoff_require_settle": True,
                        "startup_handoff_odom_topic": "/dog2/state_estimation/odom",
                        # 0.26: the PD hold rests at 0.20-0.24 rad pitch
                        # depending on FF variant (runs 19-25); with force
                        # rotation + consistent stage-1 geometry the force
                        # stack demonstrably levels the trunk after handoff
                        # (run25: tilt 0.238 -> 0.075 in ~5 s), so hand off
                        # early rather than burn the settle window waiting.
                        "startup_handoff_tilt_max_rad": 0.26,
                        "startup_handoff_joint_speed_max_rad_s": 0.6,
                        "startup_handoff_settle_duration_sec": 1.0,
                        "startup_handoff_max_extra_sec": 30.0,
                        # Trunk leveling during stand-up (differential
                        # femur/tibia targets from measured pitch). A/B on
                        # run16/run17 showed it moves the joints but not the
                        # trunk once the body has beached at ~0.6 rad pitch,
                        # so it stays off; the level stand needs the
                        # attitude-aware WBC/MPC path instead.
                        "startup_standup_level_enabled": False,
                        "startup_standup_level_kp": 0.5,
                        "startup_standup_level_max_delta_rad": 0.30,
                        # Fade the static feed-forward in with the pose
                        # ramp: full FF on a collapsed heap injects the
                        # asymmetric folding torque that seeded the pitch
                        # runaway (run16/17 beaching).
                        "startup_hold_effort_ramp_enabled": True,
                        # Online quasi-static FF: J^T f + g recomputed at the
                        # measured pose each cycle. Constant FF (either
                        # variant) is exact at one pose only and left the
                        # rise stalled at h~0.09 / tibia -1.87 (runs 19-27:
                        # designed crouch h=0.166 / tibia -1.55 never
                        # reached), handing the WBC an un-modeled
                        # shin-contact state.
                        "startup_online_ff_enabled": True,
                        "robot_description": robot_description,
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
