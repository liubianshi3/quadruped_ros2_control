#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("dog2_gait_planner"), "config", "gait_scheduler.yaml"]
    )
    model_variant = LaunchConfiguration("model_variant")
    xacro_filename = PythonExpression(
        [
            "'dog2_symmetric.urdf.xacro' if '",
            model_variant,
            "' == 'symmetric' else 'dog2.urdf.xacro'",
        ]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("dog2_description"), "urdf", xacro_filename]
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    return LaunchDescription(
        [
            # Named uniquely on purpose: launch configurations are GLOBAL
            # across the include tree, and control_stack.launch.py includes
            # state_estimation with config_file:=estimator.yaml first. A
            # generic "config_file" argument here inherited that value and
            # silently loaded the estimator yaml (no dog2_gait_scheduler
            # section), so the scheduler ran on hardcoded defaults.
            DeclareLaunchArgument("gait_scheduler_config", default_value=config),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "model_variant",
                default_value="symmetric",
                choices=["real", "symmetric"],
            ),
            Node(
                package="dog2_gait_planner",
                executable="gait_scheduler_node.py",
                name="dog2_gait_scheduler",
                output="screen",
                parameters=[
                    LaunchConfiguration("gait_scheduler_config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
            Node(
                package="dog2_gait_planner",
                executable="swing_target_node.py",
                name="dog2_swing_target",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        # Must equal 1 - duty_factor of the scheduler: the
                        # bezier swing timing assumes the swing window is
                        # this fraction of the cycle.
                        "swing_fraction": 0.25,
                        # Keep the flat-ground lift inside the measured RH
                        # workspace. A 5 cm apex is unreachable from the
                        # fourth-step liftoff pose and folds the femur toward
                        # its upper limit instead of lowering to touchdown.
                        # 2.5 cm remains clear of flat ground and keeps the
                        # complete lift-transfer-lower path reachable.
                        "swing_height": 0.025,
                        # 5 mm ground-search only. 2 cm pressed the "swing"
                        # foot into the ground with the full task PD holding
                        # it at a BODY-frame target: a stiff rubber band
                        # between trunk and ground that anchored the robot
                        # (runs 57-61: fx +20 N sustained, zero net motion;
                        # run52 without overshoot walked 0.97 m).
                        "touchdown_overshoot": 0.005,
                        # Keep crawl touchdowns at the COM-centred nominal.
                        # A full three-slot lead moved both front feet 6 cm
                        # ahead before either hind leg could unload, leaving
                        # the COM on the LF-RH support edge (about 2 N margin).
                        "crawl_velocity_lead_sec": 0.0,
                        # Latch every swing from measured FK rather than the
                        # previous nominal command. This prevents a target
                        # discontinuity from pinning the foot to the ground.
                        "robot_description": robot_description,
                    },
                ],
            ),
        ]
    )
