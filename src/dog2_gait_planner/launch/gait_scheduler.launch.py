#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("dog2_gait_planner"), "config", "gait_scheduler.yaml"]
    )
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
                        # 0.15 s swing window at cycle 0.6: 4 cm apex keeps
                        # peak foot speed reasonable while clearing ground.
                        "swing_height": 0.04,
                        # 5 mm ground-search only. 2 cm pressed the "swing"
                        # foot into the ground with the full task PD holding
                        # it at a BODY-frame target: a stiff rubber band
                        # between trunk and ground that anchored the robot
                        # (runs 57-61: fx +20 N sustained, zero net motion;
                        # run52 without overshoot walked 0.97 m).
                        "touchdown_overshoot": 0.005,
                    },
                ],
            ),
        ]
    )
