#!/usr/bin/env python3
"""Automated PASS/FAIL check for Dog2 window-frame crossing."""

from __future__ import annotations

import os
import sys
import time
from typing import Dict, Optional, Set

import rclpy
from dog2_interfaces.msg import ContactPhase, RobotState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


class CrossingCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("dog2_crossing_check")

        self.declare_parameter("timeout_sec", 150.0)
        self.declare_parameter("freshness_timeout_sec", 2.0)
        self.declare_parameter("poll_period_sec", 0.1)
        self.declare_parameter("window_x_position", 1.55)
        self.declare_parameter("body_pass_margin", 0.10)
        self.declare_parameter("rail_motion_threshold_m", 0.025)
        self.declare_parameter("result_file", "")
        self.declare_parameter("trigger_topic", "/enable_crossing")
        self.declare_parameter("crossing_state_topic", "/dog2/mpc/crossing_state")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("robot_state_topic", "/dog2/state_estimation/robot_state")
        self.declare_parameter("contact_phase_topic", "/dog2/gait/contact_phase")
        self.declare_parameter("foot_force_topic", "/dog2/mpc/foot_forces")
        self.declare_parameter("joint_effort_topic", "/dog2/wbc/joint_effort_command")
        self.declare_parameter("rail_effort_topic", "/dog2/wbc/rail_effort_command")
        self.declare_parameter("effort_command_topic", "/effort_controller/commands")

        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._freshness_timeout_sec = float(self.get_parameter("freshness_timeout_sec").value)
        self._poll_period_sec = float(self.get_parameter("poll_period_sec").value)
        self._window_x_position = float(self.get_parameter("window_x_position").value)
        self._body_pass_margin = float(self.get_parameter("body_pass_margin").value)
        self._rail_motion_threshold_m = float(
            self.get_parameter("rail_motion_threshold_m").value
        )
        self._result_file = str(self.get_parameter("result_file").value)
        self._deadline = time.monotonic() + self._timeout_sec

        self._last_odom_sec: Optional[float] = None
        self._last_robot_state_sec: Optional[float] = None
        self._last_contact_phase_sec: Optional[float] = None
        self._last_foot_force_sec: Optional[float] = None
        self._last_joint_effort_sec: Optional[float] = None
        self._last_rail_effort_sec: Optional[float] = None
        self._last_effort_command_sec: Optional[float] = None
        self._last_joint_state_sec: Optional[float] = None
        self._trigger_seen = False
        self._current_crossing_state = "UNKNOWN"
        self._crossing_states_seen: Set[str] = set()
        self._latest_x = float("-inf")
        self._max_x = float("-inf")
        self._latest_z = float("nan")
        self._min_z = float("inf")
        self._max_z = float("-inf")
        self._rail_names = (
            "lf_rail_joint",
            "lh_rail_joint",
            "rh_rail_joint",
            "rf_rail_joint",
        )
        self._initial_rail_positions: Dict[str, float] = {}
        self._max_rail_delta = 0.0
        self._last_status_log_sec = 0.0
        self.exit_code = 1
        self._done = False

        self.create_subscription(
            Bool,
            str(self.get_parameter("trigger_topic").value),
            self._on_trigger,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("crossing_state_topic").value),
            self._on_crossing_state,
            20,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )
        self.create_subscription(
            RobotState,
            str(self.get_parameter("robot_state_topic").value),
            self._on_robot_state,
            20,
        )
        self.create_subscription(
            ContactPhase,
            str(self.get_parameter("contact_phase_topic").value),
            self._on_contact_phase,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("foot_force_topic").value),
            self._on_foot_force,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("joint_effort_topic").value),
            self._on_joint_effort,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("rail_effort_topic").value),
            self._on_rail_effort,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("effort_command_topic").value),
            self._on_effort_command,
            20,
        )

        self.create_timer(self._poll_period_sec, self._poll)

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    def _age_sec(self, stamp_sec: Optional[float]) -> float:
        if stamp_sec is None:
            return float("inf")
        return max(0.0, self._now_sec() - stamp_sec)

    def _is_fresh(self, stamp_sec: Optional[float]) -> bool:
        return self._age_sec(stamp_sec) <= self._freshness_timeout_sec

    def _write_result(self, status: str, message: str) -> None:
        if not self._result_file:
            return
        try:
            directory = os.path.dirname(self._result_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self._result_file, "a", encoding="utf-8") as handle:
                handle.write(f"{status}: {message}\n")
        except Exception as exc:
            self.get_logger().warn(f"Failed to write crossing result file: {exc}")

    def _finish(self, success: bool, message: str) -> None:
        if self._done:
            return
        self._done = True
        self.exit_code = 0 if success else 1
        status = "PASS" if success else "FAIL"
        self._write_result(
            status,
            (
                f"{message}; stage={self._current_crossing_state}; "
                f"max_x={self._max_x:.3f}; min_z={self._min_z:.3f}; "
                f"max_z={self._max_z:.3f}; max_rail_delta={self._max_rail_delta:.3f}"
            ),
        )
        log = self.get_logger().info if success else self.get_logger().error
        log(
            f"{status}: {message}; stage={self._current_crossing_state}; "
            f"max_x={self._max_x:.3f}; min_z={self._min_z:.3f}; "
            f"max_z={self._max_z:.3f}; max_rail_delta={self._max_rail_delta:.3f}"
        )
        # Raising SystemExit from an rclpy timer callback can be swallowed by
        # the executor, leaving the launch test running after the result file
        # has already been written. This node is a test sentinel, so exit the
        # process directly to let launch cleanup run deterministically.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(self.exit_code)

    def _on_trigger(self, msg: Bool) -> None:
        if msg.data:
            self._trigger_seen = True

    def _on_crossing_state(self, msg: String) -> None:
        self._current_crossing_state = msg.data
        if msg.data.startswith("CROSSING:"):
            self._crossing_states_seen.add(msg.data)

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state_sec = self._now_sec()
        rail_positions = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if name in self._rail_names
        }
        if len(rail_positions) != len(self._rail_names):
            return

        if not self._initial_rail_positions:
            self._initial_rail_positions = dict(rail_positions)
            return

        for name in self._rail_names:
            delta = abs(rail_positions[name] - self._initial_rail_positions[name])
            self._max_rail_delta = max(self._max_rail_delta, delta)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_sec = self._now_sec()
        self._latest_x = float(msg.pose.pose.position.x)
        self._latest_z = float(msg.pose.pose.position.z)
        self._max_x = max(self._max_x, self._latest_x)
        self._min_z = min(self._min_z, self._latest_z)
        self._max_z = max(self._max_z, self._latest_z)

    def _on_robot_state(self, _msg: RobotState) -> None:
        self._last_robot_state_sec = self._now_sec()

    def _on_contact_phase(self, _msg: ContactPhase) -> None:
        self._last_contact_phase_sec = self._now_sec()

    def _on_foot_force(self, _msg: Float64MultiArray) -> None:
        self._last_foot_force_sec = self._now_sec()

    def _on_joint_effort(self, _msg: Float64MultiArray) -> None:
        self._last_joint_effort_sec = self._now_sec()

    def _on_rail_effort(self, _msg: Float64MultiArray) -> None:
        self._last_rail_effort_sec = self._now_sec()

    def _on_effort_command(self, _msg: Float64MultiArray) -> None:
        self._last_effort_command_sec = self._now_sec()

    def _missing_streams(self) -> list[str]:
        missing = []
        if not self._is_fresh(self._last_odom_sec):
            missing.append("/dog2/state_estimation/odom")
        if not self._is_fresh(self._last_robot_state_sec):
            missing.append("/dog2/state_estimation/robot_state")
        if not self._is_fresh(self._last_contact_phase_sec):
            missing.append("/dog2/gait/contact_phase")
        if not self._is_fresh(self._last_joint_state_sec):
            missing.append("/joint_states")
        if not self._is_fresh(self._last_foot_force_sec):
            missing.append("/dog2/mpc/foot_forces")
        if not self._is_fresh(self._last_joint_effort_sec):
            missing.append("/dog2/wbc/joint_effort_command")
        if not self._is_fresh(self._last_rail_effort_sec):
            missing.append("/dog2/wbc/rail_effort_command")
        if not self._is_fresh(self._last_effort_command_sec):
            missing.append("/effort_controller/commands")
        return missing

    def _pass_ready(self) -> tuple[bool, str]:
        if not self._trigger_seen:
            return False, "waiting for crossing trigger"
        if not self._crossing_states_seen:
            return False, "waiting for CROSSING stage"
        if self._max_rail_delta < self._rail_motion_threshold_m:
            return False, "rail motion below threshold"
        if self._max_x < (self._window_x_position + self._body_pass_margin):
            return False, "body has not cleared window x region"
        missing = self._missing_streams()
        if missing:
            return False, "stale streams: " + ", ".join(missing)
        return True, "crossing validated"

    def _poll(self) -> None:
        if self._done:
            return

        ready, message = self._pass_ready()
        if ready:
            self._finish(True, message)
            return

        now = self._now_sec()
        if now >= self._deadline:
            missing = self._missing_streams()
            suffix = f"; missing={','.join(missing)}" if missing else ""
            self._finish(False, f"{message}{suffix}")
            return

        if now - self._last_status_log_sec >= 5.0:
            self._last_status_log_sec = now
            self.get_logger().info(
                "crossing_check: trigger=%s stage=%s max_x=%.3f latest_z=%.3f min_z=%.3f max_z=%.3f max_rail_delta=%.3f"
                % (
                    self._trigger_seen,
                    self._current_crossing_state,
                    self._max_x,
                    self._latest_z,
                    self._min_z,
                    self._max_z,
                    self._max_rail_delta,
                )
            )


def main() -> None:
    rclpy.init()
    node = CrossingCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
