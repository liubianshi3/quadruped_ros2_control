#!/usr/bin/env python3
"""Runtime smoke check for the Dog2 bringup stack."""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Set

import rclpy
from controller_manager_msgs.srv import ListControllers
from dog2_interfaces.msg import ContactPhase, MPCDebug, RobotState, WBCDebug
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


@dataclass
class _PlanarPose:
    x: float
    y: float
    z: float
    yaw: float


def _yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SmokeCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("dog2_smoke_check")

        self.declare_parameter(
            "required_topics",
            [
                "/clock",
                "/joint_states",
                "/dog2/state_estimation/robot_state",
                "/dog2/state_estimation/contact_state",
            ],
        )
        self.declare_parameter(
            "required_research_topics",
            [
                "/dog2/mpc/debug",
                "/dog2/mpc/foot_forces",
                "/dog2/wbc/debug",
                "/dog2/wbc/joint_effort_command",
                "/dog2/wbc/rail_effort_command",
                "/effort_controller/commands",
            ],
        )
        self.declare_parameter(
            "required_nodes",
            [
                "robot_state_publisher",
                "dog2_state_estimator",
            ],
        )
        self.declare_parameter(
            "required_research_nodes",
            [
                "mpc_node_complete",
                "wbc_node_complete",
                "dog2_mpc_debug_adapter",
                "dog2_wbc_debug_adapter",
                "dog2_wbc_effort_mux",
            ],
        )
        self.declare_parameter(
            "required_controllers",
            [
                "joint_state_broadcaster",
            ],
        )
        self.declare_parameter("required_research_controllers", ["effort_controller"])
        self.declare_parameter("expect_research_stack", False)
        self.declare_parameter("exercise_motion", False)
        self.declare_parameter("controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("freshness_timeout_sec", 2.0)
        self.declare_parameter("poll_period_sec", 0.1)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("robot_state_topic", "/dog2/state_estimation/robot_state")
        self.declare_parameter("contact_phase_topic", "/dog2/gait/contact_phase")
        self.declare_parameter("mpc_debug_topic", "/dog2/mpc/debug")
        self.declare_parameter("wbc_debug_topic", "/dog2/wbc/debug")
        self.declare_parameter("foot_force_topic", "/dog2/mpc/foot_forces")
        self.declare_parameter("joint_effort_topic", "/dog2/wbc/joint_effort_command")
        self.declare_parameter("rail_effort_topic", "/dog2/wbc/rail_effort_command")
        self.declare_parameter("effort_command_topic", "/effort_controller/commands")
        self.declare_parameter("stand_duration_sec", 3.0)
        self.declare_parameter("forward_duration_sec", 4.0)
        self.declare_parameter("turn_duration_sec", 4.0)
        self.declare_parameter("forward_linear_x", 0.12)
        self.declare_parameter("turn_linear_x", 0.05)
        self.declare_parameter("turn_angular_z", 0.30)
        self.declare_parameter("stand_max_xy_drift_m", 0.20)
        self.declare_parameter("enforce_stand_drift", False)
        self.declare_parameter("body_height_min_m", 0.04)
        self.declare_parameter("body_height_max_m", 0.20)
        self.declare_parameter("forward_min_distance_m", 0.10)
        self.declare_parameter("turn_min_yaw_delta_rad", 0.25)
        self.declare_parameter("result_file", "")

        self._required_topics = self._as_string_set("required_topics")
        self._required_research_topics = self._as_string_set("required_research_topics")
        self._required_nodes = self._as_string_set("required_nodes")
        self._required_research_nodes = self._as_string_set("required_research_nodes")
        self._required_controllers = self._as_string_set("required_controllers")
        self._required_research_controllers = self._as_string_set("required_research_controllers")
        self._expect_research_stack = bool(self.get_parameter("expect_research_stack").value)
        self._exercise_motion = bool(self.get_parameter("exercise_motion").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._freshness_timeout_sec = float(self.get_parameter("freshness_timeout_sec").value)
        self._poll_period_sec = max(0.05, float(self.get_parameter("poll_period_sec").value))
        self._stand_duration_sec = float(self.get_parameter("stand_duration_sec").value)
        self._forward_duration_sec = float(self.get_parameter("forward_duration_sec").value)
        self._turn_duration_sec = float(self.get_parameter("turn_duration_sec").value)
        self._forward_linear_x = float(self.get_parameter("forward_linear_x").value)
        self._turn_linear_x = float(self.get_parameter("turn_linear_x").value)
        self._turn_angular_z = float(self.get_parameter("turn_angular_z").value)
        self._stand_max_xy_drift_m = float(self.get_parameter("stand_max_xy_drift_m").value)
        self._enforce_stand_drift = bool(self.get_parameter("enforce_stand_drift").value)
        self._body_height_min_m = float(self.get_parameter("body_height_min_m").value)
        self._body_height_max_m = float(self.get_parameter("body_height_max_m").value)
        self._forward_min_distance_m = float(self.get_parameter("forward_min_distance_m").value)
        self._turn_min_yaw_delta_rad = float(self.get_parameter("turn_min_yaw_delta_rad").value)
        self._result_file = str(self.get_parameter("result_file").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self._contact_phase_topic = str(self.get_parameter("contact_phase_topic").value)
        self._mpc_debug_topic = str(self.get_parameter("mpc_debug_topic").value)
        self._wbc_debug_topic = str(self.get_parameter("wbc_debug_topic").value)
        self._foot_force_topic = str(self.get_parameter("foot_force_topic").value)
        self._joint_effort_topic = str(self.get_parameter("joint_effort_topic").value)
        self._rail_effort_topic = str(self.get_parameter("rail_effort_topic").value)
        self._effort_command_topic = str(self.get_parameter("effort_command_topic").value)

        self._deadline_sec = time.monotonic() + self._timeout_sec
        self._last_status_log_sec = 0.0
        self._stage = "wait_ready"
        self._stage_started_at: Optional[float] = None
        self._stage_start_pose: Optional[_PlanarPose] = None
        self._stage_peak_projected_distance = 0.0
        self._stage_peak_planar_distance = 0.0
        self._stage_peak_yaw_delta = 0.0

        self._last_controller_names: Set[str] = set()
        self._pending_future = None
        self._pending_future_started_at: Optional[float] = None
        self._controller_client = self.create_client(
            ListControllers,
            str(self.get_parameter("controller_manager_service").value),
        )

        self._last_odom: Optional[Odometry] = None
        self._last_odom_sec: Optional[float] = None
        self._last_robot_state_sec: Optional[float] = None
        self._last_contact_phase_sec: Optional[float] = None
        self._last_mpc_debug: Optional[MPCDebug] = None
        self._last_mpc_debug_sec: Optional[float] = None
        self._last_wbc_debug: Optional[WBCDebug] = None
        self._last_wbc_debug_sec: Optional[float] = None
        self._last_foot_force_sec: Optional[float] = None
        self._last_joint_effort_sec: Optional[float] = None
        self._last_rail_effort_sec: Optional[float] = None
        self._last_effort_command_sec: Optional[float] = None

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(RobotState, self._robot_state_topic, self._on_robot_state, 20)
        self.create_subscription(ContactPhase, self._contact_phase_topic, self._on_contact_phase, 20)
        self.create_subscription(MPCDebug, self._mpc_debug_topic, self._on_mpc_debug, 20)
        self.create_subscription(WBCDebug, self._wbc_debug_topic, self._on_wbc_debug, 20)
        self.create_subscription(
            Float64MultiArray,
            self._foot_force_topic,
            self._on_foot_force,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            self._joint_effort_topic,
            self._on_joint_effort,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            self._rail_effort_topic,
            self._on_rail_effort,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            self._effort_command_topic,
            self._on_effort_command,
            20,
        )
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        self.create_timer(self._poll_period_sec, self._poll)

    def _as_string_set(self, name: str) -> Set[str]:
        return {str(item) for item in self.get_parameter(name).value if str(item).strip()}

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    def _age_sec(self, stamp_sec: Optional[float]) -> float:
        if stamp_sec is None:
            return float("inf")
        return max(0.0, self._now_sec() - stamp_sec)

    def _is_fresh(self, stamp_sec: Optional[float]) -> bool:
        return self._age_sec(stamp_sec) <= self._freshness_timeout_sec

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_sec = self._now_sec()

    def _on_robot_state(self, _msg: RobotState) -> None:
        self._last_robot_state_sec = self._now_sec()

    def _on_contact_phase(self, _msg: ContactPhase) -> None:
        self._last_contact_phase_sec = self._now_sec()

    def _on_mpc_debug(self, msg: MPCDebug) -> None:
        self._last_mpc_debug = msg
        self._last_mpc_debug_sec = self._now_sec()

    def _on_wbc_debug(self, msg: WBCDebug) -> None:
        self._last_wbc_debug = msg
        self._last_wbc_debug_sec = self._now_sec()

    def _on_foot_force(self, _msg: Float64MultiArray) -> None:
        self._last_foot_force_sec = self._now_sec()

    def _on_joint_effort(self, _msg: Float64MultiArray) -> None:
        self._last_joint_effort_sec = self._now_sec()

    def _on_rail_effort(self, _msg: Float64MultiArray) -> None:
        self._last_rail_effort_sec = self._now_sec()

    def _on_effort_command(self, _msg: Float64MultiArray) -> None:
        self._last_effort_command_sec = self._now_sec()

    def _current_pose(self) -> Optional[_PlanarPose]:
        if self._last_odom is None:
            return None
        pose = self._last_odom.pose.pose
        return _PlanarPose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            yaw=_yaw_from_quaternion_xyzw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

    def _missing_streams(self) -> list[str]:
        missing = []
        if not self._is_fresh(self._last_odom_sec):
            missing.append(self._odom_topic)
        if not self._is_fresh(self._last_robot_state_sec):
            missing.append(self._robot_state_topic)
        if not self._is_fresh(self._last_contact_phase_sec):
            missing.append(self._contact_phase_topic)

        if self._expect_research_stack:
            if not self._is_fresh(self._last_mpc_debug_sec):
                missing.append(self._mpc_debug_topic)
            if not self._is_fresh(self._last_wbc_debug_sec):
                missing.append(self._wbc_debug_topic)
            if not self._is_fresh(self._last_foot_force_sec):
                missing.append(self._foot_force_topic)
            if not self._is_fresh(self._last_joint_effort_sec):
                missing.append(self._joint_effort_topic)
            if not self._is_fresh(self._last_rail_effort_sec):
                missing.append(self._rail_effort_topic)
            if not self._is_fresh(self._last_effort_command_sec):
                missing.append(self._effort_command_topic)
        return missing

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _write_result(self, status: str, message: str) -> None:
        if not self._result_file:
            return
        try:
            directory = os.path.dirname(self._result_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self._result_file, "a", encoding="utf-8") as handle:
                handle.write(f"{status}: {message}\n")
        except Exception as exc:  # pragma: no cover - best effort diagnostics path
            self.get_logger().warn(f"Failed to write smoke result file: {exc}")

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _stage_duration(self) -> float:
        if self._stage == "stand":
            return self._stand_duration_sec
        if self._stage == "forward":
            return self._forward_duration_sec
        if self._stage == "turn":
            return self._turn_duration_sec
        return 0.0

    def _publish_stage_command(self) -> None:
        if self._stage == "forward":
            self._publish_cmd(self._forward_linear_x, 0.0)
            return
        if self._stage == "turn":
            self._publish_cmd(self._turn_linear_x, self._turn_angular_z)
            return
        self._publish_stop()

    def _begin_stage(self, stage: str) -> None:
        pose = self._current_pose()
        if pose is None:
            self._fail("Unable to sample odom pose for motion smoke stage start.")
            return

        self._stage = stage
        self._stage_started_at = self._now_sec()
        self._stage_start_pose = pose
        self._stage_peak_projected_distance = 0.0
        self._stage_peak_planar_distance = 0.0
        self._stage_peak_yaw_delta = 0.0
        self.get_logger().info(
            "Starting smoke motion stage '%s' at x=%.3f y=%.3f z=%.3f yaw=%.3f"
            % (stage, pose.x, pose.y, pose.z, pose.yaw)
        )
        self._write_result(
            "STAGE",
            "stage=%s x=%.3f y=%.3f z=%.3f yaw=%.3f" % (stage, pose.x, pose.y, pose.z, pose.yaw),
        )
        self._publish_stage_command()

    def _body_height_ok(self, pose: _PlanarPose) -> bool:
        return self._body_height_min_m <= pose.z <= self._body_height_max_m

    def _require_wbc_success(self) -> None:
        if self._expect_research_stack and self._last_wbc_debug is not None and not self._last_wbc_debug.success:
            self._fail(f"WBC reported failure: {self._last_wbc_debug.status_message}")

    def _complete_stand(self) -> None:
        if self._stage_start_pose is None:
            self._fail("Stand stage missing start pose.")
            return

        pose = self._current_pose()
        if pose is None:
            self._fail("Stand stage missing current odom pose.")
            return

        drift = math.hypot(pose.x - self._stage_start_pose.x, pose.y - self._stage_start_pose.y)
        if self._enforce_stand_drift and drift > self._stand_max_xy_drift_m:
            self._fail(
                "Stand stage drifted %.3f m (limit %.3f m)." % (drift, self._stand_max_xy_drift_m)
            )
            return
        if not self._body_height_ok(pose):
            self._fail(
                "Stand stage body height %.3f m outside [%.3f, %.3f] m."
                % (pose.z, self._body_height_min_m, self._body_height_max_m)
            )
            return

        self._require_wbc_success()
        if drift > self._stand_max_xy_drift_m:
            self.get_logger().warn(
                "Stand stage drift %.3f m exceeds diagnostic threshold %.3f m, "
                "but continuing because enforce_stand_drift=false."
                % (drift, self._stand_max_xy_drift_m)
            )
        self.get_logger().info("Stand stage passed: drift=%.3f m z=%.3f m" % (drift, pose.z))
        self._begin_stage("forward")

    def _complete_forward(self) -> None:
        if self._stage_start_pose is None:
            self._fail("Forward stage missing start pose.")
            return

        pose = self._current_pose()
        if pose is None:
            self._fail("Forward stage missing current odom pose.")
            return

        dx = pose.x - self._stage_start_pose.x
        dy = pose.y - self._stage_start_pose.y
        projected_distance = dx * math.cos(self._stage_start_pose.yaw) + dy * math.sin(
            self._stage_start_pose.yaw
        )
        projected_distance = max(projected_distance, self._stage_peak_projected_distance)
        planar_distance = max(math.hypot(dx, dy), self._stage_peak_planar_distance)
        if planar_distance < self._forward_min_distance_m:
            self._fail(
                "Forward stage only moved %.3f m in plane (need %.3f m). "
                "projected_distance=%.3f m."
                % (planar_distance, self._forward_min_distance_m, projected_distance)
            )
            return
        if not self._body_height_ok(pose):
            self._fail(
                "Forward stage body height %.3f m outside [%.3f, %.3f] m."
                % (pose.z, self._body_height_min_m, self._body_height_max_m)
            )
            return

        self._require_wbc_success()
        self.get_logger().info(
            "Forward stage passed: planar_distance=%.3f m projected_distance=%.3f m z=%.3f m"
            % (planar_distance, projected_distance, pose.z)
        )
        self._begin_stage("turn")

    def _complete_turn(self) -> None:
        if self._stage_start_pose is None:
            self._fail("Turn stage missing start pose.")
            return

        pose = self._current_pose()
        if pose is None:
            self._fail("Turn stage missing current odom pose.")
            return

        yaw_delta = abs(_wrap_angle(pose.yaw - self._stage_start_pose.yaw))
        yaw_delta = max(yaw_delta, self._stage_peak_yaw_delta)
        if yaw_delta < self._turn_min_yaw_delta_rad:
            self._fail(
                "Turn stage yaw delta %.3f rad below threshold %.3f rad."
                % (yaw_delta, self._turn_min_yaw_delta_rad)
            )
            return
        if not self._body_height_ok(pose):
            self._fail(
                "Turn stage body height %.3f m outside [%.3f, %.3f] m."
                % (pose.z, self._body_height_min_m, self._body_height_max_m)
            )
            return

        self._require_wbc_success()
        self._publish_stop()
        self.get_logger().info("Turn stage passed: yaw_delta=%.3f rad z=%.3f m" % (yaw_delta, pose.z))
        self.get_logger().info("Dog2 smoke check passed with stand -> forward -> turning motion.")
        self._write_result(
            "PASS",
            "turn_yaw_delta=%.3f z=%.3f" % (yaw_delta, pose.z),
        )
        raise SystemExit(0)

    def _fail(self, reason: str) -> None:
        self._publish_stop()
        self.get_logger().error(reason)
        self._write_result("FAIL", reason)
        raise SystemExit(1)

    def _log_waiting(
        self,
        missing_topics: list[str],
        missing_nodes: list[str],
        missing_controllers: list[str],
        missing_streams: list[str],
    ) -> None:
        now_sec = self._now_sec()
        if now_sec - self._last_status_log_sec < 2.0:
            return
        self._last_status_log_sec = now_sec
        self.get_logger().info(
            "Waiting for stack readiness. missing_topics=%s missing_nodes=%s "
            "missing_controllers=%s missing_streams=%s"
            % (missing_topics, missing_nodes, missing_controllers, missing_streams)
        )

    def _poll(self) -> None:
        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        node_names = set(self.get_node_names())
        required_topics = set(self._required_topics)
        required_nodes = set(self._required_nodes)
        required_controllers = set(self._required_controllers)

        if self._expect_research_stack:
            required_topics |= self._required_research_topics
            required_nodes |= self._required_research_nodes
            required_controllers |= self._required_research_controllers

        if self._pending_future is None and self._controller_client.wait_for_service(timeout_sec=0.05):
            self._pending_future = self._controller_client.call_async(ListControllers.Request())
            self._pending_future_started_at = self._now_sec()

        if self._pending_future is not None and self._pending_future.done():
            try:
                response = self._pending_future.result()
                self._last_controller_names = {
                    controller.name
                    for controller in response.controller
                    if controller.state == "active"
                }
            except Exception as exc:  # pragma: no cover - best effort runtime path
                self.get_logger().warn(f"Controller query failed: {exc}")
            self._pending_future = None
            self._pending_future_started_at = None
        elif self._pending_future is not None and self._pending_future_started_at is not None:
            if (self._now_sec() - self._pending_future_started_at) > 2.0:
                self.get_logger().warn(
                    "Controller query timed out; retrying.",
                    throttle_duration_sec=2.0,
                )
                self._pending_future = None
                self._pending_future_started_at = None

        missing_topics = sorted(required_topics - topic_names)
        missing_nodes = sorted(required_nodes - node_names)
        missing_controllers = sorted(required_controllers - self._last_controller_names)
        missing_streams = sorted(self._missing_streams())

        if missing_topics or missing_nodes or missing_controllers or missing_streams:
            if self._exercise_motion and self._stage != "wait_ready":
                self._fail(
                    "Smoke motion lost runtime dependencies. missing_topics=%s missing_nodes=%s "
                    "missing_controllers=%s missing_streams=%s"
                    % (missing_topics, missing_nodes, missing_controllers, missing_streams)
                )
                return

            if self._now_sec() >= self._deadline_sec:
                self._fail(
                    "Smoke check failed. missing_topics=%s missing_nodes=%s "
                    "missing_controllers=%s missing_streams=%s"
                    % (missing_topics, missing_nodes, missing_controllers, missing_streams)
                )
                return

            self._log_waiting(missing_topics, missing_nodes, missing_controllers, missing_streams)
            return

        if not self._exercise_motion:
            self.get_logger().info("Dog2 smoke check passed.")
            raise SystemExit(0)

        if self._stage == "wait_ready":
            self._begin_stage("stand")
            return

        self._publish_stage_command()
        if self._stage_started_at is None:
            self._fail("Smoke motion stage started without a start timestamp.")
            return

        pose = self._current_pose()
        if pose is not None and self._stage_start_pose is not None:
            if self._stage == "forward":
                dx = pose.x - self._stage_start_pose.x
                dy = pose.y - self._stage_start_pose.y
                projected_distance = dx * math.cos(self._stage_start_pose.yaw) + dy * math.sin(
                    self._stage_start_pose.yaw
                )
                self._stage_peak_projected_distance = max(
                    self._stage_peak_projected_distance,
                    projected_distance,
                )
                self._stage_peak_planar_distance = max(
                    self._stage_peak_planar_distance,
                    math.hypot(dx, dy),
                )
            elif self._stage == "turn":
                yaw_delta = abs(_wrap_angle(pose.yaw - self._stage_start_pose.yaw))
                self._stage_peak_yaw_delta = max(self._stage_peak_yaw_delta, yaw_delta)

        elapsed_sec = self._now_sec() - self._stage_started_at
        if elapsed_sec < self._stage_duration():
            return

        if self._stage == "stand":
            self._complete_stand()
            return
        if self._stage == "forward":
            self._complete_forward()
            return
        if self._stage == "turn":
            self._complete_turn()
            return


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SmokeCheckNode()
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = int(exc.code)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
