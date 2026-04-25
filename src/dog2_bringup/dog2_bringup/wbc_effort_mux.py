from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WBCEffortMux(Node):
    _LEG_NAMES = ("lf", "lh", "rh", "rf")

    def __init__(self) -> None:
        super().__init__("wbc_effort_mux")

        self.declare_parameter("joint_effort_topic", "/dog2/wbc/joint_effort_command")
        self.declare_parameter("rail_effort_topic", "/dog2/wbc/rail_effort_command")
        self.declare_parameter("output_topic", "/effort_controller/commands")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_log_period_sec", 1.0)
        self.declare_parameter("freeze_rail_effort", False)

        self._joint_effort_topic = str(self.get_parameter("joint_effort_topic").value)
        self._rail_effort_topic = str(self.get_parameter("rail_effort_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self._debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self._debug_log_period_sec = max(0.1, float(self.get_parameter("debug_log_period_sec").value))
        self._freeze_rail_effort = bool(self.get_parameter("freeze_rail_effort").value)

        self._joint_cmd: Optional[np.ndarray] = None
        self._rail_cmd: Optional[np.ndarray] = None
        self._joint_stamp_ns: Optional[int] = None
        self._rail_stamp_ns: Optional[int] = None
        self._published_safe_zero = False
        self._last_debug_log_ns: int = 0

        self._joint_sub = self.create_subscription(
            Float64MultiArray, self._joint_effort_topic, self._on_joint_effort, 10
        )
        self._rail_sub = self.create_subscription(
            Float64MultiArray, self._rail_effort_topic, self._on_rail_effort, 10
        )
        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._timer = self.create_timer(max(1.0 / publish_rate_hz, 0.001), self._on_timer)

        self.get_logger().info(
            "wbc_effort_mux ready: joint='%s', rail='%s', output='%s', freeze_rail_effort=%s'"
            % (
                self._joint_effort_topic,
                self._rail_effort_topic,
                self._output_topic,
                self._freeze_rail_effort,
            )
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_joint_effort(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 12:
            self.get_logger().error(f"Expected 12 WBC joint efforts, got {len(msg.data)}")
            return
        self._joint_cmd = np.asarray(msg.data, dtype=float)
        self._joint_stamp_ns = self._now_ns()

    def _on_rail_effort(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().error(f"Expected 4 WBC rail efforts, got {len(msg.data)}")
            return
        self._rail_cmd = np.asarray(msg.data, dtype=float)
        self._rail_stamp_ns = self._now_ns()

    def _is_fresh(self, stamp_ns: Optional[int]) -> bool:
        if stamp_ns is None:
            return False
        age_sec = max(0.0, (self._now_ns() - stamp_ns) / 1e9)
        return age_sec <= self._command_timeout_sec

    @staticmethod
    def _compose_effort_command(joint_cmd: np.ndarray, rail_cmd: np.ndarray) -> list[float]:
        effort = np.zeros(16, dtype=float)
        for leg in range(4):
            effort[leg * 4 + 0] = rail_cmd[leg]
            effort[leg * 4 + 1 : leg * 4 + 4] = joint_cmd[leg * 3 : (leg + 1) * 3]
        return effort.tolist()

    def _publish(self, data: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = data
        self._pub.publish(msg)

    def _log_effort_breakdown(self, effort: np.ndarray) -> None:
        if not self._debug_enabled:
            return

        now_ns = self._now_ns()
        if self._last_debug_log_ns and (
            (now_ns - self._last_debug_log_ns) / 1e9 < self._debug_log_period_sec
        ):
            return

        self._last_debug_log_ns = now_ns
        parts = []
        for leg, name in enumerate(self._LEG_NAMES):
            base = leg * 4
            rail = float(effort[base + 0])
            coxa = float(effort[base + 1])
            femur = float(effort[base + 2])
            tibia = float(effort[base + 3])
            support_abs = abs(femur) + abs(tibia)
            total_abs = abs(rail) + abs(coxa) + support_abs
            parts.append(
                f"{name}[rail={rail:.1f} c={coxa:.1f} f={femur:.1f} t={tibia:.1f} "
                f"ft_abs={support_abs:.1f} sum={total_abs:.1f}]"
            )

        self.get_logger().info("effort_mux: " + " ".join(parts))

    def _on_timer(self) -> None:
        if self._joint_cmd is None or self._rail_cmd is None:
            self.get_logger().warn(
                "Waiting for both WBC joint/rail effort streams...",
                throttle_duration_sec=2.0,
            )
            return

        if self._is_fresh(self._joint_stamp_ns) and self._is_fresh(self._rail_stamp_ns):
            rail_cmd = (
                np.zeros_like(self._rail_cmd) if self._freeze_rail_effort else self._rail_cmd
            )
            effort = np.asarray(
                self._compose_effort_command(self._joint_cmd, rail_cmd), dtype=float
            )
            self._publish(effort.tolist())
            self._log_effort_breakdown(effort)
            self._published_safe_zero = False
            return

        if not self._published_safe_zero:
            self.get_logger().warn(
                "WBC effort streams are stale; publishing safe zero effort once.",
                throttle_duration_sec=2.0,
            )
            self._publish([0.0] * 16)
            self._published_safe_zero = True


def main() -> None:
    rclpy.init()
    node = WBCEffortMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
