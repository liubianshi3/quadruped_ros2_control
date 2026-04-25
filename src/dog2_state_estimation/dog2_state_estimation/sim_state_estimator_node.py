#!/usr/bin/env python3
"""Simulation-friendly stage-1 state estimator for the Dog2 stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional

import rclpy
from dog2_interfaces.msg import ContactState, RobotState
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

LEG_ORDER = ["lf", "lh", "rh", "rf"]


@dataclass
class EstimatorSnapshot:
    """Immutable subset used by the unit tests."""

    estimator_mode: str
    source: str
    joint_count: int
    contact_state: List[bool]


def infer_contact_state(
    strategy: str,
    joint_state: Optional[JointState],
    odom: Optional[Odometry],
) -> List[bool]:
    """Return a stage-1 contact estimate."""
    # `all_true` keeps early integration simple and explicit.
    # `height_guard` degrades to all-false when the body is clearly airborne.
    if strategy == "height_guard" and odom is not None:
        if float(odom.pose.pose.position.z) > 0.45:
            return [False, False, False, False]
    if joint_state is None and odom is None:
        return [False, False, False, False]
    return [True, True, True, True]


def build_snapshot(
    estimator_mode: str,
    source: str,
    contact_state: Iterable[bool],
    joint_state: Optional[JointState],
) -> EstimatorSnapshot:
    return EstimatorSnapshot(
        estimator_mode=estimator_mode,
        source=source,
        joint_count=0 if joint_state is None else len(joint_state.name),
        contact_state=list(contact_state),
    )


class SimStateEstimatorNode(Node):
    """Unified stage-1 estimator facade."""

    def __init__(self) -> None:
        super().__init__("dog2_state_estimator")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter(
            "robot_state_topic", "/dog2/state_estimation/robot_state"
        )
        self.declare_parameter(
            "contact_state_topic", "/dog2/state_estimation/contact_state"
        )
        self.declare_parameter(
            "filtered_joint_state_topic", "/dog2/state_estimation/joint_states"
        )
        self.declare_parameter("output_odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("contact_strategy", "all_true")
        self.declare_parameter("estimator_mode", "sim_ground_truth")
        self.declare_parameter("source", "gazebo_ground_truth")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self._contact_state_topic = str(
            self.get_parameter("contact_state_topic").value
        )
        self._filtered_joint_state_topic = str(
            self.get_parameter("filtered_joint_state_topic").value
        )
        self._output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self._update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self._contact_strategy = str(self.get_parameter("contact_strategy").value)
        self._estimator_mode = str(self.get_parameter("estimator_mode").value)
        self._source = str(self.get_parameter("source").value)

        self._last_odom: Optional[Odometry] = None
        self._last_joint_state: Optional[JointState] = None
        self._last_imu: Optional[Imu] = None

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_subscription(
            JointState, self._joint_state_topic, self._on_joint_state, 50
        )
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 20)

        self._robot_state_pub = self.create_publisher(
            RobotState, self._robot_state_topic, 10
        )
        self._contact_state_pub = self.create_publisher(
            ContactState, self._contact_state_topic, 10
        )
        self._joint_state_pub = self.create_publisher(
            JointState, self._filtered_joint_state_topic, 10
        )
        self._odom_pub = self.create_publisher(Odometry, self._output_odom_topic, 10)

        self.create_timer(1.0 / self._update_rate, self._publish_state)

        self.get_logger().info(
            "dog2_state_estimator ready: odom=%s joint_states=%s robot_state=%s strategy=%s"
            % (
                self._odom_topic,
                self._joint_state_topic,
                self._robot_state_topic,
                self._contact_strategy,
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu = msg

    def _publish_state(self) -> None:
        if self._last_odom is None or self._last_joint_state is None:
            return

        contact_flags = infer_contact_state(
            self._contact_strategy, self._last_joint_state, self._last_odom
        )

        contact_msg = ContactState()
        contact_msg.header = self._last_odom.header
        contact_msg.leg_names = list(LEG_ORDER)
        contact_msg.in_contact = list(contact_flags)
        contact_msg.confidence = [0.5 for _ in LEG_ORDER]

        robot_state = RobotState()
        robot_state.header = self._last_odom.header
        robot_state.pose = self._last_odom.pose.pose
        robot_state.twist = self._last_odom.twist.twist
        robot_state.joint_state = self._last_joint_state
        robot_state.contact_state = contact_msg
        robot_state.foot_positions = [Point() for _ in LEG_ORDER]
        robot_state.estimator_mode = self._estimator_mode
        robot_state.source = self._source

        odom_out = Odometry()
        odom_out.header = self._last_odom.header
        odom_out.child_frame_id = self._last_odom.child_frame_id
        odom_out.pose = self._last_odom.pose
        odom_out.twist = self._last_odom.twist

        self._contact_state_pub.publish(contact_msg)
        self._robot_state_pub.publish(robot_state)
        self._joint_state_pub.publish(self._last_joint_state)
        self._odom_pub.publish(odom_out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
