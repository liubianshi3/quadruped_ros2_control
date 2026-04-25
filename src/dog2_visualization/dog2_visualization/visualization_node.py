#!/usr/bin/env python3
"""Unified visualization node for the Dog2 stack."""

import rclpy
from dog2_interfaces.msg import ContactState, GRFReference, MPCHorizon, RobotState, WBCDebug
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dog2_visualization.foot_force_visualizer import FootForceVisualizer


class VisualizationNode(Node):
    """Main visualization node for Dog2 visualization system."""

    def __init__(self):
        super().__init__("visualization_node")

        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("robot_state_topic", "/dog2/state_estimation/robot_state")
        self.declare_parameter("contact_state_topic", "/dog2/state_estimation/contact_state")
        self.declare_parameter("grf_topic", "/dog2/mpc/grf_reference")
        self.declare_parameter("wbc_debug_topic", "/dog2/wbc/debug")
        self.declare_parameter("horizon_topic", "/dog2/mpc/horizon")
        self.declare_parameter("legacy_foot_force_topic", "/dog2/mpc/foot_forces")

        self.update_rate = float(self.get_parameter("update_rate").value)

        self.current_robot_state = None
        self.current_contact_state = None
        self.current_grf_reference = None
        self.current_wbc_debug = None
        self.current_horizon = None
        self.current_foot_forces = None
        self.base_history = []
        self.max_history_points = 200

        self.foot_force_viz = FootForceVisualizer(frame_id="base_link")

        self._init_subscribers()
        self._init_publishers()

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Dog2 Visualization Node initialized at {self.update_rate} Hz")

    def _init_subscribers(self):
        self.robot_state_sub = self.create_subscription(
            RobotState,
            str(self.get_parameter("robot_state_topic").value),
            self.robot_state_callback,
            20,
        )
        self.contact_state_sub = self.create_subscription(
            ContactState,
            str(self.get_parameter("contact_state_topic").value),
            self.contact_state_callback,
            20,
        )
        self.grf_reference_sub = self.create_subscription(
            GRFReference,
            str(self.get_parameter("grf_topic").value),
            self.grf_reference_callback,
            20,
        )
        self.wbc_debug_sub = self.create_subscription(
            WBCDebug,
            str(self.get_parameter("wbc_debug_topic").value),
            self.wbc_debug_callback,
            20,
        )
        self.horizon_sub = self.create_subscription(
            MPCHorizon,
            str(self.get_parameter("horizon_topic").value),
            self.horizon_callback,
            20,
        )
        self.foot_forces_sub = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("legacy_foot_force_topic").value),
            self.foot_forces_callback,
            20,
        )

    def _init_publishers(self):
        self.foot_force_markers_pub = self.create_publisher(
            MarkerArray,
            "/dog2/visualization/foot_forces",
            10,
        )
        self.trajectory_markers_pub = self.create_publisher(
            MarkerArray,
            "/dog2/visualization/trajectory",
            10,
        )
        self.contact_markers_pub = self.create_publisher(
            MarkerArray,
            "/dog2/visualization/contact_markers",
            10,
        )
        self.sliding_status_pub = self.create_publisher(
            MarkerArray,
            "/dog2/visualization/sliding_status",
            10,
        )
        self.performance_text_pub = self.create_publisher(
            MarkerArray,
            "/dog2/visualization/performance_text",
            10,
        )

    def robot_state_callback(self, msg):
        self.current_robot_state = msg
        self.base_history.append(
            Point(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
            )
        )
        if len(self.base_history) > self.max_history_points:
            self.base_history = self.base_history[-self.max_history_points :]

    def contact_state_callback(self, msg):
        self.current_contact_state = msg

    def grf_reference_callback(self, msg):
        self.current_grf_reference = msg

    def wbc_debug_callback(self, msg):
        self.current_wbc_debug = msg

    def horizon_callback(self, msg):
        self.current_horizon = msg

    def foot_forces_callback(self, msg):
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 foot forces, got {len(msg.data)}")
            return
        self.current_foot_forces = msg

    def timer_callback(self):
        if self.current_robot_state is None:
            return

        self._update_foot_force_visualization()
        self._update_trajectory_visualization()
        self._update_contact_visualization()
        self._update_wbc_status_visualization()
        self._update_performance_visualization()

    def _grf_as_flat_list(self):
        if self.current_grf_reference is not None and len(self.current_grf_reference.forces) == 4:
            values = []
            for force in self.current_grf_reference.forces:
                values.extend([force.x, force.y, force.z])
            return values
        if self.current_foot_forces is not None:
            return list(self.current_foot_forces.data)
        return None

    def _contact_flags(self):
        if self.current_contact_state is not None and self.current_contact_state.in_contact:
            return list(self.current_contact_state.in_contact)
        return [True, True, True, True]

    def _update_foot_force_visualization(self):
        try:
            foot_forces = self._grf_as_flat_list()
            if foot_forces is None:
                return
            marker_array = self.foot_force_viz.create_markers(
                foot_forces,
                self._contact_flags(),
            )
            self.foot_force_markers_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"Error updating foot force visualization: {e}")

    def _update_trajectory_visualization(self):
        marker_array = MarkerArray()

        com_marker = Marker()
        com_marker.header = self.current_robot_state.header
        com_marker.ns = "com"
        com_marker.id = 0
        com_marker.type = Marker.SPHERE
        com_marker.action = Marker.ADD
        com_marker.pose.position = self.current_robot_state.pose.position
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.06
        com_marker.scale.y = 0.06
        com_marker.scale.z = 0.06
        com_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)
        marker_array.markers.append(com_marker)

        trail_marker = Marker()
        trail_marker.header = self.current_robot_state.header
        trail_marker.ns = "base_trail"
        trail_marker.id = 1
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.scale.x = 0.015
        trail_marker.color = ColorRGBA(r=0.1, g=0.8, b=0.9, a=0.8)
        trail_marker.points = list(self.base_history)
        marker_array.markers.append(trail_marker)

        if self.current_horizon is not None:
            horizon_marker = Marker()
            horizon_marker.header = self.current_horizon.header
            horizon_marker.ns = "mpc_horizon"
            horizon_marker.id = 2
            horizon_marker.type = Marker.LINE_STRIP
            horizon_marker.action = Marker.ADD
            horizon_marker.scale.x = 0.012
            horizon_marker.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)
            horizon_marker.points = list(self.current_horizon.com_points)
            marker_array.markers.append(horizon_marker)

        self.trajectory_markers_pub.publish(marker_array)

    def _update_contact_visualization(self):
        marker_array = MarkerArray()
        foot_positions = self.foot_force_viz.foot_positions
        contact_flags = self._contact_flags()
        for leg_index, point in enumerate(foot_positions):
            marker = Marker()
            marker.header = self.current_robot_state.header
            marker.ns = "contact_state"
            marker.id = leg_index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            if leg_index < len(contact_flags) and contact_flags[leg_index]:
                marker.color = ColorRGBA(r=0.0, g=0.9, b=0.2, a=0.8)
            else:
                marker.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.8)
            marker_array.markers.append(marker)
        self.contact_markers_pub.publish(marker_array)

    def _make_text_marker(self, marker_id: int, ns: str, text: str, z: float):
        marker = Marker()
        marker.header = self.current_robot_state.header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=0.0, y=0.0, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = text
        return marker

    def _update_wbc_status_visualization(self):
        if self.current_wbc_debug is None:
            return
        marker_array = MarkerArray()
        marker_array.markers.append(
            self._make_text_marker(
                0,
                "wbc_status",
                "WBC tasks: " + ", ".join(self.current_wbc_debug.active_tasks),
                0.35,
            )
        )
        self.sliding_status_pub.publish(marker_array)

    def _update_performance_visualization(self):
        marker_array = MarkerArray()
        mode_text = "MPC: no debug"
        if self.current_grf_reference is not None:
            mode_text = "MPC: GRF ready"
        marker_array.markers.append(self._make_text_marker(0, "perf", mode_text, 0.45))

        if self.current_wbc_debug is not None:
            marker_array.markers.append(
                self._make_text_marker(
                    1,
                    "perf",
                    f"WBC torque_norm={self.current_wbc_debug.torque_norm:.2f}",
                    0.52,
                )
            )

        self.performance_text_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

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
