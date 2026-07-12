#!/usr/bin/env python3
"""Minimal gait scheduler for phase-1/phase-2 system integration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import rclpy
from dog2_interfaces.msg import ContactPhase, GaitCommand
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64

LEG_NAMES = ["lf", "lh", "rh", "rf"]


# Diagonal-pair phase offsets: lf/rh start their stance at phase 0,
# lh/rf half a cycle later.
_TROT_OFFSETS = (0.0, 0.5, 0.0, 0.5)
# Reposition the two front feet before either hind foot. In the measured
# startup stance the COM is about 5 cm behind the foot-rectangle centre;
# unloading a hind foot first puts the COM outside the remaining triangle
# and would require a negative normal force. LF/RF are feasible at startup,
# and their first touchdowns centre the support polygon for LH/RH.
CRAWL_SWING_ORDER = (0, 3, 1, 2)  # lf, rf, lh, rh

# Nominal feet relative to the body-centred COM reference. The support
# triangle centroid after removing one foot gives a conservative body shift
# direction before that foot is unloaded.
_CRAWL_NOMINAL_FOOTS_XY = (
    (-0.092, -0.118),
    (0.184, -0.118),
    (0.184, 0.118),
    (-0.092, 0.118),
)
_CRAWL_COM_XY = (0.046, 0.0)


@dataclass(frozen=True)
class CrawlOutput:
    phases: List[int]
    state: str
    active_leg: int
    body_shift_x: float
    body_shift_y: float
    # One-shot liveness event emitted on the tick a deadline expires:
    # "forced_settle", "shift_timeout_fault" or "settle_timeout_fault".
    event: str = ""


def crawl_body_shift(leg: int, scale: float = 0.65) -> tuple[float, float]:
    """Return a COM shift toward the remaining support-triangle centroid."""
    support = [
        position
        for index, position in enumerate(_CRAWL_NOMINAL_FOOTS_XY)
        if index != leg
    ]
    support_x = sum(position[0] for position in support) / len(support)
    support_y = sum(position[1] for position in support) / len(support)
    return (
        scale * (support_x - _CRAWL_COM_XY[0]),
        scale * (support_y - _CRAWL_COM_XY[1]),
    )


class ContactAwareCrawl:
    """Three-foot-support crawl with release/touchdown confirmation.

    Liveness contract: no state may wait on external evidence forever.

    - SWING past ``max_swing_sec`` without a confirmed touchdown is forced
      into SETTLE. The leg is commanded as stance again, so the MPC/WBC
      ground-search presses it down deterministically instead of the swing
      PD holding it in the air (measured 9.16 s hovering swings ended in
      BASE_CONTACT). SETTLE still requires real all-foot contact before the
      next SHIFT, so the 15 N unload gate is never diluted.
    - SHIFT/SETTLE past their deadlines latch FAULT: a safe stand that keeps
      all-stance phases and freezes the body-shift command (snapping it to
      zero previously caused a 2-3 cm reference reversal). FAULT clears only
      through a stop command. The measured alternative was a 50 s silent
      unload deadlock inside a 51.3 s stage budget.
    """

    def __init__(
        self,
        *,
        pre_shift_sec: float,
        swing_sec: float,
        settle_sec: float,
        max_swing_sec: float,
        max_shift_sec: float,
        max_settle_sec: float,
        body_shift_scale: float,
        contact_aware: bool,
    ) -> None:
        self.pre_shift_sec = max(0.0, float(pre_shift_sec))
        self.swing_sec = max(0.05, float(swing_sec))
        self.settle_sec = max(0.0, float(settle_sec))
        self.max_swing_sec = max(self.swing_sec, float(max_swing_sec))
        self.max_shift_sec = max(self.pre_shift_sec, float(max_shift_sec))
        self.max_settle_sec = max(self.settle_sec, float(max_settle_sec))
        self.body_shift_scale = max(0.0, float(body_shift_scale))
        self.contact_aware = bool(contact_aware)
        self._moving = False
        self.reset()

    def reset(self) -> None:
        self.state = "SHIFT"
        self.order_index = 0
        self.elapsed_sec = 0.0
        self.release_seen = False
        self.shift_start_x = 0.0
        self.shift_start_y = 0.0
        self.fault_shift_x = 0.0
        self.fault_shift_y = 0.0

    @property
    def active_leg(self) -> int:
        return CRAWL_SWING_ORDER[self.order_index]

    def _commanded_shift(self) -> tuple[float, float]:
        target_x, target_y = crawl_body_shift(
            self.active_leg, self.body_shift_scale
        )
        if self.state == "SHIFT":
            ratio = min(1.0, self.elapsed_sec / max(self.pre_shift_sec, 1e-6))
            blend = ratio * ratio * (3.0 - 2.0 * ratio)
            return (
                self.shift_start_x + blend * (target_x - self.shift_start_x),
                self.shift_start_y + blend * (target_y - self.shift_start_y),
            )
        # Keep the COM inside the completed leg's support triangle through
        # touchdown. Returning to zero here caused a 2-3 cm lateral
        # reference reversal immediately after impact.
        return target_x, target_y

    def _latch_fault(self) -> None:
        self.fault_shift_x, self.fault_shift_y = self._commanded_shift()
        self.state = "FAULT"
        self.elapsed_sec = 0.0

    def _output(self, event: str = "") -> CrawlOutput:
        phases = [ContactPhase.STANCE] * 4
        if self.state == "SWING":
            phases[self.active_leg] = ContactPhase.SWING

        if self.state == "FAULT":
            body_shift_x = self.fault_shift_x
            body_shift_y = self.fault_shift_y
        else:
            body_shift_x, body_shift_y = self._commanded_shift()
        return CrawlOutput(
            phases=phases,
            state=self.state,
            active_leg=self.active_leg,
            body_shift_x=body_shift_x,
            body_shift_y=body_shift_y,
            event=event,
        )

    def update(
        self,
        dt_sec: float,
        moving: bool,
        actual_contacts: List[bool],
        shift_ready: bool = True,
    ) -> CrawlOutput:
        if len(actual_contacts) != 4:
            raise ValueError("actual_contacts must contain lf,lh,rh,rf")
        if not moving:
            self._moving = False
            self.reset()
            return CrawlOutput(
                phases=[ContactPhase.STANCE] * 4,
                state="STAND",
                active_leg=self.active_leg,
                body_shift_x=0.0,
                body_shift_y=0.0,
            )
        if not self._moving:
            self.reset()
            self._moving = True

        self.elapsed_sec += max(0.0, float(dt_sec))
        all_contact = all(actual_contacts)
        event = ""

        if self.state == "SHIFT":
            ready = (not self.contact_aware or all_contact) and shift_ready
            if self.elapsed_sec >= self.pre_shift_sec and ready:
                self.state = "SWING"
                self.elapsed_sec = 0.0
                self.release_seen = False
            elif self.elapsed_sec >= self.max_shift_sec:
                self._latch_fault()
                event = "shift_timeout_fault"
        elif self.state == "SWING":
            active_contact = actual_contacts[self.active_leg]
            if not active_contact:
                self.release_seen = True
            touchdown = self.release_seen and active_contact
            ready = not self.contact_aware or touchdown
            if self.elapsed_sec >= self.swing_sec and ready:
                self.state = "SETTLE"
                self.elapsed_sec = 0.0
            elif self.contact_aware and self.elapsed_sec > self.max_swing_sec:
                self.state = "SETTLE"
                self.elapsed_sec = 0.0
                event = "forced_settle"
        elif self.state == "SETTLE":
            ready = not self.contact_aware or all_contact
            if self.elapsed_sec >= self.settle_sec and ready:
                self.shift_start_x, self.shift_start_y = crawl_body_shift(
                    self.active_leg, self.body_shift_scale
                )
                self.order_index = (self.order_index + 1) % len(CRAWL_SWING_ORDER)
                self.state = "SHIFT"
                self.elapsed_sec = 0.0
                self.release_seen = False
            elif self.elapsed_sec >= self.max_settle_sec:
                self._latch_fault()
                event = "settle_timeout_fault"

        return self._output(event)


def compute_phase_array(
    gait: str, phase: float, moving: bool, duty: float = 0.5
) -> List[int]:
    if not moving:
        return [ContactPhase.STANCE for _ in LEG_NAMES]
    if gait == "trot":
        # duty > 0.5 gives an all-stance overlap between the diagonal
        # pairs. A two-point diagonal support has zero authority about the
        # line joining the feet, and this robot's COM sits ~2.7 cm off
        # that line: with an instant 50/50 switch the trunk free-tips
        # ~0.4 rad every half cycle (run40 forward tilt hit 1.07 rad in
        # the first half cycle). The overlap windows catch the tip with
        # four feet before it accumulates.
        return [
            ContactPhase.STANCE
            if ((phase - offset) % 1.0) < duty
            else ContactPhase.SWING
            for offset in _TROT_OFFSETS
        ]
    if gait == "crawl":
        slot_position = (phase % 1.0) * 4.0
        slot = min(3, int(slot_position))
        local_phase = slot_position - slot
        swing_slot_fraction = min(1.0, max(0.0, (1.0 - duty) * 4.0))
        phases = [ContactPhase.STANCE] * 4
        if local_phase < swing_slot_fraction:
            phases[CRAWL_SWING_ORDER[slot]] = ContactPhase.SWING
        return phases
    return [ContactPhase.STANCE, ContactPhase.STANCE, ContactPhase.STANCE, ContactPhase.SWING]


class GaitSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("dog2_gait_scheduler")

        self.declare_parameter("gait", "trot")
        self.declare_parameter("cycle_time", 0.8)
        self.declare_parameter("duty_factor", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("contact_phase_topic", "/dog2/gait/contact_phase")
        self.declare_parameter("gait_command_topic", "/dog2/gait/command")
        self.declare_parameter("body_shift_topic", "/dog2/gait/body_shift")
        self.declare_parameter(
            "body_shift_error_topic", "/dog2/mpc/body_shift_error"
        )
        self.declare_parameter("body_shift_tolerance_m", 0.012)
        self.declare_parameter("shift_error_freshness_sec", 0.20)
        self.declare_parameter("contact_aware", True)
        self.declare_parameter("contact_freshness_sec", 0.20)
        self.declare_parameter("crawl_pre_shift_sec", 0.45)
        self.declare_parameter("crawl_swing_sec", 0.45)
        self.declare_parameter("crawl_settle_sec", 0.20)
        self.declare_parameter("crawl_max_swing_sec", 1.20)
        # Liveness deadlines. Healthy unload waits measured a few seconds;
        # the 8 s cap converts the observed 50 s silent deadlock into an
        # attributable fault well inside the 45-51 s stage budgets.
        self.declare_parameter("crawl_max_shift_sec", 8.0)
        # Post-forced-settle ground search from the ~5 mm touchdown
        # overshoot needs well under a second; 3 s marks a leg that cannot
        # reach the ground at all.
        self.declare_parameter("crawl_max_settle_sec", 3.0)
        self.declare_parameter("crawl_body_shift_scale", 0.65)
        for leg in LEG_NAMES:
            self.declare_parameter(
                f"foot_contact_topic_{leg}", f"/dog2/foot_contact/{leg}"
            )

        self._gait = str(self.get_parameter("gait").value)
        self._cycle_time = max(0.1, float(self.get_parameter("cycle_time").value))
        self._duty_factor = min(
            0.9, max(0.5, float(self.get_parameter("duty_factor").value))
        )
        self._publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self._elapsed = 0.0
        self._moving = False
        self._last_cmd = Twist()
        self._contact_aware = bool(self.get_parameter("contact_aware").value)
        self._contact_freshness_sec = max(
            0.01, float(self.get_parameter("contact_freshness_sec").value)
        )
        self._actual_contacts = [False] * 4
        self._contact_stamp_sec = [float("-inf")] * 4
        self._body_shift_error = float("inf")
        self._body_shift_error_stamp_sec = float("-inf")
        self._body_shift_tolerance_m = max(
            0.0, float(self.get_parameter("body_shift_tolerance_m").value)
        )
        self._shift_error_freshness_sec = max(
            0.01,
            float(self.get_parameter("shift_error_freshness_sec").value),
        )
        self._crawl = ContactAwareCrawl(
            pre_shift_sec=float(self.get_parameter("crawl_pre_shift_sec").value),
            swing_sec=float(self.get_parameter("crawl_swing_sec").value),
            settle_sec=float(self.get_parameter("crawl_settle_sec").value),
            max_swing_sec=float(self.get_parameter("crawl_max_swing_sec").value),
            max_shift_sec=float(self.get_parameter("crawl_max_shift_sec").value),
            max_settle_sec=float(
                self.get_parameter("crawl_max_settle_sec").value
            ),
            body_shift_scale=float(
                self.get_parameter("crawl_body_shift_scale").value
            ),
            contact_aware=self._contact_aware,
        )
        self._crawl_last_state = "STAND"
        self._crawl_state_since_sec = self._now_sec()

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            20,
        )
        self.create_subscription(
            GaitCommand,
            str(self.get_parameter("gait_command_topic").value),
            self._on_gait_command,
            20,
        )
        self._contact_pub = self.create_publisher(
            ContactPhase,
            str(self.get_parameter("contact_phase_topic").value),
            10,
        )
        self._gait_cmd_pub = self.create_publisher(
            GaitCommand,
            str(self.get_parameter("gait_command_topic").value),
            10,
        )
        self._body_shift_pub = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("body_shift_topic").value),
            10,
        )
        if self._contact_aware:
            for leg_index, leg in enumerate(LEG_NAMES):
                self.create_subscription(
                    Contacts,
                    str(self.get_parameter(f"foot_contact_topic_{leg}").value),
                    lambda msg, index=leg_index: self._on_contact(index, msg),
                    20,
                )
        self.create_subscription(
            Float64,
            str(self.get_parameter("body_shift_error_topic").value),
            self._on_body_shift_error,
            20,
        )
        self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _set_moving(self, moving: bool) -> None:
        # Start every walk at phase 0: with duty > 0.5 that is inside the
        # all-stance overlap, so the first stride begins from a supported
        # 4-leg state instead of an arbitrary mid-swing phase.
        if moving and not self._moving:
            self._elapsed = 0.0
        self._moving = moving

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._set_moving(any(
            abs(value) > 1e-3
            for value in [msg.linear.x, msg.linear.y, msg.angular.z]
        ))

    def _on_gait_command(self, msg: GaitCommand) -> None:
        if msg.header.frame_id == self.get_name():
            return
        self._gait = msg.gait or self._gait
        self._last_cmd.linear.x = float(msg.linear_x)
        self._last_cmd.linear.y = float(msg.linear_y)
        self._last_cmd.angular.z = float(msg.angular_z)
        self._set_moving(any(
            abs(value) > 1e-3
            for value in [msg.linear_x, msg.linear_y, msg.angular_z]
        ))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_contact(self, leg: int, msg: Contacts) -> None:
        self._actual_contacts[leg] = bool(msg.contacts)
        self._contact_stamp_sec[leg] = self._now_sec()

    def _fresh_contacts(self) -> List[bool]:
        if not self._contact_aware:
            return [True] * 4
        now = self._now_sec()
        return [
            active and now - stamp <= self._contact_freshness_sec
            for active, stamp in zip(
                self._actual_contacts, self._contact_stamp_sec
            )
        ]

    def _on_body_shift_error(self, msg: Float64) -> None:
        self._body_shift_error = float(msg.data)
        self._body_shift_error_stamp_sec = self._now_sec()

    def _body_shift_ready(self) -> bool:
        return (
            self._now_sec() - self._body_shift_error_stamp_sec
            <= self._shift_error_freshness_sec
            and self._body_shift_error <= self._body_shift_tolerance_m
        )

    def _log_crawl_progress(self, crawl: CrawlOutput) -> None:
        """One line per liveness event and per state transition.

        The transition durations are the cadence-attribution record: they
        say exactly how long each SHIFT/SWING/SETTLE window took, so the
        per-cycle budget can be reconstructed from the node log alone.
        """
        if crawl.event:
            self.get_logger().error(
                f"crawl {crawl.event}: leg={LEG_NAMES[crawl.active_leg]} "
                f"after {self._now_sec() - self._crawl_state_since_sec:.2f}s"
            )
        if crawl.state != self._crawl_last_state:
            now = self._now_sec()
            self.get_logger().info(
                f"crawl {self._crawl_last_state}->{crawl.state} "
                f"leg={LEG_NAMES[crawl.active_leg]} "
                f"held={now - self._crawl_state_since_sec:.2f}s"
            )
            self._crawl_last_state = crawl.state
            self._crawl_state_since_sec = now

    def _on_timer(self) -> None:
        self._elapsed = (self._elapsed + 1.0 / self._publish_rate) % self._cycle_time
        phase = self._elapsed / self._cycle_time
        if self._gait == "crawl":
            crawl = self._crawl.update(
                1.0 / self._publish_rate,
                self._moving,
                self._fresh_contacts(),
                self._body_shift_ready(),
            )
            phases = crawl.phases
            self._log_crawl_progress(crawl)
        else:
            crawl = None
            phases = compute_phase_array(
                self._gait, phase, self._moving, self._duty_factor
            )

        contact_msg = ContactPhase()
        contact_msg.header.stamp = self.get_clock().now().to_msg()
        contact_msg.gait = self._gait
        contact_msg.leg_names = list(LEG_NAMES)
        contact_msg.phase = phases
        contact_msg.cycle_time = float(self._cycle_time)

        gait_cmd_msg = GaitCommand()
        gait_cmd_msg.header = contact_msg.header
        gait_cmd_msg.header.frame_id = self.get_name()
        gait_cmd_msg.gait = self._gait
        gait_cmd_msg.linear_x = float(self._last_cmd.linear.x)
        gait_cmd_msg.linear_y = float(self._last_cmd.linear.y)
        gait_cmd_msg.angular_z = float(self._last_cmd.angular.z)

        self._contact_pub.publish(contact_msg)
        self._gait_cmd_pub.publish(gait_cmd_msg)
        body_shift = Vector3Stamped()
        body_shift.header = contact_msg.header
        body_shift.header.frame_id = "base_link"
        if crawl is not None:
            body_shift.vector.x = crawl.body_shift_x
            body_shift.vector.y = crawl.body_shift_y
            if crawl.state != "STAND":
                # z is a discrete upcoming-leg code (1=lf ... 4=rf), not a
                # vertical shift. MPC uses it to verify the measured
                # three-foot support triangle before authorizing unload.
                body_shift.vector.z = float(crawl.active_leg + 1)
        self._body_shift_pub.publish(body_shift)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GaitSchedulerNode()
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
