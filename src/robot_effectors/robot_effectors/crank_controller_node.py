#!/usr/bin/env python3
"""
crank_controller_node.py
~~~~~~~~~~~~~~~~~~~~~~~~
Controls the 3D-printed DSServo crank effector on Raspberry Pi 4/5.

Auto-trigger logic
──────────────────
Monitors the Nav2 /navigate_to_pose action via its result topic and
also listens to /effectors/crank_trigger (std_msgs/Bool) for manual
overrides from other nodes.

When triggered, the crank sweeps:
    retracted (0°)  →  extended (180°)  →  retracted (0°)

GPIO:
    Crank servo → BCM pin 4  (default)

DSServo PWM spec (same as paddles):
    50 Hz, 0.5–2.5 ms pulse → 0–180°
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

SERVO_FREQ = 50  # Hz


def angle_to_duty(angle_deg: float) -> float:
    """Convert servo angle (0–180°) to RPi.GPIO PWM duty cycle."""
    pulse_ms = 0.5 + (angle_deg / 180.0) * 2.0
    return (pulse_ms / 20.0) * 100.0


class CrankControllerNode(Node):
    """
    Drives the 3D-printed crank servo when Nav2 reaches a navigation goal,
    or when a manual trigger message is received on /effectors/crank_trigger.
    """

    def __init__(self):
        super().__init__("crank_controller_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("pin_crank",              4)
        self.declare_parameter("retracted_angle_deg",    0.0)    # home position
        self.declare_parameter("extended_angle_deg",   180.0)    # deployed position
        self.declare_parameter("sweep_duration_sec",     0.6)    # full sweep time
        self.declare_parameter("hold_duration_sec",      0.8)    # hold at extended
        self.declare_parameter("cooldown_seconds",       3.0)    # re-trigger guard

        self.pin_crank     = self.get_parameter("pin_crank").value
        self.retracted_deg = self.get_parameter("retracted_angle_deg").value
        self.extended_deg  = self.get_parameter("extended_angle_deg").value
        self.sweep_dur     = self.get_parameter("sweep_duration_sec").value
        self.hold_dur      = self.get_parameter("hold_duration_sec").value
        self.cooldown      = self.get_parameter("cooldown_seconds").value

        # ── GPIO setup ────────────────────────────────────────────────────
        self._pwm_crank = None
        self._setup_gpio()
        self._set_angle(self.retracted_deg)

        # ── State ─────────────────────────────────────────────────────────
        self._active       = False
        self._last_trigger = 0.0
        self._sweep_lock   = threading.Lock()

        # Track previously seen Nav2 goal statuses to detect new completions
        self._seen_goal_ids = set()

        # ── ROS interfaces ─────────────────────────────────────────────────
        # Nav2 goal status array — published by bt_navigator
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav_status_cb,
            10,
        )

        # Manual override topic
        self.manual_sub = self.create_subscription(
            Bool, "/effectors/crank_trigger", self._manual_trigger_cb, 10
        )

        # Publish crank activation events
        self.trigger_pub = self.create_publisher(Bool, "/effectors/crank_activated", 10)

        self.get_logger().info(
            "CrankControllerNode ready. "
            "Watching /navigate_to_pose/_action/status for goal completion."
        )

    # ─────────────────────────────────────────────────────────────────────
    # GPIO helpers
    # ─────────────────────────────────────────────────────────────────────

    def _setup_gpio(self):
        if not HW_AVAILABLE:
            self.get_logger().warn("RPi.GPIO not available — crank stub mode.")
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_crank, GPIO.OUT)
        self._pwm_crank = GPIO.PWM(self.pin_crank, SERVO_FREQ)
        self._pwm_crank.start(angle_to_duty(self.retracted_deg))

    def _set_angle(self, angle_deg: float):
        if not HW_AVAILABLE:
            self.get_logger().debug(f"[STUB] Crank angle={angle_deg:.1f}°")
            return
        self._pwm_crank.ChangeDutyCycle(angle_to_duty(angle_deg))

    # ─────────────────────────────────────────────────────────────────────
    # Crank sweep (runs in background thread)
    # ─────────────────────────────────────────────────────────────────────

    def _sweep_async(self):
        """
        Non-blocking crank sweep sequence:
          retracted → extended (sweep_dur s)
          hold at extended     (hold_dur s)
          extended → retracted (sweep_dur s)
        """
        with self._sweep_lock:
            steps = 30
            dt    = self.sweep_dur / steps

            # Extend
            for i in range(steps + 1):
                t     = i / steps
                angle = self.retracted_deg + t * (self.extended_deg - self.retracted_deg)
                self._set_angle(angle)
                time.sleep(dt)

            time.sleep(self.hold_dur)

            # Retract
            for i in range(steps + 1):
                t     = i / steps
                angle = self.extended_deg + t * (self.retracted_deg - self.extended_deg)
                self._set_angle(angle)
                time.sleep(dt)

            self._set_angle(self.retracted_deg)
            self._active = False

    def _fire_crank(self, reason: str):
        """Trigger the crank sweep if not in cooldown."""
        now = time.time()
        if self._active or (now - self._last_trigger < self.cooldown):
            return

        self._active       = True
        self._last_trigger = now
        self.get_logger().info(f"Crank activated — reason: {reason}")
        self.trigger_pub.publish(Bool(data=True))
        threading.Thread(target=self._sweep_async, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────
    # Nav2 goal status callback
    # ─────────────────────────────────────────────────────────────────────

    def _nav_status_cb(self, msg: GoalStatusArray):
        """
        Fires crank when a Nav2 NavigateToPose goal transitions to SUCCEEDED.

        GoalStatus codes:
            STATUS_ACCEPTED   = 1
            STATUS_EXECUTING  = 2
            STATUS_CANCELING  = 3
            STATUS_SUCCEEDED  = 4
            STATUS_CANCELED   = 5
            STATUS_ABORTED    = 6
        """
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)

            if (status.status == GoalStatus.STATUS_SUCCEEDED
                    and goal_id not in self._seen_goal_ids):
                self._seen_goal_ids.add(goal_id)
                self._fire_crank("Nav2 goal reached")

    # ─────────────────────────────────────────────────────────────────────
    # Manual trigger callback
    # ─────────────────────────────────────────────────────────────────────

    def _manual_trigger_cb(self, msg: Bool):
        """Allow other nodes or CLI to manually trigger the crank."""
        if msg.data:
            self._fire_crank("manual trigger")

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._set_angle(self.retracted_deg)
        if HW_AVAILABLE:
            if self._pwm_crank: self._pwm_crank.stop()
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrankControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
