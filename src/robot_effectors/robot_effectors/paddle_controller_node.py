#!/usr/bin/env python3
"""
paddle_controller_node.py
~~~~~~~~~~~~~~~~~~~~~~~~~
Controls 2× DSServo paddle effectors via RPi.GPIO PWM.

Auto-trigger logic
──────────────────
Subscribes to /scan (sensor_msgs/LaserScan).
When the minimum LiDAR range within the forward arc drops below
`paddle_trigger_distance`, both paddles sweep from their neutral
angle to `paddle_active_angle` and then back.

A `cooldown_seconds` parameter prevents re-triggering too fast.

GPIO:
    Paddle Left  → BCM pin 17  (default)
    Paddle Right → BCM pin 27  (default)

DSServo PWM spec:
    Period:    20 ms  (50 Hz)
    Pulse:     0.5 ms = 0°,  1.5 ms = 90°,  2.5 ms = 180°
    Duty cycle = pulse_width_ms / 20.0 * 100
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

SERVO_FREQ = 50  # Hz


def angle_to_duty(angle_deg: float) -> float:
    """Convert servo angle (0–180°) to RPi.GPIO PWM duty cycle (0–100)."""
    # DSServo: 0° = 0.5 ms, 180° = 2.5 ms, period = 20 ms
    pulse_ms  = 0.5 + (angle_deg / 180.0) * 2.0
    duty      = (pulse_ms / 20.0) * 100.0
    return duty


class PaddleControllerNode(Node):
    """
    Drives two 3D-printed paddle servos based on LiDAR proximity detection.
    """

    def __init__(self):
        super().__init__("paddle_controller_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("pin_left",               17)
        self.declare_parameter("pin_right",              27)
        self.declare_parameter("neutral_angle_deg",      90.0)   # resting position
        self.declare_parameter("active_angle_deg",       45.0)   # swept-out position
        self.declare_parameter("sweep_duration_sec",      0.3)   # time to complete swing
        self.declare_parameter("return_delay_sec",        0.5)   # hold at active before returning
        self.declare_parameter("paddle_trigger_distance", 0.5)   # metres
        self.declare_parameter("check_angle_deg",        40.0)   # ±degrees from front to watch
        self.declare_parameter("cooldown_seconds",        2.0)   # min time between triggers

        self.pin_left      = self.get_parameter("pin_left").value
        self.pin_right     = self.get_parameter("pin_right").value
        self.neutral_deg   = self.get_parameter("neutral_angle_deg").value
        self.active_deg    = self.get_parameter("active_angle_deg").value
        self.sweep_dur     = self.get_parameter("sweep_duration_sec").value
        self.return_delay  = self.get_parameter("return_delay_sec").value
        self.trigger_dist  = self.get_parameter("paddle_trigger_distance").value
        check_angle        = self.get_parameter("check_angle_deg").value
        self.check_rad     = math.radians(check_angle)
        self.cooldown      = self.get_parameter("cooldown_seconds").value

        # ── GPIO / PWM setup ──────────────────────────────────────────────
        self._pwm_left  = None
        self._pwm_right = None
        self._setup_gpio()

        # Move to neutral on startup
        self._set_angle(self.neutral_deg, self.neutral_deg)

        # ── State ─────────────────────────────────────────────────────────
        self._triggered     = False
        self._last_trigger  = 0.0   # epoch seconds
        self._sweep_lock    = threading.Lock()

        # ── ROS interfaces ─────────────────────────────────────────────────
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, 10
        )
        # Publish trigger events so other nodes can react
        self.trigger_pub = self.create_publisher(Bool, "/effectors/paddle_triggered", 10)

        self.get_logger().info(
            f"PaddleControllerNode ready. "
            f"Trigger distance: {self.trigger_dist} m, "
            f"check angle: ±{check_angle}°, "
            f"cooldown: {self.cooldown} s"
        )

    # ─────────────────────────────────────────────────────────────────────
    # GPIO helpers
    # ─────────────────────────────────────────────────────────────────────

    def _setup_gpio(self):
        if not HW_AVAILABLE:
            self.get_logger().warn("RPi.GPIO not available — paddle stub mode.")
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_left,  GPIO.OUT)
        GPIO.setup(self.pin_right, GPIO.OUT)
        self._pwm_left  = GPIO.PWM(self.pin_left,  SERVO_FREQ)
        self._pwm_right = GPIO.PWM(self.pin_right, SERVO_FREQ)
        self._pwm_left.start(angle_to_duty(self.neutral_deg))
        self._pwm_right.start(angle_to_duty(self.neutral_deg))

    def _set_angle(self, left_deg: float, right_deg: float):
        """Immediately move both paddles to given angles."""
        if not HW_AVAILABLE:
            self.get_logger().debug(f"[STUB] Paddle L={left_deg:.1f}° R={right_deg:.1f}°")
            return
        self._pwm_left.ChangeDutyCycle(angle_to_duty(left_deg))
        self._pwm_right.ChangeDutyCycle(angle_to_duty(right_deg))

    def _sweep_async(self):
        """
        Perform the paddle sweep in a background thread so it doesn't
        block the ROS2 spin thread.

        Sequence:
          1. Sweep from neutral → active  (over sweep_dur seconds)
          2. Hold at active               (return_delay seconds)
          3. Return to neutral            (over sweep_dur seconds)
        """
        with self._sweep_lock:
            steps   = 20
            dt      = self.sweep_dur / steps

            # Forward sweep: neutral → active
            for i in range(steps + 1):
                t     = i / steps
                angle = self.neutral_deg + t * (self.active_deg - self.neutral_deg)
                self._set_angle(angle, self.neutral_deg + t * (self.neutral_deg - self.active_deg))
                time.sleep(dt)

            time.sleep(self.return_delay)

            # Return sweep: active → neutral
            for i in range(steps + 1):
                t     = i / steps
                angle = self.active_deg + t * (self.neutral_deg - self.active_deg)
                self._set_angle(angle, self.active_deg + t * (self.active_deg - self.neutral_deg))
                time.sleep(dt)

            # Ensure we end exactly at neutral
            self._set_angle(self.neutral_deg, self.neutral_deg)
            self._triggered = False

    # ─────────────────────────────────────────────────────────────────────
    # LiDAR callback
    # ─────────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        """Check forward arc for objects within trigger distance."""
        if self._triggered:
            return

        now = time.time()
        if now - self._last_trigger < self.cooldown:
            return

        # Identify forward arc beam indices
        num_beams = len(msg.ranges)
        angle_span = msg.angle_max - msg.angle_min

        front_idx   = int((-msg.angle_min) / angle_span * num_beams)
        half_window = int(self.check_rad / msg.angle_increment)

        start = max(0, front_idx - half_window)
        end   = min(num_beams - 1, front_idx + half_window)

        fwd_ranges = [
            r for r in msg.ranges[start:end]
            if msg.range_min < r < msg.range_max
            and not math.isinf(r)
            and not math.isnan(r)
        ]

        if not fwd_ranges:
            return

        if min(fwd_ranges) <= self.trigger_dist:
            self._triggered    = True
            self._last_trigger = now

            self.get_logger().info(
                f"Paddle trigger! Closest object: {min(fwd_ranges):.2f} m"
            )
            self.trigger_pub.publish(Bool(data=True))

            # Non-blocking sweep in a background thread
            thread = threading.Thread(target=self._sweep_async, daemon=True)
            thread.start()

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._set_angle(self.neutral_deg, self.neutral_deg)
        if HW_AVAILABLE:
            if self._pwm_left:  self._pwm_left.stop()
            if self._pwm_right: self._pwm_right.stop()
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PaddleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
