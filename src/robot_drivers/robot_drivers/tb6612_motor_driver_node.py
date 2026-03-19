#!/usr/bin/env python3
"""
tb6612_motor_driver_node.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS2 driver for 2× TB6612FNG motor controllers driving 4 independent
DC wheels on a Raspberry Pi 4/5.

Wiring layout
─────────────
TB6612FNG #1  — FRONT driver
  Channel A → Front-Left  (FL) motor
  Channel B → Front-Right (FR) motor

TB6612FNG #2  — REAR driver
  Channel A → Rear-Left   (RL) motor
  Channel B → Rear-Right  (RR) motor

TB6612FNG truth table per channel:
  IN1  IN2  PWM   Result
   0    0    x    Brake (short low)
   1    0   duty  Forward
   0    1   duty  Reverse
   1    1    x    Brake (short high)

STBY pin must be HIGH for the driver to be enabled.

GPIO pin map (BCM numbering) — see README for full table.

Subscriptions:
    /cmd_vel  (geometry_msgs/Twist)  — linear.x + angular.z → skid-steer

Publications:
    /odom     (nav_msgs/Odometry)    — dead-reckoning from commanded speeds

Dependencies (install on Pi):
    pip install RPi.GPIO pigpio
    sudo systemctl enable pigpiod && sudo systemctl start pigpiod
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# ── Attempt hardware import; fall back to stub for off-Pi development ──
try:
    import RPi.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════
# TB6612FNG single-driver abstraction
# ══════════════════════════════════════════════════════════════════════

class TB6612Driver:
    """
    Controls one TB6612FNG chip (channels A and B).
    Uses RPi.GPIO software PWM at `pwm_freq` Hz.
    """

    def __init__(self, pwm_a, ain1, ain2, pwm_b, bin1, bin2, stby, pwm_freq=1000):
        self.pins = {
            "pwm_a": pwm_a, "ain1": ain1, "ain2": ain2,
            "pwm_b": pwm_b, "bin1": bin1, "bin2": bin2,
            "stby":  stby,
        }
        self.pwm_freq  = pwm_freq
        self._pwm_a    = None
        self._pwm_b    = None
        self._enabled  = False

    def setup(self):
        """Initialise GPIO pins. Call once at node startup."""
        if not HW_AVAILABLE:
            return
        for pin in self.pins.values():
            GPIO.setup(pin, GPIO.OUT)
        self._pwm_a = GPIO.PWM(self.pins["pwm_a"], self.pwm_freq)
        self._pwm_b = GPIO.PWM(self.pins["pwm_b"], self.pwm_freq)
        self._pwm_a.start(0)
        self._pwm_b.start(0)
        GPIO.output(self.pins["stby"], GPIO.HIGH)  # enable chip
        self._enabled = True

    def set_speed(self, channel: str, speed: float):
        """
        Set one channel's speed and direction.

        Args:
            channel: "A" or "B"
            speed:   −1.0 (full reverse) → 0.0 (stop) → +1.0 (full forward)
        """
        speed   = max(-1.0, min(1.0, speed))
        duty    = abs(speed) * 100.0   # RPi.GPIO PWM uses 0-100 duty cycle
        forward = speed >= 0.0

        if channel == "A":
            in1, in2, pwm_obj = self.pins["ain1"], self.pins["ain2"], self._pwm_a
        else:
            in1, in2, pwm_obj = self.pins["bin1"], self.pins["bin2"], self._pwm_b

        if not HW_AVAILABLE:
            return

        if duty < 1.0:
            # Brake: both IN pins LOW, PWM = 0
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            pwm_obj.ChangeDutyCycle(0)
        elif forward:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            pwm_obj.ChangeDutyCycle(duty)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            pwm_obj.ChangeDutyCycle(duty)

    def stop(self):
        """Brake both channels and disable the chip."""
        if not HW_AVAILABLE:
            return
        for ch in ("A", "B"):
            self.set_speed(ch, 0.0)
        GPIO.output(self.pins["stby"], GPIO.LOW)

    def cleanup(self):
        if self._pwm_a:
            self._pwm_a.stop()
        if self._pwm_b:
            self._pwm_b.stop()


# ══════════════════════════════════════════════════════════════════════
# ROS2 Node
# ══════════════════════════════════════════════════════════════════════

class TB6612MotorDriverNode(Node):
    """
    Subscribes to /cmd_vel, converts Twist to 4-wheel skid-steer speeds,
    drives both TB6612FNG chips, and publishes dead-reckoning odometry.
    """

    def __init__(self):
        super().__init__("tb6612_motor_driver_node")

        # ── Parameters ────────────────────────────────────────────────────
        # Physical
        self.declare_parameter("wheel_base_width",  0.29)   # m lateral distance between left/right
        self.declare_parameter("wheel_radius",      0.045)  # m
        self.declare_parameter("max_speed",         0.5)    # m/s

        # GPIO — Front driver (TB6612FNG #1)
        self.declare_parameter("front_pwm_a",  2)
        self.declare_parameter("front_ain1",   22)
        self.declare_parameter("front_ain2",   23)
        self.declare_parameter("front_pwm_b",  3)
        self.declare_parameter("front_bin1",   24)
        self.declare_parameter("front_bin2",   25)
        self.declare_parameter("front_stby",   30)

        # GPIO — Rear driver (TB6612FNG #2)
        self.declare_parameter("rear_pwm_a",   4)
        self.declare_parameter("rear_ain1",    26)
        self.declare_parameter("rear_ain2",    27)
        self.declare_parameter("rear_pwm_b",   5)
        self.declare_parameter("rear_bin1",    28)
        self.declare_parameter("rear_bin2",    29)
        self.declare_parameter("rear_stby",    31)

        self.wheel_base  = self.get_parameter("wheel_base_width").value
        self.wheel_r     = self.get_parameter("wheel_radius").value
        self.max_speed   = self.get_parameter("max_speed").value

        # ── GPIO setup ────────────────────────────────────────────────────
        if HW_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
        else:
            self.get_logger().warn(
                "RPi.GPIO not available — running in SOFTWARE STUB mode (no real motors)."
            )

        def _get(name): return self.get_parameter(name).value

        self.front_driver = TB6612Driver(
            pwm_a=_get("front_pwm_a"), ain1=_get("front_ain1"), ain2=_get("front_ain2"),
            pwm_b=_get("front_pwm_b"), bin1=_get("front_bin1"), bin2=_get("front_bin2"),
            stby=_get("front_stby"),
        )
        self.rear_driver = TB6612Driver(
            pwm_a=_get("rear_pwm_a"),  ain1=_get("rear_ain1"),  ain2=_get("rear_ain2"),
            pwm_b=_get("rear_pwm_b"),  bin1=_get("rear_bin1"),  bin2=_get("rear_bin2"),
            stby=_get("rear_stby"),
        )

        self.front_driver.setup()
        self.rear_driver.setup()

        # ── Odometry state ─────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self._last_left_speed  = 0.0
        self._last_right_speed = 0.0
        self._last_time = self.get_clock().now()

        # ── ROS interfaces ─────────────────────────────────────────────────
        self.cmd_sub   = self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.odom_pub  = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br     = TransformBroadcaster(self)

        # Odometry timer at 50 Hz
        self.create_timer(0.02, self._publish_odom)

        # Safety watchdog — stop motors if no /cmd_vel for 0.5 s
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info("TB6612MotorDriverNode ready. Listening on /cmd_vel.")

    # ─────────────────────────────────────────────────────────────────────
    # Skid-steer kinematics
    # ─────────────────────────────────────────────────────────────────────

    def _twist_to_wheel_speeds(self, linear: float, angular: float):
        """
        Convert Twist (linear.x, angular.z) to left/right wheel speeds
        using differential-drive / skid-steer kinematics.

        Returns:
            (left_speed, right_speed) normalised to [−1, +1]
        """
        left_mps  = linear - (angular * self.wheel_base / 2.0)
        right_mps = linear + (angular * self.wheel_base / 2.0)

        # Normalise by max speed to get duty-cycle fraction
        left_norm  = left_mps  / self.max_speed
        right_norm = right_mps / self.max_speed

        # If either exceeds 1.0, scale both down proportionally
        max_val = max(abs(left_norm), abs(right_norm), 1.0)
        left_norm  /= max_val
        right_norm /= max_val

        return left_norm, right_norm

    # ─────────────────────────────────────────────────────────────────────
    # /cmd_vel callback
    # ─────────────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        left, right = self._twist_to_wheel_speeds(msg.linear.x, msg.angular.z)

        # Front driver: A = FL, B = FR
        self.front_driver.set_speed("A", left)
        self.front_driver.set_speed("B", right)

        # Rear driver:  A = RL, B = RR
        self.rear_driver.set_speed("A", left)
        self.rear_driver.set_speed("B", right)

        # Store for odometry integration
        self._last_left_speed  = left  * self.max_speed
        self._last_right_speed = right * self.max_speed

    # ─────────────────────────────────────────────────────────────────────
    # Safety watchdog
    # ─────────────────────────────────────────────────────────────────────

    def _watchdog(self):
        """Stop motors if no command received within 0.5 seconds."""
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > 0.5:
            self.front_driver.set_speed("A", 0.0)
            self.front_driver.set_speed("B", 0.0)
            self.rear_driver.set_speed("A",  0.0)
            self.rear_driver.set_speed("B",  0.0)
            self._last_left_speed  = 0.0
            self._last_right_speed = 0.0

    # ─────────────────────────────────────────────────────────────────────
    # Odometry publisher
    # ─────────────────────────────────────────────────────────────────────

    def _publish_odom(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        lv = self._last_left_speed
        rv = self._last_right_speed

        v     = (lv + rv) / 2.0
        omega = (rv - lv) / self.wheel_base

        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += omega * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # TF
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id  = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_br.sendTransform(tf)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_footprint"
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.front_driver.stop()
        self.rear_driver.stop()
        self.front_driver.cleanup()
        self.rear_driver.cleanup()
        if HW_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TB6612MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
