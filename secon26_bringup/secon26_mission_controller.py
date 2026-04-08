#!/usr/bin/env python3
"""
secon26_mission_controller.py

Autonomous mission controller for IEEE SoutheastCon 2026.
Uses Nav2 NavigateToPose to sequence the robot through all competition tasks.

Coordinate system (map frame):
  Origin (0,0) = bottom left corner of arena (start box)
  +X = right (east, toward right border)
  +Y = up (north, toward top border)
  Arena dimensions: 2.4384m (X) x 1.2192m (Y)

Physical antenna locations (converted from measurements):
  Antenna 1: x=0.076, y=1.143  — 3in from left,  3in from top    — Button task
  Antenna 2: x=2.362, y=1.143  — 3in from right, 3in from top    — Crank task
  Antenna 3: x=1.219, y=0.305  — Center of crater                — Crater loop
  Antenna 4: x=0.610, y=0.076  — 2ft from start, 3in from bottom — Keypad task

  Crater center: x=1.219, y=0.305
  Crater radius: ~0.305m (1ft)
  Crater loop clearance: 0.45m from center (stays outside crater rim)

  Lunar landing corner: x=0.762, y=1.143 (2ft 6in from left, 3in from top)
  Lunar landing center: x=0.914, y=1.067 (estimated center of landing zone)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
import math
import time


def make_quaternion(yaw_radians: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_radians / 2.0)
    q.w = math.cos(yaw_radians / 2.0)
    return q


def pose(x: float, y: float, yaw_rad: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    p.pose.orientation = make_quaternion(yaw_rad)
    return p


# Yaw constants for readability
FACE_EAST  = 0.0
FACE_NORTH = 1.5708
FACE_WEST  = 3.1416
FACE_SOUTH = 4.7124

# Crater geometry
CRATER_X      = 1.2192
CRATER_Y      = 0.3048
CRATER_RADIUS = 0.305   # physical radius ~1ft
LOOP_RADIUS   = 0.45    # loop path radius — stays outside crater rim

# ─── Task poses ───────────────────────────────────────────────────────────────
TASK_POSES = {

    # Starting position
    'start': pose(0.15, 0.15, FACE_EAST),

    # ── Antenna #1 — Button task (press 3 times) ──────────────────────────────
    # Located 3in from left, 3in from top
    # Robot approaches from south, faces north toward button
    'antenna1_approach': pose(0.076, 1.050, FACE_NORTH),

    # ── Antenna #2 — Crank task (rotate 540°) ─────────────────────────────────
    # Located 3in from right, 3in from top
    # Robot approaches from south, faces north toward crank
    'antenna2_approach': pose(2.362, 1.050, FACE_NORTH),

    # ── Antenna #3 — Crater loop (do NOT enter crater) ────────────────────────
    # Robot loops around crater perimeter at safe distance (LOOP_RADIUS=0.45m)
    # Waypoints are 0.45m from crater center in each cardinal direction
    'crater_loop_south': pose(CRATER_X,               CRATER_Y - LOOP_RADIUS, FACE_EAST),
    'crater_loop_east':  pose(CRATER_X + LOOP_RADIUS, CRATER_Y,               FACE_NORTH),
    'crater_loop_north': pose(CRATER_X,               CRATER_Y + LOOP_RADIUS, FACE_WEST),
    'crater_loop_west':  pose(CRATER_X - LOOP_RADIUS, CRATER_Y,               FACE_SOUTH),
    'crater_loop_done':  pose(CRATER_X,               CRATER_Y - LOOP_RADIUS, FACE_EAST),

    # ── Antenna #4 — Keypad task (enter 73738#) ────────────────────────────────
    # Located 2ft from start, 3in from bottom
    # Robot approaches from south, faces north toward keypad
    'antenna4_approach': pose(0.610, 0.200, FACE_NORTH),

    # ── Lunar landing — deposit all ducks ─────────────────────────────────────
    # Corner at x=0.762, y=1.143 — approach center of landing zone
    'lunar_landing': pose(0.914, 1.067, FACE_SOUTH),

    # ── Earth comms — IR transmission ─────────────────────────────────────────
    # Near starting area, face toward Earth arm (hangs above west wall)
    'earth_comms': pose(0.20, 0.20, FACE_WEST),
}

# ─── Mission sequence ──────────────────────────────────────────────────────────
# Ordered for efficiency — antenna 4 first (closest to start),
# then sweep up to antennas 1 and 2, then crater loop, then return
MISSION_SEQUENCE = [
    'antenna4_approach',    # Keypad — Area 1, close to start
    'antenna1_approach',    # Button — Area 2, upper left
    'antenna2_approach',    # Crank  — Area 3, upper right
    'crater_loop_south',    # Begin crater loop
    'crater_loop_east',
    'crater_loop_north',
    'crater_loop_west',
    'crater_loop_done',     # Full loop complete
    'lunar_landing',        # Deposit ducks
    'earth_comms',          # IR transmission to Earth
    'start',                # Return to start (15pt bonus)
]


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._duck_collect_client = self.create_client(Trigger, '/servo/duck_collect')
        self._crank_turn_client   = self.create_client(Trigger, '/servo/crank_turn')
        self.get_logger().info('Mission controller initialised — waiting for Nav2...')

    def run(self):
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready — starting mission sequence')
        time.sleep(2.0)

        for step_name in MISSION_SEQUENCE:
            self.get_logger().info(f'── Navigating to: {step_name}')
            target = TASK_POSES[step_name]
            target.header.stamp = self.get_clock().now().to_msg()

            success = self._navigate_to(target)

            if not success:
                self.get_logger().warn(f'Navigation to {step_name} failed — continuing')
            else:
                self.get_logger().info(f'Reached {step_name}')
                self._post_step_action(step_name)

        self.get_logger().info('Mission sequence complete')

    def _navigate_to(self, target: PoseStamped) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = target
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle: ClientGoalHandle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result is not None

    def _call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {name} not available')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

    def _post_step_action(self, step_name: str):
        if step_name == 'antenna1_approach':
            self.get_logger().info('[ACTION] Antenna 1 — pressing button 3 times')
            # TODO: publish to /button_press_cmd

        elif step_name == 'antenna2_approach':
            self.get_logger().info('[ACTION] Antenna 2 — rotating crank 540°')
            self._call_service(self._crank_turn_client, '/servo/crank_turn')

        elif step_name == 'crater_loop_done':
            self.get_logger().info('[ACTION] Crater loop complete — 35pts scored')

        elif step_name == 'antenna4_approach':
            self.get_logger().info('[ACTION] Antenna 4 — entering keypad code 73738#')
            # TODO: publish to /keypad_cmd

        elif step_name == 'lunar_landing':
            self.get_logger().info('[ACTION] Depositing ducks in lunar landing area')
            self._call_service(self._duck_collect_client, '/servo/duck_collect')

        elif step_name == 'earth_comms':
            self.get_logger().info('[ACTION] Transmitting antenna LED colours via IR')
            # TODO: publish to /ir_transmit_cmd


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
