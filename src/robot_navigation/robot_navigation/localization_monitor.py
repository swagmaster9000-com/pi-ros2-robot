import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.latest_odom = None
        self.latest_amcl = None

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose.position
        self.compare_positions()

    def amcl_callback(self, msg):
        self.latest_amcl = msg.pose.pose.position
        self.compare_positions()

    def compare_positions(self):
        if self.latest_odom is None or self.latest_amcl is None:
            return

        dx = abs(self.latest_odom.x - self.latest_amcl.x)
        dy = abs(self.latest_odom.y - self.latest_amcl.y)

        if dx > 0.10 or dy > 0.10:
            self.get_logger().warn(
                f'Localization drift detected: dx={dx:.2f}, dy={dy:.2f}'
            )
