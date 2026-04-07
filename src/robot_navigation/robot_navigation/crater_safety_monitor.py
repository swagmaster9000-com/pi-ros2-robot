import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class CraterSafetyMonitor(Node):
    def __init__(self):
        super().__init__('crater_safety_monitor')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        self.warning_pub = self.create_publisher(Bool, '/crater_warning', 10)

        self.crater_bounds = {
            'xmin': 2.0,
            'xmax': 3.5,
            'ymin': -2.0,
            'ymax': -0.5
        }

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        in_crater_zone = (
            self.crater_bounds['xmin'] <= x <= self.crater_bounds['xmax'] and
            self.crater_bounds['ymin'] <= y <= self.crater_bounds['ymax']
        )

        warning = Bool()
        warning.data = in_crater_zone
        self.warning_pub.publish(warning)
