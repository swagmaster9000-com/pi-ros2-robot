import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AntennaTaskRouter(Node):
    def __init__(self):
        super().__init__('antenna_task_router')

        self.task_sub = self.create_subscription(
            String,
            '/active_task',
            self.task_callback,
            10
        )

        self.command_pub = self.create_publisher(String, '/antenna_command', 10)

    def task_callback(self, msg):
        task = msg.data

        command = String()

        if task == 'antenna_4':
            command.data = 'extend_keypad_arm'
        elif task == 'antenna_2':
            command.data = 'rotate_ir_transmitter'
        else:
            command.data = 'idle'

        self.command_pub.publish(command)
        self.get_logger().info(f'Sent antenna command: {command.data}')
