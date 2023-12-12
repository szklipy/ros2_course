# This Python file uses the following encoding: utf-8
from PySide2 import QtWidgets
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ros2_course.turtle import TurtleSierpinski

class TurtleListener(Node):
    def __init__(self):
        super().__init__('turtle_listener')
        self.subscription = self.create_subscription(
            Int32,
            'sierpinski_size',
            self.listener_callback,
            10
        )
        self.subscription

        self.turtle_sierpinski = TurtleSierpinski()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sierpinski_size: {msg.data}')
        size = msg.data
        self.turtle_sierpinski.screen.clear()
        self.turtle_sierpinski.sierpinski(size / 2, size / 2, size)

def main(args=None):
    rclpy.init(args=args)
    turtle_listener = TurtleListener()
    rclpy.spin(turtle_listener)
    turtle_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
