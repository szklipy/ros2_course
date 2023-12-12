import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Pose as TurtlesimPose

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
            TurtlesimPose,
            '/turtle1/pose',
            self.cb_pose,
            10
        )

    def cb_pose(self, msg):
        self.pose = msg

    def go_straight(self, speed, distance):
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        initial_time = self.get_clock().now().nanoseconds

        while self.get_clock().now().nanoseconds - initial_time < int(1e9 * (distance / speed)):
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)
            loop_rate.sleep()

        # Stop the turtle
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)

    def turn(self, omega, angle):
        vel_msg = Twist()
        vel_msg.angular.z = omega

        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        initial_time = self.get_clock().now().nanoseconds

        while self.get_clock().now().nanoseconds - initial_time < int(1e9 * (angle / omega)):
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)
            loop_rate.sleep()

        # Stop the turtle
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def draw_koch_curve(self, order, size):
        self.koch_segment(order, size)

    def koch_segment(self, order, size):
        if order == 0:
            self.go_straight(1.0, size)
        else:
            self.koch_segment(order - 1, size / 3)
            self.turn(60.0, 0.0)
            self.koch_segment(order - 1, size / 3)
            self.turn(-120.0, 0.0)
            self.koch_segment(order - 1, size / 3)
            self.turn(60.0, 0.0)
            self.koch_segment(order - 1, size / 3)

    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    # Draw a Koch curve with order=3, size=100
    tc.draw_koch_curve(order=3, size=100)

    # Uncomment and run the following line to test other movements
    # tc.draw_poly(3.0, 300.0, 6, 2)  # speed, angular speed, vertices, edges

    # Destroy the node explicitly
    tc.destroy()

if __name__ == '__main__':
    main()
