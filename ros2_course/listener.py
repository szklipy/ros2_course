import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from ros2_course.turtle import TurtleSierpinski

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #self.turtle_sierpinski = TurtleSierpinski()



    def listener_callback(self, msg):
        self.get_logger().info('I heard msg: "%s"' % msg.data)
        size = msg.data
        #self.turtle_sierpinski.screen.clear()
        #self.turtle_sierpinski.sierpinski(size / 2, size / 2, size)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
