#simple node for testing ROS according to ROS docs

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 90
        self.j = 3
        time.sleep(5)

    def timer_callback(self):
        msg1 = String()
        msg1.data = str(self.i)
        
        if self.i >= 130:
            self.i= 40
        else:
            self.i += 10

        msg2 = String()
        msg2.data = str(self.j)
        if self.j >= 12:
            self.j = 0
        else:
            self.j += 1



        self.publisher_.publish(msg1)
        self.get_logger().info('Publishing: "%s"' % msg1.data)
        time.sleep(0.5)
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()