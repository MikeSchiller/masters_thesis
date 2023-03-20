#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String



class radarsub(Node):
    def __init__(self):
        super().__init__('proc_radar')
        self.get_logger().info('Empfänger kriegt piep')
        self.distance_subscriber_y = self.create_subscription(String,'/distance_y', self.distance_y_callback, 10)
        self.distance_subscriber_x = self.create_subscription(String,'/distance_x', self.distance_x_callback, 10)
        self.trackcount_sub = self.create_subscription(String,'/trackcount', self.count_callback, 10)



    def distance_x_callback(self, distx):
        self.get_logger().info('I heard for x: "%s"' % distx.data)
    def distance_y_callback(self, disty):
        self.get_logger().info('I heard for y: "%s"' % disty.data)
    def count_callback(self, count):
        self.get_logger().info('count: "%s"' % count.data)









def main(args=None):
    rclpy.init(args=args)
    node = radarsub()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


