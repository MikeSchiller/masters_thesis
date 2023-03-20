#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

counttargets = 0



class radarsub(Node):
    def __init__(self):
        super().__init__('proc_radar')
        self.get_logger().info('Empfänger kriegt piep')
        self.trackcount_sub = self.create_subscription(String,'/Radar_trackcount', self.count_callback, 10)
        self.distance_subscriber_y = self.create_subscription(String,'/Radar_distances_y', self.distance_y_callback, 10)
        self.distance_subscriber_x = self.create_subscription(String,'/Radar_distances_x', self.distance_x_callback, 10)
        



    def distance_x_callback(self, distx):
        distances_x = distx.data
        distances_x = distances_x.replace("[","")
        distances_x = distances_x.replace("]","")
        splitdistx = distances_x.split(", ")
        
        
        self.get_logger().info('I heard for x: "%s"' % splitdistx)


    def distance_y_callback(self, disty):
        distances_y = disty.data
        distances_y = distances_y.replace("[","")
        distances_y = distances_y.replace("]","")
        splitdisty = distances_y.split(", ")
        #sptest = float(splitdisty[1]) + float(splitdisty[2])

        self.get_logger().info('I heard for y: "%s"' % splitdisty)
        
    def count_callback(self, count):
        global counttargets
        counttargets = int(count.data)
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


