#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String
actual_latitude = 0
actual_longitude = 0
i = 1

with open('coordinates_testdrive1.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["SNo", "Name", "Subject"])

class csvWriter(Node):
    def __init__(self):
        super().__init__('csvWriter')
        self.get_logger().info('csvWriter')
        self.sub_actlong = self.create_subscription(String, '/car_long', self.act_long_callback, 10)
        self.sub_actlat = self.create_subscription(String, '/car_lat', self.act_lat_callback, 10)
        self.timer = self.create_timer(1, timer_callback)
    
    
    def timer_callback(self):
        global i
        writer.writerow([i, actual_latitude, actual_longitude])
        i = i+1


    #get the actual gps data in degrees
    def act_long_callback(self, act_long):
        global actual_longitude
        
        actual_longitude = float(act_long.data)
        

     #get the actual gps data in degrees   
    def act_lat_callback(self, act_lat):
        global actual_latitude
        actual_latitude = float(act_lat.data)







def main(args=None):
    rclpy.init(args=args)
    node = csvWriter()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
