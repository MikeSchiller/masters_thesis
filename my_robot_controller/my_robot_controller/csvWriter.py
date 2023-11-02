#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String
import array
actual_latitude = 0
actual_longitude = 0
lenkung = 0
i = 0
w= 4
h = 500
T = [[0 for x in range(w)] for y in range(h)]
#print(T)



class csvWriter(Node):
    def __init__(self):
        super().__init__('csvWriter')
        self.get_logger().info('csvWriter')

        self.sub_actlong = self.create_subscription(String, '/car_long', self.act_long_callback, 10)
        self.sub_actlat = self.create_subscription(String, '/car_lat', self.act_lat_callback, 10)
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    
    def timer_callback(self):
        global writer
        global actual_latitude
        global actual_longitude
        global i
        global T
        global h
        print (actual_latitude)
        print(actual_longitude)
        T[i][0] = i
        T[i][1] = actual_latitude
        T[i][2] = actual_longitude
        T[i][3] =  lenkung
        i = i+1
        print (T)
    
        if i == h:
        
            with open('coordinates_gps_testdrive1.csv', 'w+', newline='') as file:
                print ("meh")
                writer = csv.writer(file, lineterminator='\n')
                for k in range(h):

                    writer.writerow(T[k])
                    

   
        


    #get the actual gps data in degrees
    def act_long_callback(self, act_long):
        global actual_longitude
        global T
        #print ("miep")
        actual_longitude = float(act_long.data)
        

        

     #get the actual gps data in degrees   
    def act_lat_callback(self, act_lat):
        global actual_latitude
        global T
        actual_latitude = float(act_lat.data)
        
    def Steer_callback(self, lenk):
        global lenkung
        lenkung = float(lenk.data)








def main(args=None):
    rclpy.init(args=args)
    node = csvWriter()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    with open('coordinates_testdrive2.csv', 'w+', newline='') as file:
        print ("meh")
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(T)
 
        print(i)
    file.close()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
