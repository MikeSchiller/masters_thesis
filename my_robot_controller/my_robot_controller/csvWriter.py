#Writes data to .csv File

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
#Radar
radararrayX = []
radararrayY = []
counttargets = 0
i = 0
w= 4
h = 5000
T = [[0 for x in range(w)] for y in range(h)]
#print(T)



class csvWriter(Node):
    def __init__(self):
        super().__init__('csvWriter')
        self.get_logger().info('csvWriter')

        self.sub_actlong = self.create_subscription(String, '/car_long', self.act_long_callback, 10)
        self.sub_actlat = self.create_subscription(String, '/car_lat', self.act_lat_callback, 10)
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)
        '''
        self.trackcount_sub = self.create_subscription(String,'/Radar_trackcount', self.count_callback, 10)
        self.distance_subscriber_y = self.create_subscription(String,'/Radar_distances_y', self.distance_y_callback, 10)
        self.distance_subscriber_x = self.create_subscription(String,'/Radar_distances_x', self.distance_x_callback, 10)
        
        '''
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    
    def timer_callback(self):
        global writer
        #global actual_latitude
        #global actual_longitude
        global radararrayX
        global radararrayY
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
        '''
        print (radararrayX)
        print(radararrayY)
        T[i][0] = i
        T[i][1] = radararrayX
        T[i][2] = radararrayY
        T[i][3] = counttargets
        radararrayX = []
        radararrayY = []
        '''
        print (T)
    
        
        
        with open('simfahrt_gps_1511_3.csv', 'w+', newline='') as file:
            print ("meh")
            print (i)
            writer = csv.writer(file, lineterminator='\n')
            for k in range(h):

                writer.writerow(T[k])
                    

   
    ''' 
    def distance_x_callback(self, distx):
        global radararrayX
        distances_x = distx.data
        distances_x = distances_x.replace("[","")
        distances_x = distances_x.replace("]","")
        splitdistx = distances_x.split(", ")
        radararrayX = radararrayX + splitdistx
        if not radararrayY :
            pass
        else:
            radararrayX = [float(string) for string in radararrayX]
        #self.get_logger().info('I heard for x: "%s"' % splitdistx)


    def distance_y_callback(self, disty):
        global radararrayY
        distances_y = disty.data
        distances_y = distances_y.replace("[","")
        distances_y = distances_y.replace("]","")
        splitdisty = distances_y.split(", ")
        radararrayY = radararrayY + splitdisty
        #print(radararrayY)
        if radararrayY == "" :
            pass
        else:
            radararrayY = [float(string) for string in radararrayY]
        

        #self.get_logger().info('I heard for y: "%s"' % splitdisty)
        
    def count_callback(self, count):
        global counttargets
        counttargets = int(count.data)
        #self.get_logger().info('count: "%s"' % count.data)
    '''

   
    #get the actual gps data in degrees
    def act_long_callback(self, act_long):
        global actual_longitude
        global T
        #print ("miep")
        try:
            actual_longitude = float(act_long.data)
        except:
            pass
        

        

     #get the actual gps data in degrees   
    def act_lat_callback(self, act_lat):
        global actual_latitude
        global T
        try:
            actual_latitude = float(act_lat.data)
        except:
            pass

   
        
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
