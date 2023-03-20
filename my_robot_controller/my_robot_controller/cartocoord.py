import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
startLong = 0.0
startLat = 0.0
gpsLong = 0.0
gpsLat = 0.0
onlyfirstlong = 1
onlyfirstlat = 1
Fahrtrichtung = 0


class CarToCoords(Node):

    def __init__(self):
        super().__init__('carToCoords')
        self.car_long = self.create_publisher(String, 'car_long', 10)
        self.car_lat = self.create_publisher(String, 'car_lat', 10)
        self.pub_Zone1 = self.create_publisher(String, '/Zone1', 10)
        self.pub_Zone2 = self.create_publisher(String, '/Zone2', 10)
        self.sub_odo = self.create_subscription(String,'/distance_driven', self.Odo_callback, 10)
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)
        self.sub_schubPWM = self.create_subscription(String, '/car_setschubPWM', self.Schub_callback 10)
        self.sub_gpslong = self.create_subscription(String, '/act_longitude', self.act_long_callback, 10)
        self.sub_gpslat = self.create_subscription(String, '/act_latitude', self.act_lat_callback, 10)

       
    def coordsInput(self):
        global startLat
        global startLong
        global gpsLong
        global gpsLat
        msgZone1 = String()
        msgZone2 = String()
        
        print("Use the Onboard GPS (g) or set it Yourself (y) ?: ")
        setvar = input()
        if setvar == "g" :
            print ("Using onboard GPS, please wait...")
            startLong = gpsLong
            startLat = gpsLat
        elif setvar == "y" :
            print ("Please enter your start coordinates in degrees. First the  Longitude: ")
            startLong = input()
            print("Now the Latitude: ")
            startLat = input()
            print ("Your Startcoordinates are: " + str(startLong) + str(startLat) )
        else :
            self.get_logger().warning("Input not supported!")
        
        print("Do you want to enable NO GO Zones? (Y/N/help): ")
        nogoset = input()
        if nogoset == "Y":
            print("Please input your NO GO Zone (Input `1` for default Zone):")
            Zone1 = input()
            if Zone1 == "1":
                Zone1 = "(48.417985,9.938193,48.417956,9.940275,48.417710,9.940271,48.417805,9.938137)"
            else:
                msgZone1.data = Zone1
                self.pub_Zone1.publish(msgZone1)
           # print ("do you want another one? (Y/N)")
           # secondset = input()
            #if secondset == "Y":
             #   print("Please input your NO GO Zone")
              #  Zone2 = input()
               # msgZone2.data = Zone2
                #self.pub_Zone2.publish(msgZone2)

        elif nogoset == "N" :
            print("okay, moving on")
        elif nogoset == "help" :
            print("NO GO Zones are areas defined by coordinates, where the platform isn´t allowed to drive trough. A zone is defined by 4 points starting in the upper left corner and continuing clockwise. PLease use the degreeformat and seperate using `,` and put into parentheses. Do not use spaces in the input. Currently, One Zone can be active.") #You can have a total of two zones active at a time")
        else :
            self.get_logger().warning("Input not supported!")



    def Odo_callback(self, car_dist):
        car_odo1 = car_odo

    def Steer_callback(self, car_steer):
        
        pass
    
    def Schub_callback(self, car_schub):
        global Fahrtrichtung
        stoppschub = 7.3
        if car_schub > stoppschub :
            Fahrtrichtung = 1
        elif car_schub < stoppschub :
            Fahrtrichtung = -1 
        
        pass
    
    def act_long_callback(self, gps_long):
        if onlyfirstlong == 1:
            global gpsLong
            gpsLong = gps_long
            onlyfirstlong = 2
        
    def act_lat_callback(self, gps_lat):
        if onlyfirstlat == 1:
            global gpsLat
            gpsLat = gps_lat
            onlyfirstlat = 2



def main(args=None):
    rclpy.init(args=args)

    Car_to_coords = CarToCoords()

    rclpy.spin(Car_to_coords)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Car_to_coords.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()