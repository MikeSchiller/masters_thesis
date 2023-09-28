import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import String

startLong = 0.0
startLat = 0.0
gpsLong = 0.0
gpsLat = 0.0
onlyfirstlong = 1
onlyfirstlat = 1
Fahrtrichtung = 0
distance_driven = 0
steering_angle = 0
heading = 0
radius_earth = 6371000 #6371km
calclat = 0
calclong = 0
radstand = 0.3 #(Radstand des Fahrzeugs in Meter)


class CarToCoords(Node):

    def __init__(self):
        super().__init__('carToCoords')
        self.car_long = self.create_publisher(String, 'car_long', 10)
        self.car_lat = self.create_publisher(String, 'car_lat', 10)
        self.pub_Zone1 = self.create_publisher(String, '/Zone1', 10)
        self.pub_Zone2 = self.create_publisher(String, '/Zone2', 10)
        
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)
        self.sub_schubPWM = self.create_subscription(String, '/car_setschubPWM', self.Schub_callback 10)
        self.sub_gpslong = self.create_subscription(String, '/act_longitude', self.act_long_callback, 10)
        self.sub_gpslat = self.create_subscription(String, '/act_latitude', self.act_lat_callback, 10)
        self.sub_odo = self.create_subscription(String,'/distance_driven', self.Odo_callback, 10)

       
    def coordsInput(self):
        global startLat
        global startLong
        global gpsLong
        global gpsLat
        global calclat
        global calclong
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
        
        calclong = startLong
        calclat = startLat
        
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



    def calculate_car_coords(self, car_dist, heading):
        #this method takes the distance driven, as well as the heading (which can be given by the sensor or calculated seperatly) and calculates the position in the glpbal coordinate space.
        global radius_earth
        global calclat
        global calclong
        diffgrad = 2 * math.asin(car_dist / 2 * radius_earth)
        
        if heading > 90 and heading <= 180:
            x = diffgrad * math.cos(heading)
            y = diffgrad * math.sin(heading) * (-1) 
        elif heading > 180 and heading <= 270:
            x = diffgrad * math.cos(heading) * (-1) 
            y = diffgrad * math.sin(heading) * (-1)        
        elif heading > 180 and heading <= 270:
            x = diffgrad * math.cos(heading) * (-1) 
            y = diffgrad * math.sin(heading)        
        elif heading > 0 and heading <= 90:
            x = diffgrad * math.cos(heading)
            y = diffgrad * math.sin(heading) 
        
        calclong = calclong + y
        calclat = calclat +x 
        
                 
    def calculate_heading(self,steering_angle, heading, part_distance_driven):
       # global steering_angle
        #global distance_driven
        global radstand
        current_heading = heading
        
        if  steering_angle != 0:
            beta = part_distance_driven/ (math.pi * (2* radstand/ math.sin(current_heading)) * 360)
            current_heading = current_heading + beta
            
            if current_heading >= 360:
                current_heading = current_heading - 360
                
            
        else: 
            return current_heading
        

    def Odo_callback(self, car_dist):
        global distance_driven
        global heading
        distance_driven = car_dist
        distance_driven_new = car_dist
        self.calculate_car_coords(distance_driven, heading)
        if part_distance_driven > 0.05 or steering_angle == 0 and checkleft != 0 or steering_angle == 0 and checkright != 0: #Heading berechnung alle 5cm // hier jetzt noch rein, dass auch abfrage, wenn winkel auf null gesetzt wird
            self.calculate_heading(steering_angle,heading, part_distance_driven)
            part_distance_driven = 0
        else:
            part_distance_driven = distance_driven_new - distance_driven_old
        
        distance_driven_old = distance_driven_new
        
        

    def Steer_callback(self, car_steer):
        # receives steering angle in range from 40 ( full right), 90 (straight ahead) to 130 (full left)
        global heading
        global steering_angle
        steering_angle = car_steer - 90
        if steering_angle > 0 :
            checkleft = 1
            checkcheck = 2

        elif steering_angle < 0:
            checkright = 1
            checkcheck = 2

        elif steering_angle == 0 and checkcheck >0 : # kurze überbrückung damit ende der lenkung abgefangen werden kann.
            checkcheck = -1

        else:
            checkleft = 0
            checkright = 0
        

      #hier muss jetzt eine Methode rein, die aus dem Lenkwinkel das heading bestimmt  
      
    
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