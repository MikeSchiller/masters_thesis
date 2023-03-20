#takes the gps coordinates and either sends it toward gpsdrivecontroller or changes themto user defined coordinates

import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import String

startLong = 0.0
startLat = 0.0
gpsLong = 0.0
gpsLat = 0.0
Usegps = 0
onlyfirstlong = 1
onlyfirstlat = 1
Fahrtrichtung = 0
distance_driven = 0
steering_angle = 0
#heading = 0
radius_earth = 6371000 #6371km
calclat = 0
calclong = 0
calcHeading = 10.0
radstand = 0.315 #(Radstand des Fahrzeugs in Meter)
part_distance_driven = 0
distance_driven_old = 0
part_distance_driven_for_coord = 0
#
checkcheck = 0
checkleft = 0
checkright = 0
firstrun = 0


class CarToCoords(Node):

    def __init__(self):
        global firstrun
        super().__init__('carToCoords')


        self.car_long = self.create_publisher(String, 'car_long', 10)
        self.car_lat = self.create_publisher(String, 'car_lat', 10)
        self.pub_Zone1 = self.create_publisher(String, '/Zone1', 10)
        self.pub_heading = self.create_publisher(String,'headingFromCtC', 1)
        self.pub_target_long = self.create_publisher(String, '/target_long', 10)
        self.pub_target_lat = self.create_publisher(String, 'target_lat', 10)
        self.pub_start = self.create_publisher(String, 'start', 10)
        
        #einmalige Abfrage der Aufgabenparameter
        if firstrun == 0:
            self.coordsInput()
            firstrun = 1


        self.sub_cmps = self.create_subscription(String,'/cmps_heading',self.cmps_callback,10)
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)
        self.sub_schubPWM = self.create_subscription(String, '/car_setschubPWM', self.Schub_callback, 10)
        self.sub_odo = self.create_subscription(String,'/distance_driven', self.Odo_callback, 10)
        #Die bekommen keine Infos mehr
        # Korrektur, ich bin dumm, das kommt von GPS_DATA_STUFF
        self.sub_gpslong = self.create_subscription(String, '/act_longitude', self.act_long_callback, 10)
        self.sub_gpslat = self.create_subscription(String, '/act_latitude', self.act_lat_callback, 10)
        
        


       
    def coordsInput(self):
        global startLat
        global startLong
        global gpsLong
        global gpsLat
        global calclat
        global calclong
        global Usegps
        global calcHeading
        msgZone1 = String()
        target_longitude = String()
        target_latitude = String()
        startmsg= String()
        


        
        print("Use the Onboard GPS for outdoor use (g) or set it Yourself (y)(or set debug(d))?: ")
        setvar = input()

        if(setvar == "d"):
            print("debug parameters set")
            startLong = 9.9384605 # T Bau Gang Norden
            startLat= 48.4185308
            #startLong = 9.93847061 # Kreuzung T/Q Bau (indoor)
            #startLat= 48.41821953
            target_longitude.data = '9.93847061' # Kreuzung T/Q Bau (indoor)
            target_latitude.data = '48.41821953'
            calcHeading = 170
            #self.pub_target_lat.publish(target_latitude)
            #self.pub_target_long.publish(target_longitude)
        else:
        
            if setvar == "g" :
                print ("Using onboard GPS, please wait...")
                Usegps = 1
                startLong = gpsLong
                startLat = gpsLat
                
            elif setvar == "y" :
                print ("Please enter (e) your start coordinates in degrees (e.g. 48.4187985) or choose (c) a standart point : ")
                chooseorenter = input()
                if chooseorenter == 'e':
                    print("First the  Latitude: ")  
                    startLat = input()
                    startLat = float(startLat)
                
                    print("Now the Longitude: ")
                    startLong = input()
                    startLong = float(startLong)
                    print ("Your Startcoordinates are: " + str(startLong) + str(startLat) )
                elif chooseorenter == "c":
                    print("please select: ")
                    print ("(1) T Bau Gang norden")
                    print(" (2) Kreuzng T/Q Bau (indoor)")
                    startvar = input()
                    
                    match startvar:
                        case '1':
                            startLong = 9.9384605 # T Bau Gang Norden
                            startLat= 48.4185308
                        case "2":
                            startLong = 9.93847061 # Kreuzung T/Q Bau (indoor)
                            startLat= 48.41821953

                print("Now enter the start heading: ")
                calcHeading = input()
            else :
                self.get_logger().warning("Input not supported!")
                print("debug parameters set")
                startLong = 9.9384605 # T Bau Gang Norden
                startLat= 48.4185308
                target_longitude.data = '9.93847061' # Kreuzung T/Q Bau (indoor)
                target_latitude.data = '48.41821953'
                calcHeading = 340
            print(startLat)
            print(startLong)


            print("Please define a target (T) or choose from the list (L):")
            targetvar = input()
            if targetvar == "T":
                print("Please enter your target latitude in degrees (e.g. 48.4187985): ")
                target_latitude.data = input()
                print ("Now enter the target longitude: ")
                target_longitude.data = input()

            elif targetvar == "L":
                print("Here is the list with currently available targets: ")
                print(" (1) Kreuzung E-Radstellplatz")
                print(" (2) Kreuzung V-Bau")
                print(" (3) Ecke gemähte Wiese Osten")
                print(" (4) Kreuzung Zufahrt THU Höhenweg Osten")
                print(" (5) Kreuzung Zufahrt THU Höhenweg Westen")
                print(" (6) Bad Blau")
                print(" (7) Kreuzng T/Q Bau (indoor)")
                print(" (8) Kreuzng s/Q Bau (indoor)")
                print(" (9) ulmer Münster")

                tarstate = input()

                match tarstate:
                    case "1":
                        target_longitude.data = '9.939772' # Kreuzung E-Radstellplatz
                        target_latitude.data = '48.418074' # Kreuzung E-Radstellplatz
                    
                    case "2":
                        target_longitude.data = '9.937939' # Kreuzung V-Bau
                        target_latitude.data = '48.418091' # Kreuzung V-Bau 

                    case "3":
                        target_longitude.data = '9.937964' # Ecke gemähte Wiese Osten
                        target_latitude.data = '48.417796' # Ecke gemähte Wiese Osten  

                    case "4": 
                        target_longitude.data = '9.940302' # Kreuzung Zufahrt THU Höhenweg osten
                        target_latitude.data = '48.417305' # Kreuzung Zufahrt THU Höhenweg osten 

                    case "5":                                      
                        target_longitude.data = '9.937879' # Kreuzung Zufahrt THU Höhenweg westen
                        target_latitude.data = '48.417411' # Kreuzung Zufahrt THU Höhenweg westen

                    case "6":
                        target_longitude.data = '9.917946' # bad blau
                        target_latitude.data = '48.417771' # bad blau

                    case "7":
                        target_longitude.data = '9.93847061' # Kreuzung T/Q Bau (indoor)
                        target_latitude.data = '48.41821953'
                    
                    case "8": 
                        target_longitude.data = '9.93894926' # Kreuzung S/Q Bau (indoor)
                        target_latitude.data = '48.41823366'
                    
                    case "9":
                        target_longitude.data = '9.991781128225083' # Ulmer Münster
                        target_latitude.data = '48.39854269226725' # 


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

        elif nogoset == "N" :
            print("okay, let's go")

        elif nogoset == "help" :
            print("NO GO Zones are areas defined by coordinates, where the platform isn't allowed to drive trough. A zone is defined by 4 points starting in the upper left corner and continuing clockwise. PLease use the degreeformat and seperate using `,` and put into parentheses. Do not use spaces in the input. Currently, One Zone can be active.") #You can have a total of two zones active at a time")
        else :
            self.get_logger().warning("Input not supported!")
        
        calclong = startLong
        calclat = startLat
        startmsg.data = "go"
        #self.car_long()
        for i in range(10):
            self.pub_target_lat.publish(target_latitude)
            self.pub_target_long.publish(target_longitude)
            time.sleep(0.1)
            self.pub_start.publish(startmsg)
            i+=1



    def calculate_car_coords(self, car_distance, heading):
        #this method takes the distance driven, as well as the heading (which can be given by the sensor or calculated seperatly) and calculates the position in the global coordinate space.
        global radius_earth
        global gpsLong
        global gpsLat
        global calclat
        global calclong
        global Usegps
        diffgrad = 2 * math.asin(car_distance / (2 * radius_earth))
        heading = float(heading)
        
        if heading >= 90.0 and heading < 180:
            x = diffgrad * math.cos(heading)
            y = diffgrad * math.sin(heading) * (-1) 
        elif heading >= 180.0 and heading < 270:
            x = diffgrad * math.cos(heading) * (-1) 
            y = diffgrad * math.sin(heading) * (-1)        
        elif heading >= 270.0 and heading < 360:
            x = diffgrad * math.cos(heading) * (-1) 
            y = diffgrad * math.sin(heading)        
        elif heading >= 0.0 and heading < 90:
            x = diffgrad * math.cos(heading)
            y = diffgrad * math.sin(heading) 
        else:
            #i guess abfangen, wenn er zu beginn den wert noch nicht kennt/ oder irgendwann mal spinnt
            y = 0
            x = 0
           #pass
        
        calclong = calclong + y
        calclat = calclat + x 
        sendCalcLong = String()
        sendCalcLat = String()
        sendgpsLong = String()
        sendgpsLat = String()
        sendCalcLong.data = str(calclong)
        sendCalcLat.data = str(calclat)
        sendgpsLong.data = str(gpsLong)
        sendgpsLat.data = str(gpsLat)

            #wenn GPS aktiviert ist, senden von gps koordinaten, sonst verwenden von berechneten Koordinaten
        print("Usegps in coords: " + str(Usegps))
        if Usegps == 1:
            self.car_long.publish(sendgpsLong)
            print(sendgpsLat.data)
        else:
            self.car_long.publish(sendCalcLong)       
        if Usegps == 1:
            self.car_lat.publish(sendgpsLat)
            print(gpsLat)
        else:
            self.car_lat.publish(sendCalcLat)

        #print("long: " + str(calclong))
        #print("lat: " + str(calclat))
                 
    # Heading wird zusammen mit Odo alle 2cm berechnet
    def calculate_heading(self,steering_angle, part_distance_driven):
        print("Piep")
       # global steering_angle
        #global distance_driven
        global calcHeading
        global radstand

        current_heading = float(calcHeading)
        
        #Wenn Heading genau 0, Fehler weil div durch 0, daher bei 0 minimale Abweichung zugerechnet
        if  steering_angle != 0 and current_heading != 0:

            beta = part_distance_driven/ (math.pi * ((2* radstand)/ math.sin(steering_angle ))) * 360
            print("beta: " + str(beta))
            #keine Ahnung warum genau, aber beta wird selten seehr seehr groß, daher abfangen davon
            if beta < -360:
                pass
            else: 
                current_heading = current_heading + beta
             
            if current_heading >= 360:
                current_heading = current_heading - 360 
            elif current_heading < 0:
                current_heading = current_heading + 360 
            calcHeading = current_heading 
        elif steering_angle != 0 and current_heading == 0:
            beta = part_distance_driven/ (math.pi * (2* radstand/ math.sin(((steering_angle+0.1 )))) * 360)
            if beta < -360:
                pass
            else: 
                current_heading = current_heading + beta

            if current_heading >= 360:
                current_heading = current_heading - 360  
            elif current_heading < 0:
                current_heading = current_heading + 360   
            calcHeading = current_heading          
        else: 
            calcHeading= current_heading
        
        
        return calcHeading

    def Odo_callback(self, car_dist):
        global distance_driven
        global calcHeading
        global checkleft
        global checkright
        global checkcheck
        global part_distance_driven
        global distance_driven_old
        global part_distance_driven_for_coord
     
        #input tut
        if car_dist.data == "":
            distance_driven_new = 0
        else:
            distance_driven_new = float(car_dist.data) 
            #print ("new: " + str(distance_driven_new))
            #ist hier vllt der Fehler mit falschem Faktor, weil falsche Variable übergeben?
        #!!!!Das macht an dier stelle keinen Sinn
        #self.calculate_car_coords(distance_driven_new, calcHeading)

        #print("Piepsepiep " + str(part_distance_driven))
        #partdistance 
        part_distance_driven_this_iteration = distance_driven_new - distance_driven_old
        part_distance_driven = part_distance_driven + part_distance_driven_this_iteration
        part_distance_driven_for_coord = part_distance_driven_for_coord + part_distance_driven_this_iteration


        if part_distance_driven > 0.02 or steering_angle == 0 and checkleft != 0 or steering_angle == 0 and checkright != 0: #Heading berechnung alle 2cm // hier jetzt noch rein, dass auch abfrage, wenn winkel auf null gesetzt wird
            
            self.calculate_heading(steering_angle,part_distance_driven)
            #self.calculate_car_coords(part_distance_driven, calcHeading) # das hat sachen put gemacht
            part_distance_driven = 0
        else:
            pass
        #berechnung der Koordinaten jeden gefahrenen Meter (dadurch Koordinaten zwar ungenauer, aber hoffentlich weniger numerische Effekte)
        if part_distance_driven_for_coord > 1 or steering_angle == 0 and checkleft != 0 or steering_angle == 0 and checkright != 0: 
            self.calculate_car_coords(part_distance_driven_for_coord, calcHeading) # das hat sachen put gemacht
            part_distance_driven = 0
            part_distance_driven_for_coord = 0
        else:
            pass
        
        
        #print ("old: " + str(distance_driven_old))
        distance_driven_old = distance_driven_new
        


    def Steer_callback(self, car_steer):
        # receives steering angle in range from 40 ( full right), 90 (straight ahead) to 130 (full left)
        
        global steering_angle
        global checkleft
        global checkright
        global checkcheck
        if car_steer.data == "":
            steering_angle = 0
            
        else:
            #hier geht er rein
            steering_angle = float(car_steer.data) - 90
        print(steering_angle)
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
         
    def Schub_callback(self, car_schub):
        global Fahrtrichtung
        stoppschub = 7.3
        if car_schub.data != "":
            if float(car_schub.data) > stoppschub :
                Fahrtrichtung = 1
            elif float(car_schub.data) < stoppschub :
                Fahrtrichtung = -1 
        else:
            pass


 

     
    def act_long_callback(self, gps_long):
        global Usegps
        global calclong
        global gpsLong
        
            
        gpsLong = gps_long
            #wenn GPS aktiviert ist, senden von gps koordinaten, sonst verwenden von berechneten Koordinaten
            #der Part ist redundant, wenn GPS aktiv (macht das was aud????!!!)
        if Usegps == 1:
            self.car_long.publish(gps_long)
        else:
            self.car_long.publish(calclong)

    def act_lat_callback(self, gps_lat):
        global Usegps
        global calclat
        global gpsLat

        gpsLat = gps_lat
        if Usegps == 1:
            self.car_lat.publish(gps_lat)
            print(gps_lat.data)
        else:
            self.car_lat.publish(calclat)

          

    def cmps_callback(self,cmps_head):
 
        global cmps_heading
        global Usegps
        global calcHeading
        msg = String()
        if Usegps == 0:
            msg.data = str(calcHeading)
            self.pub_heading.publish(msg)
        else:
            msg.data = str(cmps_head.data)
            self.pub_heading.publish(msg)
        print("heading: " + msg.data)
        


def main(args=None):
    rclpy.init(args=args)

    Car_to_coords = CarToCoords()
    #CarToCoords.coordsInput()

    rclpy.spin(Car_to_coords)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Car_to_coords.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#Git, tust du noch