#This is the brain of the operation

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String
import math
from rpi_hardware_pwm import HardwarePWM


# Servo-GPIO (PWM-GPIO 18, Pin 12)
LenkServoPin = 18
SchubServoPin = 19
# GPIO initialisieren
gpio.setmode(gpio.BCM)
gpio.setup(LenkServoPin, gpio.OUT)
gpio.setup(SchubServoPin, gpio.OUT)
# PWM-Frequenz auf 50 Hz setzen
LenkServo = gpio.PWM(LenkServoPin, 50)
SchubServo = gpio.PWM(SchubServoPin, 50)
# PWM starten, LenkServo auf 90 Grad
LenkServo.start(7.5)
SchubServo.start(7.2)
time.sleep(3)
'''

pwm1 = HardwarePWM(pwm_channel=0, hz= 50)
pwm1.start(7.5)
pwm2 = HardwarePWM(pwm_channel=1, hz= 50)
pwm2.start(7.5)
'''

#initailisierung der Variablen
Winkelstring = String()
pubschub = String()
timer_period = 0.1  # seconds
lenkung = 7.5
schub= 7.2
stopschub =7.2
klenkung = 7.5
kschub = 7.3
notlauf = 0
rechts = 0
Rechtslenken = 0
state = 99
distance_left = 10
distance_right =20 
checkforgo = 0
stopwatsch = 1
start = 0
#Radar
radararrayX = []
radararrayY = []
counttargets = 0
#US
debounceall = 0
debounceleft = 0
debounceright = 0 
actual_longitude = 0.0
actual_latitude = 0.0
HDOP = 0.0
tracked_Heading = 0
cmps_heading = 1000 #initialheading dam
radius_earth = 6371000 #6371km
target_heading = 0
targetdistance = 1000
Zone1 =""
target_longitude = 0
target_latitude = 0
Zoneheading = 1000 # Heading bei Fahrten an NO GO Zone, 1000 wenn nicht in Nähe


#########################################################################
#Winkel 0 == 2.5 Dutycycle
#Winkel 90 == 7.5 Dutycycle
#Winkel 180 == 12.5 Dutycycle
#Lenkservo winkel zwischen 40(rechts), 90(Mittig), 130(links) Grad
#brushless auf WASABI ESC min 50 (5.2PWM) (dauerhafte rotation), max 110 (8,6PWM)
#brushed auf W1060 ESC 2.5PWM(Rückwärts) 7.3-7.4PWM (Stand) 12.5PWM (vorwärts)
#########################################################################

class gps_autonomous(Node):
    def __init__(self):
        super().__init__('servo_autonomous')
        self.get_logger().info('ja moin')
        self.get_logger().info('please make sure the following nodes are running: ')
        self.get_logger().info('keyboard_pub')
        self.get_logger().info('dualUS')
        self.get_logger().info('gps_module')
        self.get_logger().info('gps_data_stuff')
        self.get_logger().info('cmps_pub')
        self.pub_car_schub = self.create_publisher(String, '/car_setschubPWM',10) 
        self.pub_car_steer = self.create_publisher(String, '/car_steer',10) 
        
        #self.sub_actlong = self.create_subscription(String, '/act_longitude', self.act_long_callback, 10)
        #self.sub_actlat = self.create_subscription(String, '/act_latitude', self.act_lat_callback, 10)
        self.sub_actlong = self.create_subscription(String, '/car_long', self.act_long_callback, 10)
        self.sub_actlat = self.create_subscription(String, '/car_lat', self.act_lat_callback, 10)
        self.sub_target_lat = self.create_subscription(String,'target_lat',self.target_lat_callback, 10)
        self.sub_target_long = self.create_subscription(String,'target_long',self.target_long_callback, 10)
        self.sub_hdop = self.create_subscription(String, '/HDOP', self.HDOP_callback, 10)  
        self.sub_head = self.create_subscription(String, '/tracked_heading', self.Heading_callback, 10) # müsste von gps kommen
        #self.sub_cmps = self.create_subscription(String,'/cmps_heading',self.cmps_callback,10)       
        self.sub_cmps = self.create_subscription(String,'headingFromCtC',self.cmps_callback,1) 
        self.sub_steer = self.create_subscription(String, '/steering', self.steer_callback, 10)
        self.sub_drive = self.create_subscription(String, '/driving', self.drive_callback ,10)
        self.sub_switch = self.create_subscription(String, '/switch', self.switch_callback ,10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/US_distance_links', self.distance_callback_left, 1)
        self.distance_rechts_subscriber_ = self.create_subscription(String,'/US_distance_rechts', self.distance_callback_right, 1)
        self.sub_Zone1_ = self.create_subscription(String,'/Zone1', self.saveZone1, 10)
        self.trackcount_sub = self.create_subscription(String,'/Radar_trackcount', self.count_callback, 10)
        self.distance_subscriber_y = self.create_subscription(String,'/Radar_distances_y', self.distance_y_callback, 10)
        self.distance_subscriber_x = self.create_subscription(String,'/Radar_distances_x', self.distance_x_callback, 10)
        self.start_sub = self.create_subscription(String,'start',self.start_callback, 10)
        #self.sub_Zone2 = self.create_subscription(String,'/Zone2', self.saveZone2, 10)        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global Winkelstring
        
        global stopwatsch
        global start 
        end= 0
        if stopwatsch == 1:
            start = time.time()
            stopwatsch= 2
        elif stopwatsch == 2:
            end = time.time()
            print("start: " + str(start))
            print ("end: " + str(end))
            print ("stopwatsch: " +  str(end - start))
            stopwatsch = 1
        '''
        #gemessene Werte im Normbetrieb:
        start: 1679324261.570277
        end: 1679324262.080161
        stopwatsch: 0.5098841190338135

        '''
        print("##################################################################")
        #DAs kann hier so nicht aufgerufen werden
        #self.calculate_heading()
        self.fahrmethode()
        self.pub_car_steer.publish(Winkelstring)
        self.pub_car_schub.publish(pubschub) 


    def start_callback(self,start):
        global state
        if start.data == "go":
            state = 0
        else:
            state = 99
        

    def saveZone1(self, inputstr):
        #!!! Kann das so funkltionieren, oder wird das nur einmal aufgerufen?
        global Zone1
        Zone1 = inputstr
        self.decodeZone1(Zone1)
     
     
    def decodeZone1(self, inputstring):
        global actual_longitude
        global actual_latitude
        global target_latitude
        global target_longitude
        global target_heading
        global Zoneheading
        global Zone1
        bufferdist = 1 #[Einheit noch unklar], warscheinlich Meter

        sd = inputstring.split(",")
        x11 = sd[0]
        y11 = sd[1]
        x12 = sd[2]
        y12 = sd[3]
        x13 = sd[4]
        y13 = sd[5]
        x14 = sd[6]
        y14 = sd[7]
        #https://de.serlo.org/mathe/1785/geradensteigung
        #!!!!Code wird Probleme machen, wenn Punkte genau auf einer Latitude oder Longitude liegen
        #Nördliche Kante
        if y12 - y11 == 0:
            Heading112 = 90
        else:
            #Variablen für Geradengleichung    
            m112 = (x12 - x11) / (y12 - y11)  
            b112 = (m112 * x11) / y11
            #Berechnung des Headings der Gerade
            Heading112 = math.atan(m112)
            #Variable für Orthogonalengleichung
            mo112 = -1/m112
            bo112 = actual_latitude/ (m112 * actual_longitude)
            
        #Östliche Kante
        if x13 - x12 == 0:
            Heading123 = 180
        else:       
            m123 = (y13 - y12) / (x13 - x12)
            b123 = (m123 * x12) / y12
            #Berechnung des Headings der Gerade
            Heading123 = math.atan(m123)
            #Variable für Orthogonalengleichung
            mo123 = -1/m123
            bo123 = actual_latitude/ (m123 * actual_longitude)
        #Südliche Kante
        if y14 - y13 == 0:
            Heading134 = 270
        else:        
            m134 = (x14 - x13) / (y14 - y13) 
            b134 = (m134 * x13) / y13
            #Berechnung des Headings der Gerade
            Heading134 = math.atan(m134)
            #Variable für Orthogonalengleichung
            mo134 = -1/m134
            bo134 = actual_latitude/ (m134 * actual_longitude)
        #Westliche Kante
        if x11 - x14 == 0:
            Heading141 = 0
        else:
            m141 = (y11 - y14) / (x11 - x14)
            b141 = (m141 * x14) / y14
            #Berechnung des Headings der Gerade
            Heading141 = math.atan(m141)
            #Variable für Orthogonalengleichung
            mo141 = -1/m141
            bo141 = actual_latitude/ (m141 * actual_longitude)
        
        #Check whether target is inside NO GO Zone
        #y = mx+b
        if target_longitude < (m112 * target_latitude + b112) and target_longitude > (m112 * target_latitude + b112):
            # x= (y-b)/m IS das so richtig????
            if target_latitude < (target_longitude-b123)/m123 and target_latitude > (target_longitude - b141)/m141:
                TARGETINNOGO = 1
                self.get_logger().error("Target is in NO GO Zone! Please enter new Target Coordinates or adjust NO GO Zone.")
    
        
        #Check whether platform is inside NO GO Zone
        #y = mx+b
        if actual_longitude < (m112 * actual_latitude + b112) and actual_longitude > (m112 * actual_latitude + b112):
            # x= (y-b)/m IS das so richtig????
            if actual_latitude < (actual_longitude-b123)/m123 and actual_latitude > (actual_longitude - b141)/m141:
                ISINNOGO1 = 1               
        
        #Check distances between line an PLatform coords
        #Distance to line 112
        if actual_latitude > x11 and actual_latitude < x12:
            xo112 = (b112 - bo112) / (mo112 - m112) #(mo112 /m112) * (bo112 - b112)
            yo112 = mo112 * xo112  + bo112
            # umrechnung in Meter 
            #distTo112 = math.sqrt((xo112 - actual_latitude)^2 + (yo112 - actual_longitude)^2)  
            distTo112 = math.sqrt((2*radius_earth*math.sin((xo112 - actual_latitude/2)))^2 + (2*radius_earth*math.sin((yo112 - actual_longitude/2)))^2) 
        else:
            distTo112 = 1000    
        #Distance to line 123
        if actual_latitude > y13 and actual_latitude < y12:
            xo123 = (b123 - bo123) / (mo123 - m123) 
            yo123 = mo123 * xo123  + bo123
            distTo123 = math.sqrt((2*radius_earth*math.sin((xo123 - actual_latitude/2)))^2 + (2*radius_earth*math.sin((yo123 - actual_longitude/2)))^2)   
        else:
            distTo123 = 1000
        #Distance to line 134
        if actual_latitude > x14 and actual_latitude < x13:
            xo134 = (b134 - bo134) / (mo134 - m134) 
            yo134 = mo134 * xo134  + bo134
            distTo134 = math.sqrt((2*radius_earth*math.sin((xo134 - actual_latitude/2)))^2 + (2*radius_earth*math.sin((yo134 - actual_longitude/2)))^2)   
        else:
            distTo134 = 1000
        #Distance to line 141
        if actual_latitude > y14 and actual_latitude < y11:
            xo141 = (b141 - bo141) / (mo141 - m141) 
            yo141 = mo141 * xo141  + bo141
            distTo141 = math.sqrt((2*radius_earth*math.sin((xo141 - actual_latitude/2)))^2 + (2*radius_earth*math.sin((yo141 - actual_longitude/2)))^2)   
        else:
            distTo141 = 1000
        #mindist Einheit????
        #Nächster SChritt hier, schauen welche distanz am niedrigsten ist und aus steigung m neues Heading bestimmen.
        distarray = [distTo112, distTo123, distTo134, distTo141]
        mindist= min(distarray)
        pos_min = distarray.index(mindist)

        directionset = 0 #Variable, damit zu beginn einmal die Richtung festgelegt wird und sich erst wieder resettet, wenn Plattform aud Risikozone raus ist

        if pos_min == 0: #Line 112
            if mindist < bufferdist :
                if actual_latitude > target_latitude and directionset == 0:
                    Zoneheading = Heading112 + 180
                    directionset = 1
                else: 
                    Zoneheading = Heading112
                    directionset = 1
                
        elif pos_min == 0: #Line 123
            if mindist < bufferdist :
                if actual_longitude < target_longitude and directionset == 0:
                    Zoneheading = Heading123 + 180
                    directionset = 1
                else: 
                    Zoneheading = Heading123

        elif pos_min == 0: #Line 134
            if mindist < bufferdist:
                if actual_latitude > target_latitude and directionset == 0:
                    Zoneheading = Heading134 + 180
                    directionset = 1
                else: 
                    Zoneheading = Heading134
                    directionset = 1

        elif pos_min == 0: #Line 141
            if mindist < bufferdist :
                if actual_longitude > target_longitude and tempset == 0:
                    Zoneheading = Heading141 + 180
                    tempset = 1
                else: 
                    Zoneheading = Heading141


        else:
            Zoneheading = 1360 # Wert der ausgegeben wird, wenn Plattform nicht in der Nähe von NO GO Zone ist.
            directionset = 0

        if Zoneheading >= 360:
            Zoneheading = Zoneheading - 360

        #!!!! Code damit Plattform aus Zone rausfährt.
        
        return Zoneheading
                

    #get the actual gps data in degrees
    def act_long_callback(self, act_long):
        global actual_longitude
        actual_longitude = float(act_long.data)

     #get the actual gps data in degrees   
    def act_lat_callback(self, act_lat):
        global actual_latitude
        actual_latitude = float(act_lat.data)


    def HDOP_callback(self, hdop):
        global HDOP
        if hdop.data == '':
            pass
        else:
            HDOP = float(hdop.data)

    def Heading_callback(self, head):
        global tracked_Heading
        tracked_Heading = float(head.data)
        #self.calculate_heading()

    def cmps_callback(self,cmps_head):
        global cmps_heading
        cmps_heading = float(cmps_head.data)
        self.calculate_heading()
        
    def distance_callback_left(self,dist_left):
        global distance_left
        distance_left= float(dist_left.data)


    #Hier Aufruf der Fahrmethode
    def distance_callback_right(self,dist_right):
        global distance_right
        distance_right= float(dist_right.data)
        #Aufruf macht hier keinen sinn mehr
        #self.fahrmethode(self)
        
                    
    def drive_callback(self, msg):
        global kschub 
        kschub = float(msg.data)
        
    def steer_callback(self, msg):
        global klenkung 
        klenkung = float(msg.data)

    def switch_callback(self, msg):
        global state


        if msg.data == "on":
            state = 30
        else:
            state = 0
        #self.fahrmethode(self)

    def target_lat_callback(self, tlat):
        global target_latitude
        target_latitude = tlat.data
        #print("targetLat: " + target_latitude)

    def target_long_callback(self,tlong):
        global target_longitude
        target_longitude = tlong.data
        #print ("targetLong: " + target_longitude)


    #Methoden der Radar subscriber

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

    ##################################################################
    #Aufgerufen von HEading callback    
    def calculate_heading(self):
        global actual_latitude
        global actual_longitude
        global HDOP
        global tracked_Heading
        global radius_earth
        global target_heading 
        global target_longitude
        global target_latitude 
        global targetdistance
        target_heading = 0 # zu beginn der Methode null setzen, damit frisch ausgerechnet wird und Wert anschließend für andere Methoden verfügbar
        quadrantlat = 1
        quadrantlong = 1

        #!!!!! Hier ist Fehler drin
        #Distanz zwischen beiden Punkten berechnen. Dafür Erde = Kugel annahme, für kurze Distanzen ausreichend
        # CAVE: Code aktuell Für Nord östliche Bereiche der Welt ausgelegt.
        #https://www.sunearthtools.com/de/tools/distance.php#:~:text=Berechnung%20der%20Entfernung%20zwischen%20zwei%20geografischen%20Punkten,-Die%20Formel%20verwendet&text=Das%20Winkeln%20eingesetzt%20werden%20in,pi%20dividiert%20durch%20180%20erhalten.
        print ("target_lat: " + str(target_latitude))
        print ("target_long: " + str(target_longitude))
        print("act_lat: " + str(actual_latitude))
        print("act_long: " + str(actual_longitude))
        if target_latitude == "" or target_longitude == "":
            pass
        else:
            difference_lat = float(target_latitude) -  float(actual_latitude )
            difference_long = float(target_longitude) - float(actual_longitude)
            
            if difference_lat < 0:
                difference_lat = difference_lat * -1
                #Abfrage, in welchem Quadranten sich das Target vom aktuellen Punkt befindet
                #quadrantlat = 34 # Quadrant 3 oder 4
                quadrantlat = 23
            else: 
                pass
            
            
            if difference_long < 0:
                difference_long = difference_long * -1
                quadrantlong = 34
            else: 
                pass
            # Genauigkeit https://www.sunearthtools.com/dp/tools/pos_earth.php?lang=de#txtEarth_6
            # Aproxximierung der Strecke über Dreieck (Dürfte bei den Distanzen keinen Nennenswerten unterschied machen)
            if difference_long !=0:
                distance_long = 2 * radius_earth * math.sin(math.radians(difference_long/ 2)) 
            else:
                distance_long =  0

            if difference_lat != 0:
                distance_lat = 2 * radius_earth * math.sin(math.radians(difference_lat / 2)) #sin
            else:
                distance_lat = 0
            print("dist_long: " + str(distance_long)) #[m]
            print("dist_lat: " + str(distance_lat)) 
            #distanzen sehen jetzt gut aus.
            targetdistance = math.sqrt(distance_lat * distance_lat + distance_long * distance_long)

            #berechnung des Winkels
            if distance_long != 0 and distance_lat != 0:
                
                if quadrantlong == 1 and quadrantlat== 1:
                    #Quadrant 1
                    target_heading = 90 - math.degrees(math.atan(distance_lat / distance_long))
                elif quadrantlong == 1 and quadrantlat== 23:
                    #Quadrant 2
                    target_heading = 180 - math.degrees(math.atan(distance_long / distance_lat))                    
                elif quadrantlong == 34 and quadrantlat== 23:
                    #Quadrant 3
                    target_heading = 270 - math.degrees(math.atan(distance_lat / distance_long))
                elif quadrantlong == 34 and quadrantlat== 1:
                    #Quadrant 4
                    target_heading = 360 - math.degrees(math.atan(distance_long / distance_lat))




            else:
                print("gleich mein Problem")
            if target_heading < 0:
                target_heading = target_heading + 360
            elif target_heading >= 360:
                target_heading = target_heading - 360
            #target_heading = target_heading + 180 #!!!debug, wert passt hier allggemein noch nicht
            ############################################################
        
           # target_heading = 40
        
            ##############################################################
            print ('target Heading: ' + str(target_heading))


        



    def fahrmethode(self):
      #Färht geradaus  mit zwei Distanzsensor richtung hindernis, versucht Hindernis auszuweichen, wenn zu nah weicht rückwärts aus
      global distance_left
      global distance_right
      global schub
      global notlauf
      global rechts
      global lenkung
      global Rechtslenken
      global state
      global kschub
      global klenkung 
      global target_heading
      global tracked_Heading
      global cmps_heading
      global stopschub
      global Zoneheading
      global actual_latitude
      global actual_longitude
      global target_latitude
      global target_longitude
      global targetdistance
      #Radar
      global counttargets
      global radararrayX
      global radararrayY
      global debounceall 
      global debounceleft 
      global debounceright  
      global checkforgo
      mindist = 100.0 

    #Abfrage, ob seehr nah am Ziel
    
       #    dec.deg     Grad            Entfernung
        #	0.1	        0°6’0"	        11.132 km
        #	0.01    	0°0’36"	        1.113 km
        #	0.001	    0°0’3.6"	    111.3 m
        #	0.0001  	0°0’0.36"	    11.13 m
        #	0.00001  	0°0’0.036"	    1.11 m
        #	0.000001	0°0’0.0036"	    11.1 cm
        #	0.0000001	0°0’0.00036"	1.11 cm
      #print (target_latitude)
      #print (target_longitude)
      #print(actual_latitude)
      #print(actual_longitude)
      print("targetdistance: " + str(targetdistance))
      if targetdistance <= 5 and float(target_latitude) != 0:
          state = 100
    # Abfrage, ob koordianten ungleich 0N, 0E
      elif actual_latitude == 0 and actual_longitude == 0 and state != 99:
          state = 98 
      elif actual_latitude != 0 and actual_longitude != 0 and state == 98 and checkforgo == 0:
            state = 0
            checkforgo = 1
        
      

      #self.get_logger().info(dist.data)
      match state:
         
        case 0:
            #während case 0 fährt das Fahrzeug in Richtung Ziel
            #normale vorausfahrt
            self.get_logger().info("all is good")
            self.get_logger().info(str(state))
            schub = 7.6
            
            #Abfrage, ob in Nähe von NO GO Zone und änderung des Headings, wenn ja
            if Zoneheading != 1000:
                self.get_logger().info("Close to NO GO Zone!")
                target_heading = Zoneheading


            #Auswertung Heading Kompass
            max_all_dev = 10 # maximum allowed deviation from target heading
            smallsteerdev = 30 #deviation for smaller steering angle to counter oversteer
            #!!!!Das tut nicht wies soll
            print("cmps_head: " + str(cmps_heading))
            print("target_head: " + str(target_heading))
            '''
            if cmps_heading < target_heading - smallsteerdev:
                lenkung = 100
            elif cmps_heading > target_heading - smallsteerdev and cmps_heading < target_heading - max_all_dev:
                lenkung = 95
            elif cmps_heading < target_heading + smallsteerdev and cmps_heading > target_heading + max_all_dev:
                lenkung = 85
            elif cmps_heading > target_heading + smallsteerdev:
                lenkung = 80
            else:
                Lenkung = 90

            ''#'
            if cmps_heading < target_heading + max_all_dev or cmps_heading < target_heading - max_all_dev :
                lenkung = 85
            elif cmps_heading < target_heading + smallsteerdev or cmps_heading < target_heading - smallsteerdev:
                lenkung = 80            
            elif cmps_heading > target_heading + max_all_dev or cmps_heading > target_heading - max_all_dev:
                lenkung = 95
            elif cmps_heading > target_heading + smallsteerdev or cmps_heading > target_heading - smallsteerdev:
                lenkung = 100
            else:
                lenkung = 90   
            '''
            #verhindern, dass nulldurchlauf passieren kann 
            cmps_heading = cmps_heading +500
            target_heading = target_heading +500
            #V3, looks a lot better now
            if cmps_heading < target_heading - max_all_dev :
                #große Abweichung links
                lenkung = 80
            elif cmps_heading > target_heading - max_all_dev and cmps_heading < target_heading - smallsteerdev:
                #kleine Abweichung links
                lenkung = 85 
            elif cmps_heading < target_heading + max_all_dev and cmps_heading > target_heading + smallsteerdev:
                #kleine Abweichung rechts
                lenkung = 95     
            elif cmps_heading > target_heading + max_all_dev :
                #große Abweichung re
                lenkung = 100
            else: 
                lenkung = 90

            
            cmps_heading = cmps_heading -500
            target_heading = target_heading -500  
            print("lenkung: " + str(lenkung))                          


            ''' Auswertung Heading GPS Modul         
            if tracked_Heading < target_heading + 5 or tracked_Heading < target_heading - 5 :
                lenkung = 70
            elif tracked_Heading > target_heading +5 or tracked_Heading > target_heading -5:
                lenkung = 110
            else:
                lenkung = 90
            '''

            SetServoLenkung(self , lenkung)
            SetFahrzeugSchub(self , schub)
            '''
            # Implementierung einer einfachen Auswertung der Radar Daten
            minddist_radar_x = 0.5 #[m]
            minddist_radar_y = 0.4 #[m]
            for radvar in range(0, counttargets):
                #Schauen, ob sich ein Hindernis links/rechts von Fahrzeugmitte aus befindet und maximal 1 Meter weit weg ist
                

                if radararrayX == [] or radararrayY == []:
                    pass
                else:
                    if radararrayX[radvar] < minddist_radar_x and radararrayY[radvar] < 1 :
                    
                        #check ob links vom Fahrzeug
                        #links
                        if radararrayY[radvar] < 0:
                            state = 50
                        elif radararrayY[radvar] > 0:
                            state = 60
                    #schauen, ob ein Hindernis direkt vor dem Sensor ist
                    # Distanz von Fahrzeug
                    elif abs(radararrayY[radvar]) < minddist_radar_y :
                        state = 40
                
            '''        


            #Am ende der auswertung müssen die Radar arrays geleert werden
            radararrayX = []
            radararrayY = []
            
                        #check distances from US sensors and act accordingly 
            if distance_left < mindist and distance_right > mindist and notlauf == 0  :
                    state = 10
            elif distance_right < mindist and distance_left > mindist and notlauf == 0 :
                    state = 20
            elif distance_left < mindist and distance_right < mindist and notlauf == 0 :
                    state =1
            
            # including debouncing 
            #print(distance_left)
            #print(distance_right)
            '''
            if distance_left < mindist and distance_right > mindist and notlauf == 0  :
                debounceleft = debounceleft +1
                if debounceleft >= 2:
                    state = 10
                    debounceleft = 0
            elif distance_right < mindist and distance_left > mindist and notlauf == 0 :
                debounceright += 1
                if debounceright >= 2:
                    state = 20
                    debounceright = 0
            elif distance_left < mindist and distance_right < mindist and notlauf == 0 :
                debounceall += 1
                if debounceall >= 2:
                    debounceall = 0
                    state =1
            '''


           # print(debounceall)
            #print(debounceleft)
           # print(debounceright)
                



        case 1:
            self.get_logger().info(str(state))
            #geradeaus gegen Wand
            #Fahrzeug stoppt
            self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            schub = stopschub
            SetFahrzeugSchub(self, schub)
            notlauf = 1
            if distance_left < mindist or distance_right < mindist and notlauf == 1:
                state =2

        case 2:
            #ehemals gelenkt zurück 
            #aktuell:geradeaus rückwärts, dann gelenkt vorwärts
            self.get_logger().info(str(state))
            self.get_logger().info("Notlauf")
            

            schub = 6.9
            SetFahrzeugSchub(self, schub)
            SetServoLenkung(self, lenkung)
            
            if  distance_left > mindist and distance_right > mindist and notlauf == 1:
                state =3
                
        case 3:
            self.get_logger().info(str(state))
            self.get_logger().info("post Notlauf ")
            if Rechtslenken == 1:
                lenkung = 50
                schub = 7.6
            else:
                lenkung = 120
                schub = 7.6
            SetFahrzeugSchub(self, schub)
            notlauf = 0
            state = 0

        case 4:
            self.get_logger().info(str(state))
      
            self.get_logger().info("Notlauf beednet")
            schub = 7.3
            SetFahrzeugSchub(self, schub)
            notlauf = 0
            state = 0




        case 10:
            self.get_logger().info(str(state))
        # Wand von links
            self.get_logger().info("OBACHT LINKS!")
            schub = 7.65
            lenkung = 60
            SetFahrzeugSchub(self, schub)
            SetServoLenkung(self, lenkung)

            if distance_left < 15.0:
                state = 1
                Rechtslenken = 1

            elif distance_left >  mindist:
                state = 0

        case 20:
            self.get_logger().info(str(state))
        #Wand von Rechts
            self.get_logger().info("OBACHT RECHTS!")
            schub = 7.65
            lenkung = 120
            SetFahrzeugSchub(self, schub)
            SetServoLenkung(self, lenkung)

            if distance_right < 15.0:
                state = 1
                Rechtslenken = 0

            elif distance_right >  mindist:
                state = 0

        case 30:
            self.get_logger().info(str(state))
        #Keyboard input
            self.get_logger().info("manual drive")
            SetServoLenkung(self, klenkung)
            SetFahrzeugSchub(self, kschub)

        case 40:
              #Radar front nah Hindernis
              schub = 7.3

        case 50:
              #Radar links Hindernis
            self.get_logger().info("RADAR LINKS!")
            schub = 7.65
            lenkung = 50
            SetFahrzeugSchub(self, schub)
            SetServoLenkung(self, lenkung)

            if abs(min(radararrayX)) < 0.4:
                state = 40
                Rechtslenken = 1

            elif abs(min(radararrayX)) >  0.4:
                state = 0

        case 60:
              #Radar rechts Hindernis
            self.get_logger().info("RADAR RECHTS!")
            schub = 7.65
            lenkung = 120
            SetFahrzeugSchub(self, schub)
            SetServoLenkung(self, lenkung)

            if abs(min(radararrayX)) < 0.4:
                state = 40
                Rechtslenken = 1

            elif abs(min(radararrayX)) >  0.4:
                state = 0

        case 98:
              print("Warte aufs GPS Signal")
        case 99:
              print("Warte aufs GO!")     

        case 100:
              schub = stopschub
              SetFahrzeugSchub(self, schub)
              print("DA SIMMA!!!")


        



        


#Methode fürs einstellen der Lenkung
def SetServoLenkung(self, winkel):
  global Winkelstring
  # Umrechnung Grad in Tastverhaeltnis
  if winkel < 40:
    winkel = 40
  if winkel > 130:
    winkel = 130
  pwmL = winkel/18 + 2.5
  #pwm1.change_duty_cycle(pwmL)
  LenkServo.ChangeDutyCycle(pwmL)
  Winkelstring.data = str(winkel)
  #self.pub_car_steer.publish(Winkelstring) #WARUM WIRST DU NICHT FARBIG ?????

#Methode fürs einstellen des Schubs
def SetFahrzeugSchub(self,schub):
    global pubschub
    tempvar = 1
    if tempvar == 1:
        pwmold = 7.3
        tempvar = 2

    stopfahr = 7.3
    pwmS = schub
    #Methode für langsames anfahren und verhindern von wheelie, wobei bremsung aber erhalten
    if pwmS > pwmold and pwmS != stopfahr:
        i= 1
        #WARUM GEHT DER HIER JEDE ITERATION REIN????????????????????????
        while i <= 5 :
            sendschub = float(i)* ((pwmS - pwmold) / 5) + pwmold
            #time.sleep(0.1)
            i += 1
     
    # macht langsamer werden Sinn oder will ich da    
    elif pwmS < pwmold and pwmS != stopfahr:
        i = 1
        while i<=5:  
            sendschub = pwmold - ( float(i) * (pwmold - pwmS) / 5 )
            #time.sleep(0.1)
            i += 1

    else: 
        sendschub = schub

    
    pubschub.data = str(schub)   
    #pwm2.change_duty_cycle(sendschub)
    SchubServo.ChangeDutyCycle(sendschub)
    pwmold = schub


def main(args=None):
    rclpy.init(args=args)
    node = gps_autonomous()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

