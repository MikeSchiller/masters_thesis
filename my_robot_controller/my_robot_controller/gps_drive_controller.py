#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String
import math

# Servo-GPIO (PWM-GPIO 18, Pin 12)
LenkServoPin = 18
SchubServoPin = 13
# GPIO initialisieren
gpio.setmode(gpio.BCM)
gpio.setup(LenkServoPin, gpio.OUT)
gpio.setup(SchubServoPin, gpio.OUT)
# PWM-Frequenz auf 50 Hz setzen
LenkServo = gpio.PWM(LenkServoPin, 50)
SchubServo = gpio.PWM(SchubServoPin, 50)
# PWM starten, LenkServo auf 90 Grad
LenkServo.start(7.5)
SchubServo.start(7.3)
time.sleep(3)


#initailisierung der Variablen
lenkung = 7.5
schub= 7.3
klenkung = 7.5
kschub = 7.3
notlauf = 0
rechts = 0
Rechtslenken = 0
state = 0
distance_left = 0
distance_right =0 
actual_longitude = 0.0
actual_latitude = 0.0
HDOP = 0.0
tracked_Heading = 0
cmps_heading = 0
radius_earth = 6371000 #6371km
target_heading = 0
x11 = 0
y11 = 0
x12 = 0
y12 = 0
x13 = 0
y13 = 0
x14 = 0
y14 = 0
x21 = 0
y21 = 0
x22 = 0
y22 = 0
x23 = 0
y23 = 0
x24 = 0
y24 = 0
m112 = 0
m123 = 0
m134 = 0
m141 = 0
b123 = 0
b112 = 0
b134 = 0
b141 = 0
target_longitude = 9.939772 # Kreuzung E-Radstellplatz
target_latitude = 48.418074 # Kreuzung E-Radstellplatz
#target_longitude = 9.937939 # Kreuzung V-Bau
#target_latitude = 48.418091 # Kreuzung V-Bau   
#target_longitude = 9.937964 # Ecke gemähte Wiese Osten
#target_latitude = 48.417796 # Ecke gemähte Wiese Osten     
#target_longitude = 9.940302 # Kreuzung Zufahrt THU Höhenweg westen
#target_latitude = 48.417305 # Kreuzung Zufahrt THU Höhenweg westen
#target_longitude = 9.917946 # bad blau
#target_latitude = 48.417771 # bad blau
#home_adress = "(48.411954, 10.002206)"

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
        self.sub_actlong = self.create_subscription(String, '/act_longitude', self.act_long_callback, 10)
        self.sub_actlat = self.create_subscription(String, '/act_latitude', self.act_lat_callback, 10)
        self.sub_hdop = self.create_subscription(String, '/HDOP', self.HDOP_callback, 10)  
        self.sub_head = self.create_subscription(String, '/tracked_heading', self.Heading_callback, 10) 
        self.sub_cmps = self.create_subscription(String,'/cmps_heading',self.cmps_callback,10)       
        self.sub_steer = self.create_subscription(String, '/steering', self.steer_callback, 10)
        self.sub_drive = self.create_subscription(String, '/driving', self.drive_callback ,10)
        self.sub_switch = self.create_subscription(String, '/switch', self.switch_callback ,10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/distance_links', self.distance_callback_left, 10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/distance_rechts', self.distance_callback_right, 10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/Zone1', self.decodeZone1, 10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/Zone2', self.decodeZone2, 10)        
        

     
     
     
    def decodeZone1(self, inputstring):
        global x11 
        global y11 
        global x12 
        global y12 
        global x13 
        global y13 
        global x14 
        global y14 
        global m112
        global m123
        global m134
        global m141
        global b112
        global b123
        global b134
        global b141
        global actual_longitude
        global actual_latitude
        global target_latitude
        global target_longitude

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
        #Nördliche Kante
        if y12 - y11 == 0:
            winkel12 = 90
        else:
            #Variablen für Geradengleichung    
            m112 = (x12 - x11) / (y12 - y11)  
            b112 = (m112 * x11) / y11
            #Variable für Orthogonalengleichung
            mo112 = -1/m112
            bo112 = actual_latitude/ (m112 * actual_longitude)
            
        #Östliche Kante
        if x13 - x12 == 0:
            winkel23 = 180
        else:       
            m123 = (y13 - y12) / (x13 - x12)
            b123 = (m123 * x12) / y12
            #Variable für Orthogonalengleichung
            mo123 = -1/m123
            bo123 = actual_latitude/ (m123 * actual_longitude)
        #Südliche Kante
        if y14 - y13 == 0:
            winkel34 = 270
        else:        
            m134 = (x14 - x13) / (y14 - y13) 
            b134 = (m134 * x13) / y13
            #Variable für Orthogonalengleichung
            mo134 = -1/m134
            bo134 = actual_latitude/ (m134 * actual_longitude)
        #Westliche Kante
        if x11 - x14 == 0:
            winkel41 = 0
        else:
            m141 = (y11 - y14) / (x11 - x14)
            b141 = (m141 * x14) / y14
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
            distTo112 = math.sqrt((xo112 - actual_latitude)^2 + (yo112 - actual_longitude)^2)      
        #Distance to line 123
        if actual_latitude > y13 and actual_latitude < y12:
            xo123 = (b123 - bo123) / (mo123 - m123) 
            yo123 = mo123 * xo123  + bo123
            distTo123 = math.sqrt((xo123 - actual_latitude)^2 + (yo123 - actual_longitude)^2)  
        #Distance to line 134
        if actual_latitude > x14 and actual_latitude < x13:
            xo134 = (b134 - bo134) / (mo134 - m134) 
            yo134 = mo134 * xo134  + bo134
            distTo134 = math.sqrt((xo134 - actual_latitude)^2 + (yo134 - actual_longitude)^2)  
        #Distance to line 141
        if actual_latitude > y14 and actual_latitude < y11:
            xo141 = (b141 - bo141) / (mo141 - m141) 
            yo141 = mo141 * xo141  + bo141
            distTo141 = math.sqrt((xo141 - actual_latitude)^2 + (yo141 - actual_longitude)^2)  
        
        
        

    def decodeZone2(self, inputstring):
        global x21 
        global y21 
        global x22 
        global y22 
        global x23 
        global y23 
        global x24 
        global y24 
        sd = inputstring.split(",")
        x21 = sd[0]
        y21 = sd[1]
        x22 = sd[2]
        y22 = sd[3]
        x23 = sd[4]
        y23 = sd[5]
        x24 = sd[6]
        y24 = sd[7]   
        
        
        #self.subscription  # prevent unused variable warning
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
        HDOP = float(hdop.data)

    def Heading_callback(self, head):
        global tracked_Heading
        tracked_Heading = float(head.data)
        self.calculate_heading()

    def cmps_callback(self,cmps_head):
        global cmps_heading
        cmps_heading = float(cmps_head.data)
   
        
    def distance_callback_left(self,dist_left):
        global distance_left
        distance_left= float(dist_left.data)

    def distance_callback_right(self,dist_right):
        global distance_left
        global distance_right
        distance_right= float(dist_right.data)
        self.fahrmethode(distance_left,distance_right)
        
                    
    def drive_callback(self, msg):
        global kschub 
        kschub = float(msg.data)
        
    def steer_callback(self, msg):
        global klenkung 
        klenkung = float(msg.data)

    def switch_callback(self, msg):
        global state
        global distance_left
        global distance_right
        if msg.data == "on":
            state = 30
        else:
            state = 0
        self.fahrmethode(distance_left,distance_right)


    ##################################################################
        
    def calculate_heading(self):
        global actual_latitude
        global actual_longitude
        global HDOP
        global tracked_Heading
        global radius_earth
        global target_heading 
        global target_longitude
        global target_latitude 


        #Distanz zwischen beiden Punkten berechnen. Dafür Erde = Kugel annahme, für kurze Distanzen ausreichend
        # CAVE: Code aktuell Für Nord östliche Bereiche der Welt ausgelegt.
        #https://www.sunearthtools.com/de/tools/distance.php#:~:text=Berechnung%20der%20Entfernung%20zwischen%20zwei%20geografischen%20Punkten,-Die%20Formel%20verwendet&text=Das%20Winkeln%20eingesetzt%20werden%20in,pi%20dividiert%20durch%20180%20erhalten.
        difference_lat =target_latitude -  actual_latitude 
        difference_long = target_longitude - actual_longitude
        
        if difference_lat < 0:
            difference_lat = difference_lat * -1
            target_heading = target_heading +90
        else: 
            pass
        
        
        if difference_long < 0:
            difference_long = difference_long * -1
            target_heading = target_heading + 180
        else: 
            pass
        # Genauigkeit https://www.sunearthtools.com/dp/tools/pos_earth.php?lang=de#txtEarth_6
        # Aproxximierung der Strecke über Dreieck (Dürfte bei den Distanzen keinen Nennenswerten unterschied machen)
        distance_long = 2 * radius_earth * math.sin(difference_long / 2)
        distance_lat = 2 * radius_earth * math.sin(difference_lat / 2)
        print(distance_long) #sollte in m sein
        print(distance_lat)

        #berechnung des Winkels
        target_heading = target_heading + math.tan(distance_long / distance_lat)
        if target_heading < 0:
            target_heading = target_heading + 360
        print ('target Heading: ' + str(target_heading))
        target_heading = 0 #Warum wird hier NuLL gesetzt ????



    def fahrmethode(self, distance_left, distance_right):
      #Färht geradaus  mit zwei Distanzsensor richtung hindernis, versucht Hindernis auszuweichen, wenn zu nah weicht rückwärts aus
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

      #self.get_logger().info(dist.data)
      match state:
         
        case 0:
            #normale vorausfahrt
            self.get_logger().info("all is good")
            self.get_logger().info(str(state))
            schub = 8.8
            
            #Auswertung Heading Kompass
            max_all_dev = 3 # maximum allowed deviation from target heading
            if cmps_heading < target_heading + max_all_dev or cmps_heading < target_heading - max_all_dev :
                lenkung = 70
            elif cmps_heading > target_heading + max_all_dev or cmps_heading > target_heading - max_all_dev:
                lenkung = 110
            else:
                lenkung = 90   

            ''' Auswertung Heading GPS Modul         
            if tracked_Heading < target_heading + 5 or tracked_Heading < target_heading - 5 :
                lenkung = 70
            elif tracked_Heading > target_heading +5 or tracked_Heading > target_heading -5:
                lenkung = 110
            else:
                lenkung = 90
            '''

            SetServoLenkung(lenkung)
            SetFahrzeugSchub(schub)
            
            #check distances from US sensors and act accordingly    
            if distance_left < 25.0 and distance_right > 25.0 and notlauf == 0  :
                state = 10
            elif distance_right < 25.0 and distance_left > 25.0 and notlauf == 0 :
                state = 20
            elif distance_left < 25.0 and distance_right < 25.0 and notlauf == 0 :
                state =1
                


        case 1:
            self.get_logger().info(str(state))
            #geradeaus gegen Wand
            self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            schub = 7.3
            SetFahrzeugSchub(schub)
            notlauf = 1
            if distance_left < 50.0 or distance_right < 50.0 and notlauf == 1:
                state =2

        case 2:
            self.get_logger().info(str(state))
            self.get_logger().info("Notlauf")
            if Rechtslenken == 1:
                lenkung = 40
                schub = 6.6
            else :
                lenkung = 130
                schub = 6.6

            SetFahrzeugSchub(schub)
            SetServoLenkung(lenkung)
            
            if  distance_left > 50.0 and distance_right > 50.0 and notlauf == 1:
                state =3

        case 3:
            self.get_logger().info(str(state))
      
            self.get_logger().info("Notlauf beednet")
            schub = 7.3
            SetFahrzeugSchub(schub)
            notlauf = 0
            state = 0




        case 10:
            self.get_logger().info(str(state))
        # Wand von links
            self.get_logger().info("OBACHT LINKS!")
            schub = 7.65
            lenkung = 130
            SetFahrzeugSchub(schub)
            SetServoLenkung(self, lenkung)

            if distance_left < 7.0:
                state = 1
                Rechtslenken = 1

            elif distance_left >  30.0:
                state = 0

        case 20:
            self.get_logger().info(str(state))
        #Wand von Rechts
            self.get_logger().info("OBACHT RECHTS!")
            schub = 7.65
            lenkung = 40
            SetFahrzeugSchub(schub)
            SetServoLenkung(lenkung)

            if distance_right < 7.0:
                state = 1
                Rechtslenken = 0

            elif distance_right >  30.0:
                state = 0

        case 30:
            self.get_logger().info(str(state))
        #Keyboard input
            self.get_logger().info("manual drive")
            SetServoLenkung(klenkung)
            SetFahrzeugSchub(kschub)

        



        


#Methode fürs einstellen der Lenkung
def SetServoLenkung(self, winkel):
  Winkelstring =String()
  # Umrechnung Grad in Tastverhaeltnis
  if winkel < 40:
    winkel = 40
  if winkel > 130:
    winkel = 130
  pwmL = winkel/18 + 2.5
  LenkServo.ChangeDutyCycle(pwmL)
  Winkelstring.data = winkel
  self.pub_car_steer.publish(Winkelstring) #WARUM WIRST DU NICHT FARBIG ?????

#Methode fürs einstellen des Schubs
def SetFahrzeugSchub(schub):
  pwmS = schub
  self.pub_car_speed.publish(str(schub))
  SchubServo.ChangeDutyCycle(pwmS)


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

