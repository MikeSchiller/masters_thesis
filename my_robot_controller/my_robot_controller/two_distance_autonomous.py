#slightly more advanced version of simple_autonomous using both US Sensors

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String

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
#########################################################################
#Winkel 0 == 2.5 Dutycycle
#Winkel 90 == 7.5 Dutycycle
#Winkel 180 == 12.5 Dutycycle
#Lenkservo winkel zwischen 40(rechts), 90(Mittig), 130(links) Grad
#brushless auf WASABI ESC min 50 (5.2PWM) (dauerhafte rotation), max 110 (8,6PWM)
#brushed auf W1060 ESC 2.5PWM(Rückwärts) 7.3-7.4PWM (Stand) 12.5PWM (vorwärts)
#########################################################################

class servo_autonomous(Node):
    def __init__(self):
        super().__init__('servo_autonomous')
        self.get_logger().info('ja moin')
        self.sub_steer = self.create_subscription(String, '/steering', self.steer_callback, 10)
        self.sub_drive = self.create_subscription(String, '/driving', self.drive_callback ,10)
        self.sub_switch = self.create_subscription(String, '/switch', self.switch_callback ,10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/US_distance_links', self.distance_callback_left, 10)
        self.distance_left_subscriber_ = self.create_subscription(String,'/US_distance_rechts', self.distance_callback_right, 10)
        

        
        #self.subscription  # prevent unused variable warning

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

      #self.get_logger().info(dist.data)
      match state:
         
        case 0:
            #normale vorausfahrt
            self.get_logger().info("all is good")
            self.get_logger().info(str(state))
            schub = 7.8
            SetFahrzeugSchub(schub)
            lenkung = 90
            SetServoLenkung(lenkung)
            #mindist= 50
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
                schub = 6.9
            else :
                lenkung = 130
                schub = 6.9

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
            schub = 7.7
            lenkung = 50
            SetFahrzeugSchub(schub)
            SetServoLenkung(lenkung)

            if distance_left < 7.0:
                state = 1
                Rechtslenken = 1

            elif distance_left >  30.0:
                state = 0

        case 20:
            self.get_logger().info(str(state))
        #Wand von Rechts
            self.get_logger().info("OBACHT RECHTS!")
            schub = 7.7
            lenkung = 120
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
def SetServoLenkung(winkel):
  # Umrechnung Grad in Tastverhaeltnis
  if winkel < 40:
    winkel = 40
  if winkel > 130:
    winkel = 130
  pwmL = winkel/18 + 2.5
  LenkServo.ChangeDutyCycle(pwmL)

#Methode fürs einstellen des Schubs
def SetFahrzeugSchub(schub):
  pwmS = schub
  SchubServo.ChangeDutyCycle(pwmS)


def main(args=None):
    rclpy.init(args=args)
    node = servo_autonomous()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

