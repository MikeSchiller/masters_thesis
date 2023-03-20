#node for simple autonomous driving without any target

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio
import time
from std_msgs.msg import String

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
notlauf = 0
rechts = 0
zuletztRechts = 0
state = 0
#########################################################################
#Winkel 0 == 2.5 Dutycycle
#Winkel 90 == 7.5 Dutycycle
#Winkel 180 == 12.5 Dutycycle
#Lenkservo winkel zwischen 40(rechts), 90(Mittig), 130(links) Grad
#bruschless auf WASABI ESC min 50 (5.2PWM) (dauerhafte rotation), max 110 (8,6PWM)
#brushed auf W1060 ESC 2.5PWM(Rückwärts) 7.3-7.4PWM (Stand) 12.5PWM (vorwärts)
#########################################################################

class servo_autonomous(Node):
    def __init__(self):
        super().__init__('servo_autonomous')
        self.get_logger().info('ja moin')
        #self.subscription = self.create_subscription(String'topic',self.listener_callback,10)
        self.distance_subscriber_ = self.create_subscription(String,'/distance', self.distance_callback, 10)
        #self.subscription  # prevent unused variable warning


    def distance_callback(self, dist):
      #Färht geradaus bis hindernis, weicht rückwärts aus
      global schub
      global notlauf
      global rechts
      global lenkung
      global zuletztRechts
      global state 
      i = 0
      #self.get_logger().info(dist.data)
      match state:
         
        case 0:
            self.get_logger().info("all is good")
            schub = 7.8
            SetFahrzeugSchub(schub)
            lenkung = 90
            SetServoLenkung(lenkung)
            if float(dist.data) < 10.0 and notlauf == 0:
                state = 1
      
        case 1:
      
            self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            schub = 7.3
            SetFahrzeugSchub(schub)
            notlauf = 1
      
            if float(dist.data) < 50.0 and notlauf == 1:
                state =2

        case 2:
            self.get_logger().info("Notlauf")
            if zuletztRechts == 0:
                lenkung = 40
                rechts = 1
                schub = 6.6
            else :
                lenkung = 130
                schub = 6.6

            SetFahrzeugSchub(schub)
            SetServoLenkung(lenkung)
            if  float(dist.data) > 50.0 and notlauf == 1:
                state =3

        case 3:
      
            self.get_logger().info("Notlauf beednet")
            schub = 7.3
            SetFahrzeugSchub(schub)
            notlauf = 0
            state = 4
            if rechts == 1:
                zuletztRechts = 1
                rechts = 0
                    
            else:
                zuletztRechts = 0

        case 4:
            while i < 20:

                #if float(dist.data) < 10.0:
                 #   self.get_logger().info("NotlAAAAAAAAAAAA")
                  #  state = 1
                   # break
                #Unschön: Hier wird ne Sekunde lang nicht die Distanz nicht überwacht

                
                if zuletztRechts == 1:
                    lenkung = 130
                    schub = 7.8
                else:
                    lenkung = 40
                    schub = 7.8

                self.get_logger().info("Nach Notlauf")
                SetServoLenkung(lenkung)
                SetFahrzeugSchub(schub)
                time.sleep(0.1)
                i+=1
                    
            i=0
            state = 0


        



        


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

