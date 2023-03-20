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
lenkung = 7.5
schub= 7.3
notlauf = 0




####
#Winkel 0 == 2.5 Dutycycle
#Winkel 90 == 7.5 Dutycycle
#Winkel 180 == 12.5 Dutycycle
#Lenkservo winkel zwischen 40(rechts), 90(Mittig), 130(links) Grad
#bruschless auf WASABI ESC min 50 (5.2PWM) (dauerhafte rotation), max 110 (8,6PWM)
#brushed auf W1060 ESC 2.5PWM(Rückwärts) 7.3-7.4PWM (Stand) 12.5PWM (vorwärts)

class servosub(Node):
    def __init__(self):
        super().__init__('servo_sub')
        self.get_logger().info('ja moinsen')
        self.sub_steer = self.create_subscription(String, '/steering', self.steer_callback, 10)
        self.sub_drive = self.create_subscription(String, '/driving', self.drive_callback ,10)
        #self.distance_subscriber_ = self.create_subscription(String,'/distance', self.distance_callback, 10)
        #self.subscription  # prevent unused variable warning

    """
    def distance_callback(self, dist):
      global schub
      global notlauf
      #self.get_logger().info(dist.data)
      if float(dist.data) < 10.0 and notlauf == 0:
        self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        schub = 7.3
        SetFahrzeugSchub(schub)
        notlauf = 1

      elif float(dist.data) < 30.0 and notlauf == 1:
        self.get_logger().info("Notlauf")
        schub = 6.6
        SetFahrzeugSchub(schub)

      elif float(dist.data) > 32.0 and notlauf == 1:
        self.get_logger().info("Notlauf beednet")
        schub = 7.3
        SetFahrzeugSchub(schub)
        notlauf = 0

      else :
        self.get_logger().info("all is good")
    """



    def drive_callback(self, msg):
        global schub 
        schub = float(msg.data)
        self.get_logger().info('rec: "%s"' % msg.data)
        SetFahrzeugSchub(schub)
        
    def steer_callback(self, msg):
        global lenkung 
        global schub 
        lenkung = float(msg.data)
        self.get_logger().info('rec: "%s"' % msg.data)
        SetServoLenkung(lenkung)
    
        


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
    node = servosub()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


