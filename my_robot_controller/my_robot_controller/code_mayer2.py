#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

from std_msgs.msg import String

winkelwert = 0
servopin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT)
pi_pwm = GPIO.PWM(servopin, 50)
pi_pwm.start(2.5)



# Umrechnung Grad in Tastverhaeltnis
def setservo(winkel):
  if winkel < 0:
    winkel = 0
  if winkel > 180:
    winkel = 180
  winkelwert = winkel/18 + 2.5
  pi_pwm.ChangeDutyCycle(winkelwert) 
  


class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.get_logger().info('moinsen')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        setservo(float(msg.data))   
        #pi_pwm.ChangeDutyCycle(winkelwert) 
        #pi_pwm.ChangeDutyCycle(int(msg.data))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()