#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

servopin = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT)
pi_pwm = GPIO.PWM(servopin, 50)
pi_pwm.start(0)


class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')


    self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)
    self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


    pi_pwm.ChangeDutyCycle(int(msg.data))


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