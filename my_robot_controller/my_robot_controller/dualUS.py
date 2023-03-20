'''Auslesen von 2 US Sensoren '''

import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from std_msgs.msg import String

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# assign GPIO Pins
GPIO_TRIGGER1 = 27
GPIO_TRIGGER2 = 25
GPIO_ECHO1 = 17
GPIO_ECHO2 = 22

# Set direction of GPIO pins (IN --> Input / OUT --> Output)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
GPIO.setup(GPIO_ECHO2, GPIO.IN)

#set global variables
msg_left = String()
msg_right = String()

class USPublisher(Node):


    def __init__(self):
        super().__init__('dualUS')
        self.publisher_links = self.create_publisher(String, '/US_distance_links', 1)
        self.publisher_rechts = self.create_publisher(String, '/US_distance_rechts', 1)
        self.get_logger().info("EYYYY")
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer = self.create_timer(2, self.message_callback)
 

    def timer_callback(self):
        global msg_left
        global msg_right

        # set trigger to HIGH
        GPIO.output(GPIO_TRIGGER1, True)
    
        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER1, False)
    
        startTime1 = time.time()
        arrivalTime1 = time.time()
    
        # store startTime
        while GPIO.input(GPIO_ECHO1) == 0:
            startTime1 = time.time()
    
        # store arrivalTime
        while GPIO.input(GPIO_ECHO1) == 1:
            arrivalTime1 = time.time()
    
        # Time difference between start and arrival
        timeElapsed1 = arrivalTime1 - startTime1
        # multiply by the speed of sound (34300 cm/s)
        # and divide by 2, there and back again
        distance1 = (timeElapsed1 * 34300) / 2

        msg_left.data = str(distance1)

        time.sleep(0.1)
        
        #Messwert zweiter US Sensor
        # set trigger to HIGH
        GPIO.output(GPIO_TRIGGER2, True)
        
        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER2, False)
        
        startTime2 = time.time()
        arrivalTime2 = time.time()
        
        # store startTime
        while GPIO.input(GPIO_ECHO2) == 0:
            startTime2 = time.time()
        
        # store arrivalTime HIER PROBLEM
        while GPIO.input(GPIO_ECHO2) == 1:
            arrivalTime2 = time.time()
        
        # Time difference between start and arrival
        timeElapsed2 = arrivalTime2 - startTime2
        # multiply by the speed of sound (34300 cm/s)
        # and divide by 2, there and back again
        distance2 = (timeElapsed2 * 34300) / 2
        
        msg_right.data = str(distance2)


        #print ("links: " + msg_left.data)
        #print("rechts: " + msg_right.data)

        self.publisher_links.publish(msg_left)
        self.publisher_rechts.publish(msg_right)

        '''
        if distance1 > 300 or distance2 > 300:
            self.get_logger().info('Distanz macht keinen Sinn')
        else:
            #print(type(msg), type(msg.data), msg.data)
            self.publisher_links.publish(msg_left)
            self.publisher_rechts.publish(msg_right)
        '''
            
    def message_callback(self):
            self.get_logger().info('US_LEFT: "%s"' % msg_left.data)
            self.get_logger().info('US_RIGHT: "%s"' %  msg_right.data)






def main(args=None):
    rclpy.init(args=args)
    node = USPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

