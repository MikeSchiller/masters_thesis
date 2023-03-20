import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from std_msgs.msg import String

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# assign GPIO Pins
GPIO_TRIGGER1 = 27
GPIO_ECHO1 = 17
GPIO_TRIGGER2 = 9
GPIO_ECHO2 = 11

# Set direction of GPIO pins (IN --> Input / OUT --> Output)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)

class USPublisher(Node):


    def __init__(self):
        super().__init__('US_test')
        self.publisher_links = self.create_publisher(String, '/distance_links', 10)
        self.publisher_rechts = self.create_publisher(String, '/distance_rechts', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 

    def timer_callback(self):
        #abfrage erster Sensor LINKS
        msg1 = String()
        # set trigger to HIGH
        GPIO.output(GPIO_TRIGGER1, True)
    
        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER1, False)
    
        startTime = time.time()
        arrivalTime = time.time()
    
        # store startTime
        while GPIO.input(GPIO_ECHO1) == 0:
            startTime = time.time()
    
        # store arrivalTime
        while GPIO.input(GPIO_ECHO1) == 1:
            arrivalTime = time.time()
    
        # Time difference between start and arrival
        timeElapsed = arrivalTime - startTime
        # multiply by the speed of sound (34300 cm/s)
        # and divide by 2, there and back again
        distance = (timeElapsed * 34300) / 2
        msg1.data = str(distance)

#Abfrage zweiter Sensor RECHTS
        msg2 = String()
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
    
        # store arrivalTime
        while GPIO.input(GPIO_ECHO2) == 1:
            arrivalTime2 = time.time()
    
        # Time difference between start and arrival
        timeElapsed2 = arrivalTime2 - startTime2
        # multiply by the speed of sound (34300 cm/s)
        # and divide by 2, there and back again
        distance2 = (timeElapsed2 * 34300) / 2
        msg2.data = str(distance2)



        if distance > 2000:
            self.get_logger().info('Distanz macht keinen Sinn')
        else:
            self.get_logger().info('Hier die Distanz: "%s"' % distance)
            #print(type(msg), type(msg.data), msg.data)
            self.publisher_links.publish(msg1)
            self.publisher_rechts.publish(msg2)








def main(args=None):
    rclpy.init(args=args)
    node = USPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

