'''Auslesen von der Gabellichtschranke und umrechnen in gefahrene Meter '''

import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from std_msgs.msg import String
import math


GPIO.setmode(GPIO.BCM)

# Hier wird der Eingangs-Pin deklariert, an dem der Sensor angeschlossen ist. Zusaetzlich wird auch der PullUP Widerstand am Eingang aktiviert
GPIO_PIN = 24
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

i = 0
start_time = 0
end_time = 0
elapsed_time = 0
turn_kurbelwelle= 0.0
turn_rad = 0.0
distance_travelled_odo = 0.0
rad_durchmesser = 0.0643 #[Meter]
rad_umfang = math.pi * rad_durchmesser # Umfang des Rades in [Meter]
distance = String()

class BarrierPublisher(Node):


    def __init__(self):
        super().__init__('Barrier_pub')
        self.publisher = self.create_publisher(String, '/distance_driven', 10)
       
        self.get_logger().info("Licht tut, licht tut nicht, licht tut....")
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    # Diese AusgabeFunktion wird bei Signaldetektion ausgefuehrt
    def ausgabeFunktion(self):
            global i
            global start_time 
            global end_time 
            global elapsed_time 
            global turn_kurbelwelle
            global turn_rad
            global rad_umfang
            global distance 


            i = i+1
            turn_kurbelwelle = i/20
            turn_rad = turn_kurbelwelle/4.80808
            distance_travelled_odo = turn_rad* rad_umfang
            distance.data = (str(distance_travelled_odo))
            # vom Sensor gemessene zurückgelegte Strecke in METER
            # Wird im ROS Knoten versendet
            #if i == 1000:
             #   start_time = time.perf_counter()
            #elif i > 1000 and i< 7130:

            #self.publisher.publish(distance)
            #self.get_logger().info(distance.data)

    def timer_callback(self):
        global distance
        self.publisher.publish(distance)
        self.get_logger().info(distance.data + "Meter")



        

 
    # Beim Detektieren eines Signals (steigende Signalflanke) wird die Ausgabefunktion ausgeloest
    GPIO.add_event_detect(GPIO_PIN, GPIO.RISING, callback=ausgabeFunktion, bouncetime=1)

  


def main(args=None):
    rclpy.init(args=args)
    node = BarrierPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

