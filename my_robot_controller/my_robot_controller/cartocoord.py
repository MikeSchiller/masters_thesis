import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String


class CarToCoords(Node):

    def __init__(self):
        super().__init__('carToCoords')
        self.car_long = self.create_publisher(String, 'car_long', 10)
        self.car_lat = self.create_publisher(String, 'car_lat', 10)
        self.sub_odo = self.create_subscription(String,'/car_odo', self.Odo_callback, 10)
        self.sub_steer = self.create_subscription(String,'/car_steer', self.Steer_callback, 10)



    def Odo_callback(self, car_odo):
        car_odo1 = car_odo

    def Steer_callback(self, car_steer):
        
        pass



def main(args=None):
    rclpy.init(args=args)

    Car_to_coords = CarToCoords()

    rclpy.spin(Car_to_coords)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Car_to_coords.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()