#Website used for this node: https://tutorials-raspberrypi.com/build-your-own-raspberry-pi-compass-hmc5883l/
import rclpy
from rclpy.node import Node
import time
from i2clibraries import i2c_hmc5883l
from std_msgs.msg import String

hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
heading = 0
message = String()
 
hmc5883l.setContinuousMode()
# Set Declination according to this website: https://www.magnetic-declination.com/
hmc5883l.setDeclination(3, 42)

class Cmps_pub(Node):

    def __init__(self):
        super().__init__('Cmps_publisher')
        self.publisher_ = self.create_publisher(String, '/cmps_heading', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def heading_strtofloat_converter(self,heading):
        #"270° 29'"
        index1 = heading.find("°")
        index2 = index1 + 1
        index3 = heading.find("'")

        head_deg = float(heading[0:index1])
        head_min = float(heading[index2:index3])
        heading_float = head_deg + (1/60*head_min)
        heading_string = str(heading_float)
        heading_string = heading_string[0:6]
        return heading_string   

    def timer_callback(self):
        global hmc5883l
        global heading
        global message

        # To get degrees and minutes into variables
        #(degrees, minutes) = hmc5883l.getDeclination()
        #(degress, minutes) = hmc5883l.getHeading()
 
        # To get string of degrees and minutes
        #declination = hmc5883l.getDeclinationString()
        heading = hmc5883l.getHeadingString()

        message.data = self.heading_strtofloat_converter(heading)

        self.publisher_.publish(message)
        self.get_logger().info('Publishing: "%s"' % message.data)



def main(args=None):
    rclpy.init(args=args)

    cmps_pub = Cmps_pub()

    rclpy.spin(cmps_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmps_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()