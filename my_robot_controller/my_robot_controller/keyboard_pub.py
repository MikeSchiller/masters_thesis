import rclpy
from rclpy.node import Node
import time
import getch

from std_msgs.msg import String
#from std_msgs.msg import Float32
check = 0
checklenk = 0
links = 120
rechts = 50
vor = 9
zurück = 5
stopfahr = 7.3
stoplenk = 90
checkswitch = 0


class KeyboardPublisher(Node):
     
    check = 0

    def __init__(self):

        super().__init__('keyboard_publisher')
        self.get_logger().info("YO")
        #self.distance_subscriber_ = self.create_subscription(String,'/distance', self.distance_callback, 10)
        self.pub_steer = self.create_publisher(String, '/steering', 10)
        self.pub_drive = self.create_publisher(String, '/driving', 10)
        self.pub_switch = self.create_publisher(String, '/switch', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)       
    

    def timer_callback(self):
        global check 
        global checklenk
        global links
        global rechts 
        global vor 
        global zurück 
        global stopfahr
        global stoplenk
        global checkswitch


        msg1 = String()
        msg2 = String()
        msg3 = String()
        input1 = getch.getch()


        if input1 == '1':
            links = 95
            rechts = 85
        
        if input1 == '2':
            links = 100
            rechts = 80
            
        
        if input1 == '3':
            links = 110
            rechts = 70
        

        if input1 == '4':
            links = 120
            rechts = 60
        

        if input1 == '5':
            links = 130
            rechts = 50
            
        if input1 == '-':
            vor = 7.7
            zurück = 6.8
        
        if input1 == '+':
            vor = 8
            zurück = 6.3

        if input1 == 'v':
            vor = 10.5
            zurück = 3.5       
        
        if input1 == 'w':
            if check == 0:
                msg2.data = str(vor)
                check = 1
            else:
                msg2.data = str(stopfahr)
                check=0

        if input1 == "a":
            if checklenk == 0:
                msg1.data = str(links)
                checklenk = 1
            else:
                msg1.data = str(stoplenk)
                checklenk=0

        if input1 == "s":
            if check == 0:
                msg2.data = str(zurück)
                check = 1
            else:
                msg2.data = str(stopfahr)
                check = 0

        if input1 == "d":
            if checklenk == 0:
                msg1.data = str(rechts)
                checklenk = 1
            else:
                msg1.data = str(stoplenk)
                checklenk=0

        if input1 == "p":
            if checkswitch == 0:
                msg3.data = "on"
                checkswitch = 1
            
            else:
                msg3.data = "off"
                checkswitch = 0
        
        if input1 == "l":
            stoplenk = stoplenk + 0.5
            msg1.data = str(stoplenk)

        if input1 == "r":
            stoplenk = stoplenk - 0.5 
            msg1.data = str(stoplenk)           


       

        if  input1 == "a" or input1 == "d": 
            self.pub_steer.publish(msg1)
            self.get_logger().info('Publishing1: "%s"' % msg1.data)
  
        if  input1 == "l" or input1 == "r": 
            self.pub_steer.publish(msg1)
            self.get_logger().info('Publishing1: "%s"' % msg1.data)
            
        elif input1 == "w" or input1 == "s": 
            self.pub_drive.publish(msg2)
            self.get_logger().info('Publishing2: "%s"' % msg2.data)

        elif input1 == "p":
            self.pub_switch.publish(msg3)
            self.get_logger().info('Publishing3: "%s"' % msg3.data)

        else:
            self.get_logger().info('nö')
     




def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()