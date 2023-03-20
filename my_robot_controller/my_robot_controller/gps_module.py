'Reads the Data of the GSM/GPRS/GNSS HAT and preapares for further refinenment'
'Code based on tutorial by manufacturer waveshare https://www.waveshare.com/wiki/GSM/GPRS/GNSS_HAT'

#!/usr/bin/python
import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from std_msgs.msg import String
import serial


'''
#startup gps module
# Not used for debug purposes.
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.output(7, GPIO.LOW)
time.sleep(4)
GPIO.output(7, GPIO.HIGH)
'''

ser = serial.Serial("/dev/ttyS0",115200)
W_buff = [b"AT+CGNSPWR=1\r\n", b"AT+CGNSSEQ=\"RMC\"\r\n", b"AT+CGNSINF\r\n", b"AT+CGNSURC=2\r\n", b"AT+CGNSTST=1\r\n"]
ser.write(W_buff[0])
ser.flushInput()
data = ""
num = 0
RMCdatasend = String()
GGAdatasend = String()


class GPS_module(Node):



    def __init__(self):
        super().__init__('GPS_module')
        self.pub_RMC = self.create_publisher(String, '/gps_RMC', 10)
        self.pub_GGA = self.create_publisher(String, '/gps_GGA', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def extract_RMC(self, big_string):
        print(big_string)
        start = 'GNRMC'
        end = '*'
        results = ""
        start_index = big_string.find(start)
        end_index = big_string.find(end, start_index)

        if start_index <= 0:
            print("nope")
        else:
            results = big_string[start_index:end_index]
        
        return results
    
    def extract_GGA(self, big_string):
        
        start = 'GGA'
        end = '*'
        results = ""
        start_index = big_string.find(start)
        end_index = big_string.find(end, start_index)

        if start_index <= 0:
            print("nope")
        else:
            results = big_string[start_index:end_index]
        
        return results
            
    def timer_callback(self):
        global num
        global ser
        global data
        global W_buff
        global RMCdatasend
        global GGAdatasend
        #print (str(ser.inWaiting()))
        while ser.inWaiting() > 0:
            data += str(ser.read(ser.inWaiting()))
            
            
        if data != "":
            #print (data)

            if  num < 4:	# the string have ok
                print (num)
                time.sleep(0.5)
                ser.write(W_buff[num+1])
                num =num +1
            if num == 4:
                time.sleep(1)
                ser.write(W_buff[4])
                RMCdata = self.extract_RMC(data)
                RMCdatasend.data =  RMCdata 
                #RMCdatasend.data = "GNRMC,120808.000,A,4825.106315,N,00956.307472,E,0.00,109.88,271023,,,A" #CAVE, only for debug
                GGAdata = self.extract_GGA(data)
                GGAdatasend.data = GGAdata 
                #GGAdatasend.data = "GGA,120808.000,4825.106315,N,00956.307472,E,1,5,2.83,602.869,M,48.000,M,," #CAVE, only for debug
                if GGAdatasend == "" or RMCdatasend == "":
                    pass
                else:
                    self.pub_RMC.publish(RMCdatasend)
                    self.get_logger().info('Publishing: "%s"' % RMCdatasend.data)
                    self.pub_GGA.publish(GGAdatasend)
                    self.get_logger().info('Publishing: "%s"' % GGAdatasend.data)
                    
            data = ""



def main(args=None):
    rclpy.init(args=args)
    node = GPS_module()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

