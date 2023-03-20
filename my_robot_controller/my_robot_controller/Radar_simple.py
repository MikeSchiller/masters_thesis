import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
import socket
import sys
import matplotlib.pyplot as plt
import math

def recv_msg(sockTCP, msg_len, max_msg_size):
    resp_frame = bytearray(msg_len)
    pos = 0
    while pos < msg_len:
        resp_frame[pos:pos + max_msg_size] = sockTCP.recv(max_msg_size)
        pos += max_msg_size
    return resp_frame

# Define variables
msg_len = 9
max_msg_size = 8
packageLength = 1500
x = list(range(128))

# Create TCP_IP object with corresponding IP and port
TCP_IP = '192.168.100.201' # original ip address
#TCP_IP = '192.168.2.201'
TCP_PORT = 6172
sockTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sockTCP.connect((TCP_IP, TCP_PORT))
except:
    print('Error while connecting with TCP/IP socket')
    sys.exit(1)

# Create UDP object with corresponding IP and port
UDP_IP = "192.168.100.1"
UDP_PORT = 4567
sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
try:
    sockUDP.bind((UDP_IP, UDP_PORT))
except:
    print('Error while connecting with UDP socket')
    sys.exit(1)

# Connect with sensor
header = bytes("INIT", 'utf-8')
payloadlength = (0).to_bytes(4, byteorder='little')
cmd_frame = header + payloadlength
sockTCP.send(cmd_frame)
resp_frame = recv_msg(sockTCP, msg_len, max_msg_size)
if resp_frame[8] != 0:
    print('Error: Command not acknowledged')
    sys.exit(1)

# Set max range to 10m
header = bytes("RSET", 'utf-8')
payloadlength = (4).to_bytes(4, byteorder='little')
max_range = (1).to_bytes(4, byteorder='little')
cmd_frame = header + payloadlength + max_range
sockTCP.send(cmd_frame)
resp_frame = recv_msg(sockTCP, msg_len, max_msg_size)
if resp_frame[8] != 0:
    print('Error: Command not acknowledged')
    sys.exit(1)

# Create figure
fig = plt.figure(figsize=(10, 5))
plt.ion()
plt.show()

# Enable PDAT and TDAT data
header = bytes("RDOT", 'utf-8')
datarequest = (24).to_bytes(4, byteorder='little')
cmd_frame = header + payloadlength + datarequest
sockTCP.send(cmd_frame)
resp_frame = recv_msg(sockTCP, msg_len, max_msg_size)

# init arrays
distance_pdat = []
speed_pdat = []
azimuth_pdat = []
elevation_pdat = []
magnitude_pdat = []
distance_tdat = []
speed_tdat = []
azimuth_tdat = []
elevation_tdat = []
magnitude_tdat = []

class RadarPublisher(Node):


    def __init__(self):
        super().__init__('Radar_test')
        self.publisher_x = self.create_publisher(String, '/Radar_distances_x', 10)
        self.publisher_y = self.create_publisher(String, '/Radar_distances_y', 10)
        self.trackcount_publisher= self.create_publisher(String, '/Radar_trackcount', 10)
        self.get_logger().info("Radar macht piep")
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
 

    def timer_callback(self):
        # readout and plot time and frequency adc_data continuously
        for ctr in range(100):
            
            # GET PDAT DATA ---------------------------------
            pdat_data = []
            packageData, adr = sockUDP.recvfrom(packageLength)
            while packageData[0:4] != b'PDAT':  # do while header isn't expected header
                packageData, adr = sockUDP.recvfrom(packageLength)
            respLength = int.from_bytes(packageData[4:8], byteorder='little')  # get response length
            numberoftargets = round(respLength / 10)  # calculate number of detected targets
            packageData = packageData[8:len(packageData)]  # exclude header from data
            pdat_data = packageData  # store data
            packageData, adr = sockUDP.recvfrom(packageLength)  # get data
            while packageData.find(b'TDAT') == -1:
                pdat_data += packageData  # store data
                packageData, adr = sockUDP.recvfrom(packageLength)  # get data
            

            # GET TDAT DATA -------------------------------
            respLength = int.from_bytes(packageData[4:8], byteorder='little')  # get response length
            numberoftrackedtargets = round(respLength / 10)  # calculate number of tracked targets
            packageData = packageData[8:len(packageData)]  # exclude header from data
            tdat_data = packageData  # store data
            packageData, adr = sockUDP.recvfrom(packageLength)  # get data
            while packageData.find(b'PDAT') == -1:
                tdat_data += packageData  # store data
                packageData, adr = sockUDP.recvfrom(packageLength)  # get data


            # init arrays
            distance_pdat = []
            speed_pdat = []
            azimuth_pdat = []
            elevation_pdat = []
            magnitude_pdat = []
            distance_tdat = []
            speed_tdat = []
            azimuth_tdat = []
            elevation_tdat = []
            magnitude_tdat = []

            #initialize variables for sending
            msgx =  String()
            msgy = String()
            msgc = String()

            # get distance [cm], speed [km/h*100] and azimuth angle [degree*100] of the detected raw targets by converting pdat into uint16/int16
            for target in range(0, numberoftargets):
                distance_pdat.append(int.from_bytes(pdat_data[10 * target:10 * target + 2], byteorder='little', signed=False))
                speed_pdat.append(
                    int.from_bytes(pdat_data[10 * target + 2:10 * target + 4], byteorder='little', signed=True) / 100)
                azimuth_pdat.append(
                    math.radians(
                        int.from_bytes(pdat_data[10 * target + 4:10 * target + 6], byteorder='little', signed=True) / 100))
                elevation_pdat.append(
                    math.radians(
                        int.from_bytes(pdat_data[10 * target + 6:10 * target + 8], byteorder='little', signed=True) / 100))
                magnitude_pdat.append(
                    int.from_bytes(pdat_data[10 * target + 8:10 * target + 10], byteorder='little', signed=False))

            # get distance [cm], speed [km/h*100] and azimuth angle [degree*100] of the tracked targets by convert tdat data into uint16/int16
            for target in range(0, numberoftrackedtargets):
                distance_tdat.append(int.from_bytes(tdat_data[10 * target:10 * target + 2], byteorder='little', signed=False))
                speed_tdat.append(
                    int.from_bytes(tdat_data[10 * target + 2:10 * target + 4], byteorder='little', signed=True) / 100)
                azimuth_tdat.append(
                    math.radians(
                        int.from_bytes(tdat_data[10 * target + 4:10 * target + 6], byteorder='little', signed=True) / 100))
                elevation_tdat.append(
                    math.radians(
                        int.from_bytes(tdat_data[10 * target + 6:10 * target + 8], byteorder='little', signed=True) / 100))
                magnitude_tdat.append(
                    int.from_bytes(tdat_data[10 * target + 8:10 * target + 10], byteorder='little', signed=False))
            # clear figure
            #plt.clf()

            # calculate x and y coordinates and plot the detected raw targets
            #sub1 = fig.add_subplot(121)
            distance_x = 0
            distance_y = 0
            #linecolor = ''
            for target in range(0, numberoftargets):
                distance_x = distance_pdat[target] * math.sin(azimuth_pdat[target]) / 100
                distance_y = distance_pdat[target] * math.cos(azimuth_pdat[target]) / 100
                #self.get_logger().info(str(distance_y))


            # calculate x and y coordinates and store in arrays for sending

            #sub2 = fig.add_subplot(122)
            distances_x = []
            distances_y = []
            for target in range(0, numberoftrackedtargets):
                distances_x.append(float(f'{(distance_tdat[target] * math.sin(azimuth_tdat[target]) / 100):.4f}'))
                distances_y.append(float(f'{(distance_tdat[target] * math.cos(azimuth_tdat[target]) / 100):.4f}'))
                #distance gets truncated to only inlcude the first 4 digits after . 


            #send the acummulated data
            msgx.data = str(distances_x)
            msgy.data = str(distances_y)
            msgc.data = str(numberoftrackedtargets)
            self.publisher_x.publish(msgx)
            self.publisher_y.publish(msgy)
            self.trackcount_publisher.publish(msgc)






def main(args=None):
    rclpy.init(args=args)
    node = RadarPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
    # disconnect from sensor
    payloadlength = (0).to_bytes(4, byteorder='little')
    header = bytes("GBYE", 'utf-8')
    cmd_frame = header + payloadlength
    sockTCP.send(cmd_frame)

    # get response
    response_gbye = recv_msg(sockTCP, msg_len, max_msg_size)
    if response_gbye[8] != 0:
        print('Error during disconnecting with V-MD3')
        sys.exit(1)

    # close connection to TCP/IP
    sockTCP.close()

    # close connection to UDP
    sockUDP.close()

