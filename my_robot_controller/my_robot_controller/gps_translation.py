#This node takes the outputstring of gps_module and extracts all avalable information for further processing

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String



class gps_translation(Node):


    def __init__(self):
        super().__init__('gps_translation')
        self.get_logger().info('gps kriegt piep')
        self.get_logger().info("1")
        self.sub_RMC = self.create_subscription(String, '/gps_RMC', self.RMC_callback, 10)
        self.get_logger().info("2")
        self.sub_GGA = self.create_subscription(String, '/gps_GGA', self.GGA_callback, 10)
        self.get_logger().info("3")
        

    def RMC_callback(self, RMC_data):
        self.get_logger().info('Receiving: "%s"' % dataRMC)
        dataRMC = RMC_data.data
        splitdata = dataRMC.split(",")
        if len(splitdata) == 13:
            MessageID = splitdata[0]
            UTC_Time = splitdata[1]
            Status = splitdata[2]
            Latitude = splitdata[3]
            NorS = splitdata[4]
            Longitude = splitdata[5]
            WorE = splitdata[6]
            Speed_Over_Ground_kts = splitdata[7]
            Track_Angle = splitdata[8]
            Date = splitdata[9]
            Magnetic_Variation = splitdata[10]
            unknown = splitdata [11]
            Checksumpart = splitdata[12]

            #adptLatitude= float(Latitude) / 100
            #adptLongitude = float(Longitude) / 100
            #adptUTC_Time = datetime.strptime(UTC_Time[0:6], '%H%M%S')

            if Latitude == "":
                Latitude = "Kein Signal"
            else:
                pass
                
            if Longitude == "":
                Longitude = "Kein Signal"
            else:
                pass
            
            print("MessageID: " +MessageID)
            print("UTC-Time: " + UTC_Time[0:6])
            print("Status: " + Status)
            print("Latitude: " + Latitude + NorS) #str(adptLatitude))
            #print("North or South: " + NorS)
            print("Longitude: " + Longitude + WorE) #str(adptLongitude))
            #print("West or East: " + WorE)
            print("Speed [knots]: " + Speed_Over_Ground_kts)
            print("Track Angle: " +Track_Angle)
            print("Date: " + Date)
            print("Magnetic Variation: " + Magnetic_Variation)
            print("unkown: " + unknown)
            print("Checksumpart: " + Checksumpart)
            
        else:
            print("RMC s zu kuuz")


        
    def GGA_callback(self, GGA_data):
        splitdata = GGA_data.split(",")
        if len(splitdata) == 15:
            GGA_MessageID = splitdata[0]
            GGA_UTC_Time = splitdata[1]
            GGA_Latitude = splitdata[2]
            GGA_NorS = splitdata[3]
            GGA_Longitude = splitdata[4]
            GGA_WorE = splitdata[5]
            GGA_GPS_Quality = splitdata[6]
            GGA_Number_of_Satellites = splitdata[7]
            GGA_HDOP = splitdata[8]
            GGA_Othometric_Height = splitdata[9]
            GGA_unit_of_height = splitdata[10]
            GGA_geoid_seperation = splitdata [11]
            GGA_unit_geoid_seperation = splitdata[12]


            print("GPS Quality: " + GGA_GPS_Quality)
            print("NUmber of Satellites: " + GGA_Number_of_Satellites)
            print("Height over GND: " + GGA_Othometric_Height + GGA_unit_of_height)
            print("HDOP: " + GGA_HDOP)

        else:
            print("GGA s zu kuuz")



def main(args=None):
    rclpy.init(args=args)
    node = gps_translation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


