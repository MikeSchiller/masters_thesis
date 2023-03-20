#sends the processed data from the gps module to the carToCoords node instead of gps drive controller


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from geopy.distance import geodesic

msg_longitude = String()
msg_latitude = String()
msg_hdop = String()
msg_tr_head = String()



class gps_data_stuff(Node):


        


    def __init__(self):
        super().__init__('gps_data_stuff')
        self.get_logger().info('gps kriegt piep')
        self.sub_RMC = self.create_subscription(String, '/gps_RMC', self.RMC_callback, 10)
        self.sub_GGA = self.create_subscription(String, '/gps_GGA', self.GGA_callback, 10)
        self.pub_long = self.create_publisher(String, "/act_longitude",10)
        self.pub_lat = self.create_publisher(String, "/act_latitude",10)
        self.pub_heading = self.create_publisher(String, "/tracked_heading",10)
        self.pub_hdop = self.create_publisher(String, "/HDOP",10)

    def coord_long_converter(self,longitude):
        longitudedegree = float(longitude[0:3])
        longitudeminute = float(longitude[3:len(longitude)])
        dezLongitude = longitudedegree + (longitudeminute / 60)
        #print (longitudedegree)
        #print(longitudeminute)

        return dezLongitude
    
    def coord_lat_converter(self, latitude):
        latitudedegree = float(latitude[0:2])
        latitudeminute = float(latitude[2:len(latitude)])
        dezLatitude = latitudedegree + (latitudeminute / 60)

        return dezLatitude
        

    def RMC_callback(self, RMC_data):
        global msg_tr_head
        global msg_longitude
        global msg_latitude
        print ("#################################################################")

        if RMC_data.data == "":
            print ("Kein Signal")

        else:

            dataRMC = RMC_data.data
            splitdata = dataRMC.split(",")
            if len(splitdata) == 13:
                MessageID = splitdata[0]
                UTC_Time = splitdata[1]
                Status = splitdata[2]
                Latitude = splitdata[3] #Format Grad Minuten GGmm.mmmm https://de.wikipedia.org/wiki/NMEA_0183
                NorS = splitdata[4]
                Longitude = splitdata[5] #Format Grad Minuten GGGmm.mmmm
                WorE = splitdata[6]
                Speed_Over_Ground_kts = splitdata[7]
                Track_Angle = splitdata[8]
                Date = splitdata[9]
                Magnetic_Variation = splitdata[10]
                unknown = splitdata [11]
                Checksumpart = splitdata[12]

                            #adptUTC_Time = datetime.strptime(UTC_Time[0:6], '%H%M%S')

                if Latitude == "":
                    Latitude = "Kein Signal"
                else:
                    #convert latitude and longitude into degree format http://www.mwegner.de/geo/geo-koordinaten/umrechnung-grad-minute-sekunde-dezimalgrad.html
                    dezlatitude = self.coord_lat_converter(Latitude)
                    #convert variables to publishable data
                    msg_latitude.data = str(dezlatitude)
                    print("lati: " + str(dezlatitude))
                    self.pub_long.publish(msg_latitude)

                
                if Longitude == "":
                    Longitude = "Kein Signal"
                else:
                    dezlongitude = self.coord_long_converter(Longitude)
                    msg_longitude.data = str(dezlongitude)
                    print("longi: " + str(dezlongitude))
                    self.pub_long.publish(msg_longitude)
                
                if Track_Angle == "":
                    Track_Angle = "Kein Signal"
                else:
                    msg_tr_head.data = str(Track_Angle)
                    
                '''
                    print("MessageID: " +MessageID)
                    print("UTC-Time: " + UTC_Time[0:6])
                    print("Status: " + Status)
                    #print("Latitude: " + Latitude + NorS) 
                    #print ("Latitude(degrees): " + str(dezlatitude))
                    #print("Longitude: " + Longitude + WorE) 
                    print("Longitude(degrees): "+ str(dezlongitude))
                    print("Speed [knots]: " + Speed_Over_Ground_kts)
                    print("Track Angle: " +Track_Angle)
                    print("Date: " + Date)
                    print("Magnetic Variation: " + Magnetic_Variation)
                    print("unkown: " + unknown)
                    print("Checksumpart: " + Checksumpart)
                    self.pub_long.publish(msg_longitude)
                    self.pub_lat.publish(msg_latitude)
                    self.pub_heading.publish(msg_tr_head) 
                '''
                
            else:
                print("RMC s zu kuuz")
            
         
       


        
    def GGA_callback(self, GGA_data):
        global msg_hdop

        if GGA_data == "":
            print ("Kein Siganal")
        else:
            dataGGA= GGA_data.data
            splitdata = dataGGA.split(",")
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
                msg_hdop.data =  GGA_HDOP
                self.pub_hdop.publish(msg_hdop)
                
            else:
                print("GGA s zu kuuz")


        



def main(args=None):
    rclpy.init(args=args)
    node = gps_data_stuff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


