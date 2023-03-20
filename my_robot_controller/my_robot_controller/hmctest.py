#testscript for testing hmc5883L

from i2clibraries import i2c_hmc5883l
from std_msgs.msg import String
import time
import rclpy
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from geopy.distance import geodesic

i = 0
x = []
y=[]

hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
 
hmc5883l.setContinuousMode()
hmc5883l.setDeclination(2, 15)
#'''
while i < 500:
    temp = str(hmc5883l.getAxes())
    print(temp)
    splittemp = temp.split(',')
    xtemp = splittemp [0]
    xtemp = xtemp.replace('(', '')
    x.append( float(xtemp) )
    ytemp = splittemp[1]
    y.append(float(ytemp))
    i += 1
    time.sleep(0.1)

minx = min(x)
maxx = max(x)
miny = min(y)
maxy = max(y)
offsetx = (maxx + minx) /2
offsety = (maxy + miny) /2
print(offsetx)
print(offsety)
'''
while True:
    print(hmc5883l)
    time.sleep(0.5)'''
