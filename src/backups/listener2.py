#!/usr/bin/python
import roslib
roslib.load_manifest('rgbdslam')
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import math
import sys
import os
import tf
import datetime
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '/opt/ros/fuerte/share/mavlink/pymavlink'))
from rgbdslam import *
import mavutil

master = mavutil.mavlink_connection()

def mainloop():
	
	initiallocation = [42.8704040000,-87.6492930000,30.0]
	curlocation = [0,0,0]
	curlocation[0] = initiallocation[0]
	curlocation[1] = initiallocation[1]
	curlocation[2] = initiallocation[2]
	rospy.init_node('listening',anonymous=True)
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	curx = 0.0
	cury = 0.0
	curz = 0.0
	lastx = 0.0
	lasty = 0.0
	del_dist = 0.0
	lasttime = 0.0
	curtime = 0.0
	while not rospy.is_shutdown():
		time.sleep(.15)
		lasttime = curtime
		curtime = time.time()
		elapsedtime = (curtime-lasttime)
		updaterate = 1/elapsedtime #Hz
		print updaterate
		dt = datetime.datetime.now()
		tempstr = "New Latitude: {:.10f} Longitude: {:.10f} Alt: {:.4f}".format(curlocation[0],curlocation[1],curlocation[2]) 
		#print tempstr
		tempstr = "Init Latitude: {:.10f} Longitude: {:.10f} Alt: {:.4f}".format(initiallocation[0],initiallocation[1],initiallocation[2])
		#print tempstr
		tempstr = "Delta(m) x: {:.4f} y: {:.4f} z: {:.4f}".format(curx,cury,curz)
		#print tempstr
		del_ground_dist = math.sqrt(math.pow((curx-lastx),2)+math.pow((cury-lasty),2))
		ground_speed = (del_ground_dist/elapsedtime)*100.0 #(cm/s)
		
		try:
			(pos,rot) = listener.lookupTransform('/map','/camera_link',rospy.Time(0))
			lastx = curx
			lasty = cury
			curx = pos[0]
			cury = pos[1]
			curz = pos[2]
			curlocation[0] = curx/110540.0 + initiallocation[0]
			curlocation[1] = cury/(111320*math.cos(curlocation[0]*3.14/180)) + initiallocation[1]
			curlocation[2] = curz #Altitude Transform from Ground Level			
			
		except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
			print "Error"
			curlocation[0] = initiallocation[0]
			curlocation[1] = initiallocation[1]
			curlocation[2] = initiallocation[2]
		a = dec2gpsdeg(curlocation[0])
		GPS_Lat = "{}{}{:.4f}".format(a[0],a[1],a[2])
		a = dec2gpsdeg(curlocation[1])
		GPS_Long = "{}{}{:.4f}".format(a[0],a[1],a[2])
		a = datetime2gpsdatetime(dt)
		GPS_Time = "{}{}{:.2f}".format(a[0],a[1],a[2])
		GPS_Date = "{}{}{}".format(a[3],a[4],a[5])
		GPS_Alt = curlocation[2]

		#Prepare GPRMC Message
		GPRMC = "GPRMC,{},A,{},N,{},W,{:.1f},0.0,{},0.0,E,".format(GPS_Time,GPS_Lat,GPS_Long,ground_speed,GPS_Date)
		a = calcchecksum(GPRMC)		
		GPRMC = "${}*{}".format(GPRMC,a)  #GPRMC message is ready for transmission
		print GPRMC

		#Prepare GPGGA Message
		GPGGA = "GPGGA,{},{},N,{},W,3,12,0,{},,0,,,".format(GPS_Time,GPS_Lat,GPS_Long,GPS_Alt)
		a = calcchecksum(GPGGA)
		GPGGA = "${}*{}".format(GPGGA,a)  #GPGGA message is ready for transmission
		print GPGGA
		
		#Prepare GPVTG Message
		GPVTG = "GPVTG,0,T,0,M,{:.1f},C,{:.1f},C".format(ground_speed,ground_speed)
		a = calcchecksum(GPVTG)
		GPVTG = "${}*{}".format(GPVTG,a)
		print GPVTG

def dec2gpsdeg(num):
  a = [0, 0, 0]
  a[0] = int(num)
  a[1] = int((num*60) % 60)
  a[2] = (num*3600)%60
  return a
def datetime2gpsdatetime(item):
  a = [0,0,0,0,0,0]
  a[0] = item.hour
  a[1] = item.minute
  a[2] = item.second + item.microsecond/1000000.0
  a[3] = item.day
  a[4] = item.month
  a[5] = item.year-2000
  return a


def calcchecksum(item):
	s = 0
	for i in range(len(item) ):
    		s = s ^ ord(item[i])
	s = "%02X" % s
	return s
		
if __name__ == '__main__':
        mainloop()


