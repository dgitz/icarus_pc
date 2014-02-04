#!/usr/bin/python
import roslib
roslib.load_manifest('rgbdslam')
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu,CameraInfo
import math
import sys
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
os.environ['MAVLINK10'] = '1'
import tf
import datetime
import time
import socket
import errno
import serial
import rgbdslam.msg
from icarus_helper import *
from collections import namedtuple
from pprint import pprint
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '/opt/ros/fuerte/share/mavlink/pymavlink'))

#from rgbdslam import *

from optparse import OptionParser
parser = OptionParser("primarycontroller.py [options]")
#parser.add_option("--gcs-device",dest="gcs_device",default="None",help="GCS Device Connection: /dev/ttyUSB0,10.7.45.208,etc")
parser.add_option("--mode",dest="mode",default="None",help="oa,net,slam,None")
parser.add_option("--gcs-device-type",dest="gcs_device_type",default="None",help="Serial,udp,tcp")
parser.add_option("--gcs-device",dest="gcs_device",default="None",help="GCS Device connection: /dev/ttyUSB0,10.7.45.208")
parser.add_option("--gcs-device-speed",dest="gcs_device_speed",default="57600")
parser.add_option("--remote-device-type",dest="remote_device_type",default="None",help="Serial,udp,tcp")
parser.add_option("--remote-device",dest="remote_device",default="None",help="Remote Device Connection: 192.168.1.104")
parser.add_option("--remote-device-port",dest="remote_device_port",default="9761",help="Port #")
parser.add_option("--fc-device-type",dest="fc_device_type",default="None",help="Serial")
parser.add_option("--fc-device",dest="fc_device",default="None")
parser.add_option("--fc-device-speed",dest="fc_device_speed",default="115200")
parser.add_option("--fcgps-device-type",dest="fcgps_device_type",default="None",help="Serial")
parser.add_option("--fcgps-device",dest="fcgps_device",default="None",help="FC Device connection: /dev/ttyUSB0")
parser.add_option("--fcgps-device-speed",dest="fcgps_device_speed",default="38400")
parser.add_option("--mc-device-type",dest="mc_device_type",default="None",help="Serial")
parser.add_option("--mc-device",dest="mc_device",default="None")
parser.add_option("--mc-device-speed",dest="mc_device_speed",default="57600")
import mavlinkv10 as mavlink
import mavutil

(opts,args) = parser.parse_args()
#print "Flight Controller: " + opts.fc_device
#print "device_gcs.device: " + opts.gcs_device
#print "Flight Controller GPS: " + opts.fcgps_device
#print "Remote: " + opts.remote_device

#WaypointStruct = namedtuple('WaypointStruct',['seq','frame','command','current','autocontinue','param1','param2','param3','param4','x','y','z'])
 
		

#my_MissionItems.append(missionitem())
#my_MissionItems[0].x = 41.3
#my_MissionItems[0].y = -87.3
#print my_MissionItems[0].calc_relbearing(12.0,37.0)
#print my_MissionItems[0].calc_distance(12.0,37.0)

mode = opts.mode
device_pc = device(enabled=True,name="Primary Controller",conn="Self")
device_pc.appenderror(calc_errorcode(system=SYSTEM_FLYER_PC,errortype=ERRORTYPE_NOERROR,severity=SEVERITY_CAUTION,message=MESSAGE_INITIALIZING))
device_pc.mav_state=mavlink.MAV_STATE_BOOT
device_pc.setcolor(TERM_RED)
device_pc.protocol = "MAVLINK"
device_pc.update_rate = 5.0
device_pc.type = "UAV"
if opts.fc_device <> "None":
	
	if opts.fc_device_type == "Serial":		
		#fc = mavutil.mavlink_connection(opts.fc_device, baud=opts.fc_device_speed)
		device_fc = device(enabled=True,name="Flight Controller",conn=str(opts.fc_device))
		device_fc.device = mavutil.mavlink_connection(opts.fc_device, baud=opts.fc_device_speed)
		device_fc.protocol = "APM_MAVLINK"
		device_fc.appenderror(calc_errorcode(SYSTEM_FLYER_FC,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
		#device_fc.device = fc
		
	else:
		print "Unsupported option for Flight Controller."
		sys.exit(0)
else:
	device_fc = device(enabled=False,name="Flight Controller",conn=None)
	device_fc.appenderror(calc_errorcode(SYSTEM_FLYER_FC,ERRORTYPE_GENERALERROR,SEVERITY_SEVERE,MESSAGE_DEVICENOTPRESENTORAVAILABLE))
device_fc.setcolor(TERM_YELLOW)
device_fc.update_rate = 5.0
device_fc.type = "UAV"

if opts.mc_device <> "None":
	if opts.mc_device_type == "Serial":
		device_mc = device(enabled=True,name="Motion Controller",conn=str(opts.mc_device))
		device_mc.device = serial.Serial(opts.mc_device,57600,timeout=1)
		device_mc.protocol = "ICARUS"
		device_mc.appenderror(calc_errorcode(SYSTEM_FLYER_MC,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
	else:
		print "Unsupported option for Motion Controller. "
		sys.exit(0)
else:
	device_mc = device(enabled=False,name="Motion Controller",conn=None)
	device_mc.appenderror(calc_errorcode(SYSTEM_FLYER_MC,ERRORTYPE_GENERALERROR,SEVERITY_SEVERE,MESSAGE_DEVICENOTPRESENTORAVAILABLE))
device_mc.setcolor(TERM_BLUE)
device_mc.update_rate = 5.0
device_mc.type = "UAV"
if opts.gcs_device <> "None":
	
	if opts.gcs_device_type == "udp":
		tempstr = "udp:"+str(opts.gcs_device)+str(":14550")
		
		device_gcs = device(enabled=True,name="Ground Control Station",conn=str(tempstr))
		device_gcs.device = mavutil.mavlink_connection(tempstr,input=False,source_system=1)
		device_gcs.appenderror(calc_errorcode(SYSTEM_GCS,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
		device_gcs.protocol = "MAVLINK"
		#device_gcs.device = gcs
	elif opts.gcs_device_type == "Serial":
		device_gcs = device(enabled=True,name="GCS",conn=str(opts.gcs_device))
		device_gcs.device = mavutil.mavlink_connection(opts.gcs_device, baud=opts.gcs_device_speed)
		device_gcs.protocol = "MAVLINK"
		device_gcs.appenderror(calc_errorcode(SYSTEM_GCS,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
		
	else:
	  	print "Unsupported option for GCS."
		sys.exit(0)
else:
	#gcs = False
	device_gcs = device(enabled=False,name="Ground Control Station",conn=None)
	device_gcs.appenderror(calc_errorcode(SYSTEM_GCS,ERRORTYPE_GENERALERROR,SEVERITY_SEVERE,MESSAGE_DEVICENOTPRESENTORAVAILABLE))
device_gcs.setcolor(TERM_GREEN)
device_gcs.update_rate = 5.0
device_gcs.type = "CONTROL"

if opts.fcgps_device <> "None":
	if opts.fcgps_device_type == "Serial":
		device_fcgps = device(enabled=True,name="Flight Controller GPS",conn=str(opts.fcgps_device))
		device_fcgps.device = serial.Serial(opts.fcgps_device,38400,timeout=1)		
		device_fcgps.appenderror(calc_errorcode(SYSTEM_FLYER_FCGPS,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
		device_fcgps.protocol = "NMEA"
	else:
		print "Unsupported option for Flight Controller GPS."
		sys.exit(0)
		
else:
	#fcgps = False
	device_fcgps = device(enabled=False,name="Flight Controller GPS",conn=None)
	device_fcgps.appenderror(calc_errorcode(SYSTEM_FLYER_FCGPS,ERRORTYPE_GENERALERROR,SEVERITY_SEVERE,MESSAGE_DEVICENOTPRESENTORAVAILABLE))
device_fcgps.setcolor(TERM_PURPLE)
device_fcgps.update_rate = 5.0
device_fcgps.type = "GPS"
		
if opts.remote_device <> "None":
	if opts.remote_device_type == "udp":
		tempstr = "udp:"+str(opts.remote_device)+str(":14550")
		device_remote = device(enabled=True,name="Remote",conn=str(tempstr))		
		device_remote.device = mavutil.mavlink_connection(tempstr,input=False,source_system=1)
		device_remote.appenderror(calc_errorcode(SYSTEM_REMOTE,ERRORTYPE_NOERROR,SEVERITY_CAUTION,MESSAGE_INITIALIZING))
		device_remote.protocol = "APM_MAVLINK"
	else:
		print "Unsupported option for Remote."
		sys.exit(0)
else:
	device_remote = device(enabled=False,name="Remote",conn=None)
	device_remote.appenderror(calc_errorcode(SYSTEM_REMOTE,ERRORTYPE_GENERALERROR,SEVERITY_SEVERE,MESSAGE_DEVICENOTPRESENTORAVAILABLE))
device_remote.setcolor(TERM_WHITE)
device_remote.update_rate = 5.0
device_remote.type = "CONTROL"

device_pc.enableprint = True
device_fc.enableprint = True
device_fcgps.enableprint = True
device_mc.enableprint = True
device_remote.enableprint = True
device_gcs.enableprint = True




device_pc.display()
#device_pc.display_errors()
device_fc.display()
#device_fc.display_errors()
device_fcgps.display()
#device_fcgps.display_errors()
device_mc.display()
#device_mc.display_errors()
device_gcs.display()
#device_gcs.display_errors()

device_remote.display()
#device_remote.display_errors()

#ROS Publishers
pub_gps = rospy.Publisher('GPS',rgbdslam.msg.GPS)
pub_pos = rospy.Publisher('Position',rgbdslam.msg.Position)
pub_attitude = rospy.Publisher('Attitude',rgbdslam.msg.Attitude)
pub_data_to_fc_gps = rospy.Publisher('Data_To_FC_GPS',rgbdslam.msg.DataToFCGPS)
pub_data_to_fc = rospy.Publisher('Data_To_FC',rgbdslam.msg.DataToFC)
pub_data_from_fc = rospy.Publisher('Data_From_FC',rgbdslam.msg.DataFromFC)



my_MissionItems = []
#WaypointStruct = namedtuple('WaypointStruct',['seq','frame','command','current','autocontinue','param1','param2','param3','param4','x','y','z'])

class ros_service:
	def __init__(self):
		if mode == 'None':
			dumb = 1
	
def mainloop():
	time.sleep(3)
	initvariables()
	global WaypointCount
	global Current_Pitch_rad
	global Current_Roll_rad
	global Current_Yaw_rad
	global starttime
	global first_attitude_packet
	global Initial_Yaw_rad
	global my_MissionItems
	global imagenum
	my_MissionItems = []
	#device_gcs.display()
	first_attitude_packet = True
	Initial_Yaw_rad = 0.0
	initiallocation = [41.8702840000,87.6492970000,240.0]
	curlocation = [0,0,0]
	curlocation[0] = initiallocation[0]
	curlocation[1] = initiallocation[1]
	curlocation[2] = initiallocation[2]
	#rospy.init_node('pc',anonymous=True)
	
	
	rospy.init_node('ros_service',anonymous=True)
	rc = ros_service()
	rate = rospy.Rate(10.0)
	listener = tf.TransformListener()
	break_counter = 0
	break_counter_max = 20
	if device_fc.enabled == True:
		init_device(device_fc.device)		
	if device_gcs.enabled == True:
		send_heartbeat(device_gcs,device_pc.state)
		#init_device(device_gcs.device)
		
	

	curx = 0.0
	cury = 0.0
	curz = 0.0
	lastx = 0.0
	lasty = 0.0
	del_dist = 0.0
	lasttime = 0.0
	starttime = time.time()
	curtime = starttime
	user_command = "q"
	pc_state = mavlink.MAV_STATE_STANDBY
	device_pc.state=mavlink.MAV_STATE_STANDBY
	device_pc.changemode(mavlink.MAV_MODE_PREFLIGHT)
	device_pc.appenderror(calc_errorcode(system=SYSTEM_FLYER_PC,errortype=ERRORTYPE_NOERROR,severity=SEVERITY_INFORMATION,message=MESSAGE_NOERROR))
        
	#device_mc.changemode(mavlink.MAV_MODE_PREFLIGHT)		
	print "Waiting..."
	rospy.sleep(1)
	#device_mc.changemode(mavlink.MAV_MODE_MANUAL_DISARMED)	
	
        
	#device_fc.changemode(mavlink.MAV_MODE_PREFLIGHT)
	
	#rospy.sleep(5) #Wait 15 seconds to allow all devices to powerup
	
	#device_mc.changemode(mavlink.MAV_MODE_MANUAL_DISARMED)
	
	while not rospy.is_shutdown():
		#time.sleep(1)
		#tempstr = 'image{}.png'.format(imagenum)
		#imagenum = imagenum + 1
		#print tempstr
		#cv2.imwrite(tempstr,color_image)
		#time.sleep(.001)
		rospy.sleep(0.001)
		lasttime = curtime
		curtime = time.time()
		elapsedtime = (curtime-lasttime)
		boottime = int((curtime-starttime)*1000)
		#sprint boottime
		updaterate = 1/elapsedtime #Hz
		#print updaterate
		dt = datetime.datetime.now()
		
		tempstr = "New Latitude: {:.10f} Longitude: {:.10f} Alt: {:.4f}".format(curlocation[0],curlocation[1],curlocation[2]) 
		#print tempstr
		tempstr = "Init Latitude: {:.14f} Longitude: {:.14f} Alt: {:.4f}".format(initiallocation[0],initiallocation[1],initiallocation[2])
		#print tempstr
		tempstr = "Delta(m) x: {:.4f} y: {:.4f} z: {:.4f}".format(curx,cury,curz)
		#print tempstr
		del_ground_dist = math.sqrt(math.pow((curx-lastx),2)+math.pow((cury-lasty),2))
		ground_speed = (del_ground_dist/elapsedtime)*100.0 #(cm/s)
		
		
		
		try:
			if first_attitude_packet == False:
				(pos,rot) = listener.lookupTransform('/map','/camera_link',rospy.Time(0))
				lastx = curx
				lasty = cury
				cury = pos[0]*math.sin(Initial_Yaw_rad)
				curx = pos[1]*math.cos(Initial_Yaw_rad)
				curz = pos[2]
				curlocation[0] = curx/110540.0 + initiallocation[0]
				curlocation[1] = -cury/(111320*math.cos(curlocation[0]*3.14/180)) + initiallocation[1]
				curlocation[2] = curz + initiallocation[2]#Altitude Transform from Ground Level
					
			
		except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
			#print "Error"
			curlocation[0] = initiallocation[0]
			curlocation[1] = initiallocation[1]
			curlocation[2] = initiallocation[2]
		
		pub_pos.publish(str(boottime),str(curx),str(cury),str(curz))
		GPS_Lat_dec = curlocation[0]
		GPS_Long_dec = curlocation[1]
		GPS_Alt = curlocation[2]
		a = dec2gpsdeg(GPS_Lat_dec)		
		GPS_Lat = "{}{:.5f}".format(a[0],a[1])
		a = dec2gpsdeg(GPS_Long_dec)
		GPS_Long = "{}{:.5f}".format(a[0],a[1])
		a = datetime2gpsdatetime(dt)
		GPS_Time = "{}{}{:.2f}".format(a[0],a[1],a[2])
		GPS_Date = "{}{}{}".format(a[3],a[4],a[5])		

		#Prepare GPRMC Message
		GPRMC = "GPRMC,{},A,{},N,0{},W,{:.1f},0.0,{},0.0,E,".format(GPS_Time,GPS_Lat,GPS_Long,ground_speed,GPS_Date)
		a = calcchecksum(GPRMC)		
		GPRMC = "${}*{}\r\n".format(GPRMC,a)  #GPRMC message is ready for transmission

		#Prepare GPGGA Message
		GPGGA = "GPGGA,{},{},N,0{},W,3,12,0,{},,0,,,".format(GPS_Time,GPS_Lat,GPS_Long,GPS_Alt)
		a = calcchecksum(GPGGA)
		GPGGA = "${}*{}\r\n".format(GPGGA,a)  #GPGGA message is ready for transmission
		
		#Prepare GPVTG Message
		GPVTG = "GPVTG,0,T,0,M,{:.1f},C,{:.1f},C".format(ground_speed,ground_speed)
		a = calcchecksum(GPVTG)
		GPVTG = "${}*{}\r\n".format(GPVTG,a)

		pub_gps.publish(str(boottime),str(GPS_Lat_dec),str(GPS_Long_dec),str(GPS_Alt))
		if device_pc.enabled == True:
			if ((boottime - device_pc.last_update)>(1/device_pc.update_rate*1000.0)):
				device_pc.last_update = boottime
				device_pc.display()
		if device_fc.enabled == True:
			update_device(device_fc)
			print "a"
			if ((boottime - device_fc.last_update)>(1/device_fc.update_rate*1000.0)):
				device_fc.last_update = boottime
				device_fc.display()
				#device_fc.changemode(APM_CIRCLE)
		if device_gcs.enabled == True:
			#Lmit Rate of GCS Updates
#boottime = int((curtime-starttime)*1000)
			update_device(device_gcs)
			if ((boottime - device_gcs.last_update)>(1/device_gcs.update_rate*1000.0)):
				device_gcs.last_update = boottime
				device_gcs.display()
				#print "Actual GCS Update Rate: {:.2f} Hz".format(1000.0/(boottime-gcs_last_send_time))
                        	#send_heartbeat(device_gcs,mavlink.MAV_STATE_ACTIVE)
				send_heartbeat(device_gcs,device_pc.state)
				send_position(device_gcs.device,boottime,GPS_Lat_dec,GPS_Long_dec,GPS_Alt,GPS_Alt,0,0,0,0)
				send_attitude(device_gcs.device,boottime,Current_Roll_rad,Current_Pitch_rad,Current_Yaw_rad,0,0,0)
		if device_mc.enabled == True:
			update_device(device_mc)
			if ((boottime - device_mc.last_update)>(1/device_mc.update_rate*1000.0)):
				device_mc.last_update = boottime
				#device_mc.changemode(mavlink.MAV_MODE_TEST_DISARMED)
				device_mc.display()
				#device_mc.display_errors()
				send_heartbeat(device_mc,device_pc.state)
				device_mc.armdisarm(True)
				device_mc.senddist(min_dist_sector_in)
				#tempstr = "$CAM,DIST,10,20,30,40,50,60,70,80,90"+"*\r\n"	
				#device_mc.device.write(tempstr)
				
						
		if device_remote.enabled == True:
			update_device(device_remote)
			if ((boottime - device_remote.last_update)>(1/device_remote.update_rate*1000.0)):
				device_remote.last_update = boottime
				device_remote.display()
				send_heartbeat(device_remote,device_pc.state)
				send_attitude(device_remote.device,boottime,Current_Roll_rad,Current_Pitch_rad,Current_Yaw_rad,0,0,0)
				send_position(device_remote.device,boottime,GPS_Lat_dec,GPS_Long_dec,GPS_Alt,GPS_Alt,0,0,0,0)
		if device_fcgps.enabled == True:
			
			if ((boottime - device_fcgps.last_update)>(1/device_fcgps.update_rate*1000.0)):
				device_fcgps.last_update = boottime
				device_fcgps.display()
				
				device_fcgps.device.write(GPRMC)
				pub_data_to_fc_gps.publish(GPRMC)
				device_fcgps.device.write(GPVTG)
				pub_data_to_fc_gps.publish(GPVTG)
				device_fcgps.device.write(GPGGA)
				pub_data_to_fc_gps.publish(GPGGA)
		
		#Assume Pre-Flight Checks have been completed.  Go to MANUAL_DISARMED Mode and MAV_STATE_STANDBY if No Errors
		#if device_pc.mode == mavlink.MAV_MODE_PREFLIGHT:
		#	device_pc.changemode(mavlink.MAV_MODE_MANUAL_DISARMED)
		#if device_pc.errors[len(device_pc.errors)-1] == "10000":
		#	device_pc.changemode(mavlink.MAV_MODE_STABILIZE_DISARMED)
		#	device_pc.display()
		#else:
		#	dumb = 1
		#	#print "Can't set {} to STANDBY.".format(device_pc.name)
		#if device_fc.state == mavlink.MAV_STATE_ACTIVE:
		#	if trigger == True:
		#		print "Now trying"
		#		trigger = False
		#		device_fc.changemode(mavlink.MAV_MODE_AUTO_ARMED)
		#else:
		#	print "Not ready yet"
		


def init_device(m):
	if m == device_fc.device:
		'''wait for a heartbeat so we know the target system IDs'''
	    	device_fc.printtext("Waiting for Flight Controller heartbeat")
	    	m.wait_heartbeat()
	    	device_fc.printtext("Heartbeat from Flight Controller (system %u component %u)" % (m.target_system, m.target_component))
		device_fc.printtext("Sleeping for 3 seconds to allow system, to be ready")
		rospy.sleep(3)
		device_fc.printtext("Sending all stream request for rate %u" % 10)

		m.mav.request_data_stream_send(device_fc.device.target_system, device_fc.device.target_component,
		                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
	if m == device_gcs.device:
		#wait for heartbeat
		m.wait_heartbeat()
		

def update_device(m):
	global WaypointCount
	global Current_Yaw_rad
	global Current_Pitch_rad
	global Current_Roll_rad
	global fc_badpacket_counter
	global first_attitude_packet
	global Initial_Yaw_rad
	global my_MissionItems
	if (m.protocol == "MAVLINK") or (m.protocol == "APM_MAVLINK"): 
		msg = m.device.recv_match(blocking=False)
		#pub_data_from_fc.publish(str(msg.get_type()))
		if m.device == device_remote.device:
			if msg:
				#print msg
				device_remote.printtext(str(msg))
		if m.device == device_fc.device:
			print msg
		#if m == device_gcs.device:
		#	if msg:
		#		print msg
		if msg:
			if msg.get_type() == "BAD_DATA":            
				fc_badpacket_counter = fc_badpacket_counter + 1
			    	if mavutil.all_printable(msg.data):
					sys.stdout.write(msg.data)
					sys.stdout.flush()			  
			elif msg.get_type() == "HEARTBEAT": 
				if m.device == device_gcs.device:
					device_gcs.appenderror(calc_errorcode(SYSTEM_GCS,ERRORTYPE_NOERROR,SEVERITY_INFORMATION,MESSAGE_NOERROR))
					#device_gcs.display_errors()
					#print "device_gcs.device: {}".format(msg)
				if m.device == device_fc.device:
					device_gcs.appenderror(calc_errorcode(SYSTEM_GCS,ERRORTYPE_NOERROR,SEVERITY_INFORMATION,MESSAGE_NOERROR))
					device_fc.state = msg.system_status
					device_fc.mode = msg.base_mode
					#print "FC: {}".format(msg)
				
					#fc_state = msg.system_status
				if m.device == device_remote.device:
					device_remote.printtext("Remote: {}".format(msg))
					device_remote.appenderror(calc_errorcode(SYSTEM_REMOTE,ERRORTYPE_NOERROR,SEVERITY_INFORMATION,MESSAGE_NOERROR))			
			elif msg.get_type() == "ATTITUDE" :
				if first_attitude_packet:
					Initial_Yaw_rad = msg.yaw
					first_attitude_packet = False
				Current_Pitch_rad = msg.pitch
				Current_Yaw_rad = msg.yaw
				Current_Roll_rad = msg.roll
				pub_attitude.publish(msg.roll, msg.pitch, msg.yaw)
			elif msg.get_type() == "STATUSTEXT":
				dumb = 1
				#print msg
			elif msg.get_type() == "MISSION_CURRENT":
				if m == device_fc.device:
					dumb = 1
					#print "FC: {}".format(msg)			

			elif msg.get_type() == "MISSION_ITEM":
				if m.device == device_gcs.device:
					device_gcs.printtext(str(msg))
					waypoint_rcv_fsm(m,"NewWP",msg)
				elif m.device == device_fc.device:
					device_fc.printtext(str(msg))

			elif msg.get_type() == "MISSION_ACK":
				if m.device == device_gcs.device:
					waypoint_rcv_fsm(m,"Finish",msg)
				
				if m.device == device_fc.device:
					device_fc.printtext(str(msg))
					send_mission_request_list(device_fc.device)
			elif msg.get_type() == "GPS_RAW_INT":
				dumb = 1
			elif msg.get_type() == "RADIO":
				dumb = 1
			elif msg.get_type() == "AHRS":
				dumb = 1
			elif msg.get_type() == "HWSTATUS":
				dumb = 1
			elif msg.get_type() == "SYS_STATUS":
				dumb = 1
				#if m == fc:
				#	print msg
			elif msg.get_type() == "NAV_CONTROLLER_OUTPUT":
				dumb = 1
				#if m == fc:
				#	print msg
			elif msg.get_type() == "MEMINFO":
				dumb = 1
			elif msg.get_type() == "REQUEST_DATA_STREAM":
				if m.device == device_remote.device:
					device_remote.mav_data_streams.append(msg.req_stream_id)
					#device_remote.display_streams()
					device_remote.appenderror(calc_errorcode(SYSTEM_REMOTE,ERRORTYPE_NOERROR,SEVERITY_INFORMATION,MESSAGE_NOERROR))

			elif msg.get_type() == "GLOBAL_POSITION_INT":
				dumb = 1
			elif msg.get_type() == "RC_CHANNELS_SCALED":
				dumb = 1
			elif msg.get_type() == "SERVO_OUTPUT_RAW":
				dumb = 1
			elif msg.get_type() == "RC_CHANNELS_RAW":
				dumb = 1
			elif msg.get_type() == "VFR_HUD":
				dumb = 1
			elif msg.get_type() == "RAW_IMU":
				dumb = 1
			elif msg.get_type() == "SCALED_PRESSURE":
				dumb = 1
			elif msg.get_type() == "SENSOR_OFFSETS":
				dumb = 1
			elif msg.get_type() == "MISSION_REQUEST_LIST":
				for wp in my_MissionItems:
					wp.display()
				if m.device == device_gcs.device:
					device_pc.printtext("Going to send these waypoints: ")
				
					device_pc.printtext(str(msg))
					try:
						if (len(my_MissionItems)>0):
						
							waypoint_xmit_fsm(device_gcs.device,"Start",msg)
						else:
							waypoint_xmit_fsm(device_gcs.device,"Empty",msg)
					
					except NameError:
						dumb = 1
						#waypoint_xmit_fsm(device_gcs.device,"Empty",msg)
						#print "ERRORORRR"
				elif m.device == device_fc.device:
					device_fc.printtext(str(msg))
			elif msg.get_type() == "MISSION_REQUEST":
				if m.device == device_gcs.device:
					device_gcs.printtext(str(msg))
					waypoint_xmit_fsm(device_gcs.device,"NewWP",msg)
				elif m.device == device_fc.device:
					waypoint_xmit_fsm(device_fc.device,"NewWP",msg)
				
			elif msg.get_type() == "MISSION_CLEAR_ALL":
				if m.device == device_gcs.device:
					device_gcs.device.target_system = 0
					device_gcs.device.target_component = 0
					device_gcs.printtext(str(msg))
					my_MissionItems = []
					send_ack(device_gcs.device)
				elif m.device == device_fc.device:
					device_fc.printtext(str(msg))	
			elif msg.get_type() == "MISSION_COUNT":
				if m.device == device_gcs.device:
					device_gcs.printtext(str(msg))
					device_gcs.device.target_system = 0
					device_gcs.device.target_component = 0
					WaypointCount = int(msg.count)
					my_MissionItems = []		
				
					waypoint_rcv_fsm(device_gcs.device,"Start",msg)
				elif m.device == device_fc.device:
					device_fc.printtext(str(msg))
			elif msg.get_type() == "COMMAND_LONG":
				if m.device == device_gcs.device:
					print msg
				
					device_gcs.printtext(str(msg))
					if msg.command == mavlink.MAV_CMD_NAV_LAND:
						if device_pc.isarmed():
							device_gcs.printtext("MAV_CMD_NAV_LAND")
							device_pc.changecommand(msg.command)
							print "Need more code here"
						else:
							tempstr = "not even armed yet"
							send_text(device_gcs.device,tempstr)
							print tempstr
					
					elif msg.command == mavlink.MAV_CMD_NAV_TAKEOFF:
						if device_pc.isarmed():
							device_gcs.printtext("MAV_CMD_NAV_TAKEOFF")
							device_pc.changecommand(msg.command)
							#statemode_fsm(device_fc.device,mavlink.MAV_STATE_ACTIVE,mavlink.MAV_MODE_AUTO_ARMED)
						else:
							tempstr =  "Not even armed yet."
							send_text(device_gcs.device,tempstr)
							print tempstr
						
					elif msg.command == mavlink.MAV_CMD_OVERRIDE_GOTO:
						device_gcs.printtext("MAV_CMD_OVERRIDE_GOTO")
						if (str(msg.confirmation) == "1") and(str(msg.param1) == "0.0") and (str(msg.param2) == "2.0"):						
							if device_pc.isarmed():
								#device_pc.changecommand(msg.command)
								print "Need more code here"
								print "PAUSE"
							else:
								tempstr =  "Not even armed yet."
								send_text(device_gcs.device,tempstr)
								print tempstr						

						if (str(msg.confirmation) == "1") and (str(msg.param1) == "1.0") and (str(msg.param2) == "2.0"):	
							if device_pc.isarmed():
								print "Need mroe code here"
							else:
								tempstr =  "Not even armed yet."
								send_text(device_gcs.device,tempstr)
								print tempstr
				if m.device == device_fc.device:
					device_fc.printtext(str(msg))
			elif msg.get_type() == "SET_MODE":
				if m.device == device_gcs.device:
					print msg
					device_gcs.printtext(str(msg))
					print msg.base_mode
					print device_pc.isarmed()
					if str(msg.base_mode) == "132":
						if device_pc.isarmed() == False:
							print "trying to arm"
							device_pc.armdisarm(True)
							send_ack(device_gcs.device)
						else:
							device_pc.armdisarm(False)
					elif str(msg.base_mode) == "4":
						if device_pc.isarmed() == True:
							print "trying to disarm"
							device_pc.armdisarm(False)
							send_ack(device_gcs.device)
					else:
						if device_pc.isarmed() == True:
							device_pc.armdisarm(False)
						device_pc.changemode(msg.base_mode)
						if device_fc.enabled == True:
							device_fc.changemode(msg.base_mode)
				if m.device == device_remote.device:
					if device_pc.isarmed() == True:
						device_pc.armdisarm(False)
					if msg.custom_mode == APM_STABILIZE:
						device_pc.changemode(mavlink.MAV_MODE_STABILIZE_DISARMED)
						#device_pc.armdisarm(True)
					elif msg.custom_mode == APM_AUTO:
						device_pc.changemode(mavlink.MAV_MODE_AUTO_DISARMED)
					elif msg.custom_mode == APM_LAND:
						device_pc.changecommand(mavlink.MAV_CMD_NAV_LAND)
					elif msg.custom_mode == APM_LOITER:
						device_pc.changecommand(mavlink.MAV_CMD_NAV_LOITER_UNLIM)
					elif msg.custom_mode == APM_GUIDED:
						device_pc.changemode(mavlink.MAV_MODE_GUIDED_DISARMED)
			else:
				if m.device == device_gcs.device:
					device_gcs.printtext(str(msg))
				if m.device == device_remote.device:
					device_remote.printtext(str(msg))
				if m.device == device_fc.device:
					dumb = 1
					#print msg
				
			if msg == "STATUSTEXT":
				if m.device == device_fc.device:
					if (msg.find("flight plan received")>0):
						dumb = 1
						device_fc.printtext(str(msg))
				if m.device == device_fc.device:
					dumb = 1
					device_fc.printtext(str(msg))
	if m.protocol == "ICARUS":
		try:
			msg = device_mc.device.readline()
			
			#device_mc.printtext(msg)
			if msg[0] == "$" and msg[len(msg)-3]=="*":
				#print msg
				msg = msg[1:len(msg)-3]		
				contents = msg.split(",")
				if contents[0] == "STA":
					if contents[1] == "STATE":
						device_mc.state = int(contents[2])
						if device_mc.state == mavlink.MAV_STATE_ACTIVE:
							device_mc.armed = True
						else:
							device_mc.armed = False
						#print device_mc.state
					elif contents[1] == "MODE":
						device_mc.cur_mode = int(contents[2])
						#print device_mc.mode
						
				elif contents[0] == "ERR":
					device_mc.appenderror(contents[1])
				elif contents[0] == "CAL":
					dumb = 1
				elif contents[0] == "CON":
					dumb = 1
				elif contents[0] == "INF":
					dumb = 1
				elif contents[0] == "MOTOR":
					dumb = 1
				elif contents[0] == "NET":
					dumb = 1
				elif contents[0] == "SEN":
					dumb = 1
				elif contents[0] == "SRV":
					dumb = 1
				elif contents[0] == "PWMIN":
					print msg
				else:
					print msg
		except:
			device_mc.appenderror(calc_errorcode(system=SYSTEM_FLYER_MC,errortype=ERRORTYPE_COMMUNICATION,severity=SEVERITY_CAUTION,message=MESSAGE_DROPPEDPACKETS))
			
		

def statemode_fsm(fc_device,target_fc_state,target_fc_mode):
	if pc_state == mavlink.MAV_STATE_BOOT:
		device_pc.printtext("Still Booting")
	elif pc_state == mavlink.MAV_STATE_STANDBY:
		if fc_mode == 81:
			device_pc.printtext("Ready to Activate")
			send_mode(device_fc.device,mavlink.MAV_STATE_STANDBY)
			#send_mode(device_mc.device,mavlink.MAV_STATE_STANDBY)
		else:
			device_pc.printtext("Not Ready")
		
			
#def mode_send(m,mode):
#	m.mav.set_mode_send(m.target_system,mode,0)
def waypoint_xmit_fsm(m,state,msg):
	global my_MissionItems
	if m == device_gcs.device:
		if state == "Start":
			send_mission_count(device_gcs.device,len(my_MissionItems))
		if state == "NewWP":
			m.mav.mission_item_send(0,0,my_MissionItems[int(msg.seq)].seq,my_MissionItems[int(msg.seq)].frame,my_MissionItems[int(msg.seq)].command,my_MissionItems[int(msg.seq)].current,my_MissionItems[int(msg.seq)].autocontinue,my_MissionItems[int(msg.seq)].param1,my_MissionItems[int(msg.seq)].param2,my_MissionItems[int(msg.seq)].param3,my_MissionItems[int(msg.seq)].param4,my_MissionItems[int(msg.seq)].x,my_MissionItems[int(msg.seq)].y,my_MissionItems[int(msg.seq)].z)
		if state == "Empty":
			send_mission_count(device_gcs.device,0)
			
	elif m == device_fc.device:
		if state == "Start":
			device_pc.printtext("Trying to send {} waypoints to FC".format(len(my_MissionItems)))
			send_mission_count(device_fc.device,len(my_MissionItems))
		if state == "NewWP":
			device_pc.printtext("Sending Waypoint	{}".format(int(msg.seq)))		
			send_mission_item(device_fc.device,my_MissionItems[int(msg.seq)].seq,my_MissionItems[int(msg.seq)].x,my_MissionItems[int(msg.seq)].y,my_MissionItems[int(msg.seq)].z)			

def waypoint_rcv_fsm(m,state,msg):
	global WaypointCount
	global my_MissionItems	
	if m == device_gcs.device:
		if (len(my_MissionItems) == WaypointCount) and (len(my_MissionItems) > 0):
			state = "Finish"
			
		if state == "Start":
		
			send_mission_item_request(device_gcs.device,0)
		if state == "NewWP":

			index = int(msg.seq)
			my_MissionItems.append(missionitem())
			my_MissionItems[index].seq = msg.seq
			my_MissionItems[index].frame = msg.frame
			my_MissionItems[index].command = msg.command
			my_MissionItems[index].current = msg.current
			my_MissionItems[index].autocontinue = msg.autocontinue
			my_MissionItems[index].param1 = msg.param1
			my_MissionItems[index].param2 = msg.param2
			my_MissionItems[index].param3 = msg.param3
			my_MissionItems[index].param4 = msg.param4
			my_MissionItems[index].x = msg.x
			my_MissionItems[index].y = msg.y
			my_MissionItems[index].z = msg.z
			#my_MissionItems[index].display()
			device_pc.printtext("New WP: {}".format(my_MissionItems[index].seq))
			send_mission_item_request(device_gcs.device,msg.seq+1)
		if state == "Finish":
			send_ack(m)
			for mission in my_MissionItems:
				device_pc.printtext(mission.seq)
			if device_fc.enabled == True:
				waypoint_xmit_fsm(device_fc.device,"Start",msg)
	elif m == device_fc.device:
		dumb = 1
def send_text(m,text):
	m.mav.statustext_send(100,text)
def send_mode(m,mode):
	if m.protocol == "MAVLINK" or m.protocol == "APM_MAVLINK":	
		m.mav.set_mode_send(m.target_system,mode,0)
	elif m.protocol == "ICARUS":
		tempstr = "$CON,MODE," + str(mode) + "*\r\n"
		m.device.write(tempstr)
		
def send_mission_request_list(m):
	m.mav.mission_request_list_send(m.target_system, m.target_component)		
def send_mission_count(m,count):
	m.mav.mission_count_send(m.target_system, m.target_component, count)

def send_mission_item_request(m,num):
	m.mav.mission_request_send(m.target_system,m.target_component,num)
def send_ack(m):		
	m.mav.mission_ack_send(m.target_system,m.target_component,0)
	
def rcv_mission_items(m):
	m.mav.mission_request_send(m.target_system,m.target_component,0)
			
def send_mission_item(m,seq,lat,lon,alt):
	m.mav.mission_item_send(m.target_system,m.target_component,seq,mavlink.MAV_FRAME_GLOBAL,16,1,1,0,0,0,0,lat,lon,alt)
	#m.mav.mission_set_current_send(m.target_system,m.target_component,0)

def send_heartbeat(m,status):
	if m.protocol == "MAVLINK" or m.protocol == "APM_MAVLINK":
  		m.device.mav.heartbeat_send(type=mavlink.MAV_TYPE_QUADROTOR,autopilot=mavlink.MAV_AUTOPILOT_GENERIC, base_mode=4, custom_mode=0, system_status=status)
	elif m.protocol == "ICARUS":		
		tempstr = "$NET,HRTBT,{}".format(int(m.heartbeat_number)) + "*\r\n"	
		m.device.write(tempstr)
		m.heartbeat_number = m.heartbeat_number + 1
		if m.heartbeat_number > m.max_heartbeats:
			m.heartbeat_number = 0
def send_position(m,timeboot,latitude,longitude,altitude,del_altitude,vel_x,vel_y,vel_z,heading):
	m.mav.global_position_int_send(time_boot_ms=timeboot,lat=int(latitude*1e7),lon=int(-longitude*1e7),alt=int(altitude*1000),relative_alt=int(del_altitude*1000),vx=33,vy=44,vz=55,hdg=12)

def send_attitude(m,timeboot,curroll,curpitch,curyaw,curroll_speed,curpitch_speed,curyaw_speed):
	m.mav.attitude_send(time_boot_ms=timeboot, roll=curroll, pitch=curpitch, yaw=curyaw, rollspeed=curroll_speed, pitchspeed=curpitch_speed, yawspeed=curyaw_speed)

def mav_control(roll,pitch,yaw,thrust):
    	device_remote.device.mav.set_roll_pitch_yaw_thrust_send(device_remote.device.target_system, device_remote.device.target_component,roll,pitch,yaw,thrust)
def send_rc(m,data):
    	m.mav.rc_channels_override_send(m.target_system, m.target_component,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7])
    	device_pc.printtext(("sending rc: %s"%data))

def initvariables():
	global Current_Yaw_rad
	global Current_Pitch_rad
	global Current_Roll_rad
	global fc_badpacket_counter
	global startime
	global imagenum
	global depth_image
	global color_image
	global mouse_x
	global mouse_y
	global high
	global DEPTH_CAMERA_HEIGHT
	global DEPTH_CAMERA_WIDTH
	global DEPTH_IMAGE_METERS_TO_GRAY
	global num_condensed_array_rows
	global num_condensed_array_cols				
	global max_dist_sector_in
	global min_dist_sector_in
	global lasttime_depth
	global mask_image
	lasttime_depth = 0
	num_condensed_array_rows = 3
	num_condensed_array_cols = 3
	max_dist_sector_in = np.zeros(num_condensed_array_rows*num_condensed_array_cols)
	min_dist_sector_in = np.zeros(num_condensed_array_rows*num_condensed_array_cols)
	mask_image = np.ones((480,640))
	mask_image[375:480,0:640] = np.NaN
	mask_image[250:375,150:375] = np.NaN
	DEPTH_IMAGE_METERS_TO_GRAY = 1.0/4.0
	DEPTH_CAMERA_HEIGHT = -1
	DEPTH_CAMERA_WIDTH = -1
	depth_image_max_intensity = 1
	depth_image_scale = 24
	high = 0
	mouse_x = 0
	mouse_y = 0
	imagenum = 0
	starttime = 0
	timelastsend = 0
	Current_Yaw_rad = 0.0
	Current_Pitch_rad = 0.0
	Current_Roll_rad = 0.0
	fc_badpacket_counter = 0
	WaypointCount = 0	
	
def dec2gpsdeg(num):
  #a = [0, 0, 0]
  #a[0] = int(num)
  #a[1] = int((num*60.0) % 60)
  #a[2] = (num*3600.0)%60
  a = [0,0]
  a[0] = int(num)
  a[1] = (num*60.0)%60
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


