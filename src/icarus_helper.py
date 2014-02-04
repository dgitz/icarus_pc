import struct,array,time,os,sys,math,string
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '/opt/ros/groovy/share/mavlink/pymavlink'))
import mavlinkv10 as mavlink
import cv2
import numpy as np
import pdb
from math import *

# Error Code Enumerations

MAX_ERRORS = 10

#Field 1: System and Subsystem
SYSTEM_FLYER = "10"
SYSTEM_FLYER_PC = "11"
SYSTEM_FLYER_FC = "12"
SYSTEM_FLYER_FCGPS = "13"
SYSTEM_FLYER_MC = "14"
SYSTEM_GCS = "50"
SYSTEM_REMOTE = "70"

#Field 2
ERRORTYPE_NOERROR = "0"
ERRORTYPE_ELECTRICAL = "1"
ERRORTYPE_SOFTWARE = "2"
ERRORTYPE_COMMUNICATION = "3"
ERRORTYPE_SENSORS = "4"
ERRORTYPE_ACTUATORS = "5"
ERRORTYPE_DATASTORAGE = "6"
ERRORTYPE_GENERALERROR = "9"

#Field 3
SEVERITY_NOERROR = "0"
SEVERITY_INFORMATION = "1"
SEVERITY_MINIMAL = "2"
SEVERITY_CAUTION = "3"
SEVERITY_SEVERE = "4"
SEVERITY_EMERGENCY = "5"

#Field 4: Message
MESSAGE_NOERROR = "0"
MESSAGE_INITIALIZING = "1"
MESSAGE_INITIALIZINGERROR = "2"
MESSAGE_GENERALERROR = "3"
MESSAGE_DROPPEDPACKETS = "4"
MESSAGE_MISSINGHEARTBEATS = "5"
MESSAGE_DEVICENOTPRESENTORAVAILABLE = "6"


#APM Mode Enumerations (SET_MODE: custom_mode)
APM_STABILIZE = 0
APM_AUTO = 3


#APM Command Enumerations (SET_MODE: custom_mode)
APM_ACRO = 1
APM_ALTHOLD = 2
APM_GUIDED = 4
APM_LOITER = 5
APM_RTL = 6
APM_CIRCLE = 7
APM_POSHOLD = 8
APM_LAND = 9
APM_TOY = 11

#Terminal Colors
TERM_RED = "31;1m"
TERM_GREEN = "32;1m"
TERM_YELLOW = "33;1m"
TERM_PURPLE = "35;1m"
TERM_BLUE = "36;1m"
TERM_WHITE = "39;20m"



def calc_errorcode(system,errortype,severity,message):
	return "{}-{}-{}-{}".format(system,errortype,severity,message)
def error_message(code):
	message = string.split(code,'-')[3]
	#print message
	if message == MESSAGE_NOERROR:
		return "NO ERROR"
	elif message == MESSAGE_INITIALIZING:
		return "INITIALIZING"
	elif message == MESSAGE_INITIALIZINGERROR:
		return "INITIALIZING ERROR"
	elif message == MESSAGE_GENERALERROR:
		return "GENERAL ERROR"
	elif message == MESSAGE_DROPPEDPACKETS:
		return "DROPPED PACKETS"
	elif message == MESSAGE_MISSINGHEARTBEATS:
		return "MISSING HEARTBEATS"
	elif message == MESSAGE_DEVICENOTPRESENTORAVAILABLE:
		return "DEVICE NOT PRESENT OR AVAILABLE"
	else:
		return "UNKNOWN"
class missionitem:
	def __init__(self):
		self.seq = None
		self.frame = mavlink.MAV_FRAME_GLOBAL #x=Latitude,y=Longitude,z=Altitude above sea level
		self.command = None
		self.current = False
		self.reached = False
		self.autocontinue = None
		self.param1 = None
		self.param2 = None
		self.param3 = None
		self.param4 = None
		self.x = 0 
		self.y = 0 
		self.z = 0 
		
	def display(self):
		print "WP: {} Current: {} x: {} y: {} z: {}".format(int(self.seq),self.current,self.x,self.y,self.z)
	def calc_distance(self,current_x,current_y):
		if self.frame == mavlink.MAV_FRAME_GLOBAL:
			R = 6371.0 #Earth Radius in km
			dx = (current_x-self.x)*(math.pi/180.0)
			dy = (current_y-self.y)*(math.pi/180.0)
			lat1 = self.x*(math.pi/180.0)
			lat2 = current_x*(math.pi/180)
			a = (math.sin(dx/2.0)*math.sin(dx/2.0))+(math.cos(dy/2.0)*math.cos(dy/2.0)*math.cos(lat1)*math.cos(lat2))
			c = 2.0*math.atan2(math.sqrt(a),math.sqrt(1.0-a))
			return R*c
	def calc_relbearing(self,current_x,current_y):
		if self.frame == mavlink.MAV_FRAME_GLOBAL:
			x1 = self.x*math.pi/180.0
			x2 = current_x*math.pi/180.0
			y1 = self.y*math.pi/180.0
			y2 = current_y*math.pi/180.0
			return (math.atan2(math.sin(y2-y1)*math.cos(x2),(math.cos(x1)*math.sin(x2)-math.sin(x1)*math.cos(x2)*math.cos(y2-y1)))%2.0*math.pi)*180.0/(math.pi)
class neuralnetwork_layer:
	def __init__(self,index,name,neurons,transferfunction,biasweights,weights):
		self.index = index
		self.name = name
		self.weights = weights
		self.biasweights = biasweights
		self.transferfunction = transferfunction
		self.neurons = neurons
		self.inputs = None
		self.y = np.zeros(neurons)
		self.z = np.zeros(neurons)
	
class neuralnetwork:
	def __init__(self,answers):
		self.type = ''
		self.answers = None
		self.layers = []
		self.classifiedanswers = answers
		
	def addlayer(self,index,name,neurons,weights,biasweights,transferfunction):
		self.layers.append(neuralnetwork_layer(index,name,neurons,transferfunction,biasweights,weights))
        def calcnet(self,inputvector):

		for l in range(0,len(self.layers)):
			
			if l == 0:
				self.layers[l].inputs = inputvector
			else:
				self.layers[l].inputs = self.layers[l-1].y
		        for nindex in range(0,self.layers[l].neurons-1):
				tempmat = self.layers[l].weights[nindex][0:len(self.layers[l].inputs)]
				pdb.set_trace()
				
				self.layers[l].z[nindex]  = np.inner(self.layers[l].inputs,tempmat)
	
				if self.layers[l].z[nindex] < -20:
					self.layers[l].y[nindex] = -1
				elif self.layers[l].z[nindex] > 20:
					self.layers[l].y[nindex] = 1
				else:
					self.layers[l].y[nindex] = 2/(1+exp(-2*self.layers[l].z[nindex]))-1
		return self.classifiedanswers[self.layers[1].y.argmax()]	
	
				
class device:
	def __init__(self,enabled,name,conn):
		self.name = name
		self.enabled = enabled
		self.conn = conn
		self.errors = []
		self.mav_comp_id = -1
		self.state = mavlink.MAV_STATE_UNINIT
		self.set_mode = mavlink.MAV_MODE_PREFLIGHT
		self.cur_mode = -1
		self.command = None
		self.protocol = ""
		self.device = ""
		self.armed = False
		self.color = TERM_WHITE
		self.mav_data_streams = [] #MAV_DATA_STREAM ID's
		self.update_rate = 1.0
		self.last_update = 0.0
		self.enableprint = False
		self.type = None
		self.heartbeat_number = 0
		self.max_heartbeats = 20
	def setcolor(self,color):
		self.color = color
	def getcolor(self):
		return self.color
	def getprotocol(self):
		return self.protocol
	def printtext(self,msg):
		if self.enableprint:
			print "\x1B[" + self.color + msg + "\x1B[" + "0m"
	def check_criticalerrors(self):
		#Entire System has to have a most recent severity error code of 2 or level to not be in a critical state.
		
		#Search backwards
		for i in range(len(self.errors)-1,-1,-1):
			tempstr = self.errors[i]
			severity = string.split(tempstr,'-')[2]
			if int(severity) < SEVERITY_CAUTION:
				return True
			else:
				print "BIG DEAL!"
				return False
		print "YO!"
		return False	
		
				
	def changecommand(self,newcommand):
		#Need command fsm code here
		self.command = newcommand
	def senddist(self,distvector):
		tempstr = "$CAM,DIST"
		for i in range(0,len(distvector)):
			tempstr = "{},{}".format(tempstr,str(int(distvector[i])))
		tempstr = tempstr + "*\r\n"
		print tempstr
		self.device.write(tempstr)
	def changemode(self,newmode):
		#Need mode fsm code here
		
		if self.protocol == "APM_MAVLINK":
			#self.device.mav.set_mode_send(self.device.target_system,0,1)
			print "not coded yet"
			self.device.mav.command_long_send(self.device.target_system,0,newmode,0,0,0,0,0,0,0,0)
		elif self.protocol == "MAVLINK":
			self.mode = newmode
			if self.device:
				print "not coded yet"
		elif self.protocol == "ICARUS":
			#self.mode = newmod
			if self.device:
				self.set_mode = newmode
				if self.set_mode <> self.cur_mode:
					
					tempstr = "$CON,MODE," + str(self.set_mode) + "*\r\n"
					self.device.write(tempstr)
			
		#self.mode = newmode
		#self.printtext("{} MODE: {}".format(self.name,self.mode))
	def isarmed(self):
		if self.armed:
			return True
		else:
			return False
	def armdisarm(self,armcmd):
		#Arming/Disarming Code here
		
		if (self.protocol == "MAVLINK"):
			
			if self.enabled:
				if self.check_criticalerrors():

					if armcmd == True:					
						if (self.state == mavlink.MAV_STATE_STANDBY) and (self.cur_mode <> mavlink.MAV_MODE_PREFLIGHT):	#Able to Arm 
							#sself.armed = True
							self.state = mavlink.MAV_STATE_ACTIVE
							if self.cur_mode == mavlink.MAV_MODE_STABILIZE_DISARMED:
								self.changemode(mavlink.MAV_MODE_STABILIZE_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_AUTO_DISARMED:
								self.changemode(mavlink.MAV_MODE_AUTO_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_MANUAL_DISARMED:
								self.changemode(mavlink.MAV_MODE_MANUAL_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_GUIDED_DISARMED:
								self.changemode(mavlink.MODE_GUIDED_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_TEST_DISARMED:
								self.changemode(mavlink.MAV_MODE_TEST_ARMED)
							else:
								print "Can't Arm, {} is not in DISARMED Mode.".format(self.name)
								self.armed = False
					elif armcmd == False:
						self.changemode(mavlink.MAV_MODE_MANUAL_DISARMED)
						self.state = mavlink.MAV_STATE_STANDBY
						self.armed = False	
						
		elif (self.protocol == "APM_MAVLINK"):
			print "Not coded yet"
		elif (self.protocol == "ICARUS"):
			if self.enabled:
				if self.check_criticalerrors():
					if armcmd == True:
						if (self.state == mavlink.MAV_STATE_STANDBY) and (self.cur_mode <> mavlink.MAV_MODE_PREFLIGHT):	#Able to Arm		
							if self.cur_mode == mavlink.MAV_MODE_STABILIZE_DISARMED:
									self.changemode(mavlink.MAV_MODE_STABILIZE_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_AUTO_DISARMED:
									self.changemode(mavlink.MAV_MODE_AUTO_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_MANUAL_DISARMED:
									self.changemode(mavlink.MAV_MODE_MANUAL_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_GUIDED_DISARMED:
									self.changemode(mavlink.MODE_GUIDED_ARMED)
							elif self.cur_mode == mavlink.MAV_MODE_TEST_DISARMED:
									self.changemode(mavlink.MAV_MODE_TEST_ARMED)
							else:
								print "Can't Arm, {} is not in DISARMED Mode.".format(self.name)
		
					
	def display(self):
		if self.type == "UAV":
			self.printtext("{}, MODE: {}, STATE: {}, CMD: {}, ARMED: {}".format(self.name,mavmode_message(self.cur_mode),mavstate_message(self.state),mavcommand_message(self.command), self.armed))
		elif self.type == "CONTROL":
			self.printtext("{}, STATUS: {}".format(self.name,error_message(self.errors[len(self.errors)-1])))
		elif self.type == "GPS":
			print "not coded yet."
		#print 
	
	def appenderror(self,code):
		self.errors.append(code)
		if len(self.errors) > MAX_ERRORS:
			del self.errors[0]
	#def clearerror(self)
	#	self.errors.clear()
	def display_errors(self):
		self.printtext("{} Error Log:".format(self.name))
		for i in range (-1,len(self.errors)-1):
			self.printtext("{}".format(self.errors[i]))
	def display_streams(self):
		for i in range(-1,len(self.mav_data_streams)):
			self.printtext("Stream: {}".format(self.mav_data_streams[i]))


def mavstate_message(code):
	if mavlink.MAV_STATE_UNINIT == code:
		return "MAV_STATE_UNINIT"
	elif mavlink.MAV_STATE_BOOT == code:
		return "MAV_STATE_BOOT"
	elif mavlink.MAV_STATE_CALIBRATING == code:
		return "MAV_STATE_CALIBRATING"
	elif mavlink.MAV_STATE_STANDBY == code:
		return "MAV_STATE_STANDBY"
	elif mavlink.MAV_STATE_ACTIVE == code:
		return "MAV_STATE_ACTIVE"
	elif mavlink.MAV_STATE_CRITICAL == code:
		return "MAV_STATE_CRITICAL"
	elif mavlink.MAV_STATE_EMERGENCY == code:
		return "MAV_STATE_EMERGENCY"
	elif mavlink.MAV_STATE_POWEROFF == code:
		return "MAV_STATE_POWEROFF"
	else:
		return "UNKNOWN"
def mavmode_message(code):
	if mavlink.MAV_MODE_PREFLIGHT == code:
		return "MAV_MODE_PREFLIGHT"
	elif mavlink.MAV_MODE_MANUAL_DISARMED == code:
		return "MAV_MODE_MANUAL_DISARMED"
	elif mavlink.MAV_MODE_MANUAL_ARMED == code:
		return "MAV_MODE_MANUAL_ARMED"
	elif mavlink.MAV_MODE_TEST_DISARMED == code:
		return "MAV_MODE_TEST_DISARMED"
	elif mavlink.MAV_MODE_TEST_ARMED == code:
		return "MAV_MODE_TEST_ARMED"
	elif mavlink.MAV_MODE_STABILIZE_DISARMED == code:
		return "MAV_MODE_STABILIZE_DISARMED"
	elif mavlink.MAV_MODE_STABILIZE_ARMED == code:
		return "MAV_MODE_STABILIZE_ARMED"
	elif mavlink.MAV_MODE_GUIDED_DISARMED == code:
		return "MAV_MODE_GUIDED_DISARMED"
	elif mavlink.MAV_MODE_GUIDED_ARMED == code:
		return "MAV_MODE_GUIDED_ARMED"
	elif mavlink.MAV_MODE_AUTO_ARMED == code:
		return "MAV_MODE_AUTO_ARMED"
	elif mavlink.MAV_MODE_AUTO_DISARMED == code:
		return "MAV_MODE_AUTO_DISARMED"
	#others
	else:
		return "UNKNOWN"

def mavcommand_message(code):
	if mavlink.MAV_CMD_NAV_ROI == code:
		return "MAV_CMD_NAV_ROI"
	elif mavlink.MAV_CMD_NAV_TAKEOFF == code:
		return "MAV_CMD_NAV_TAKEOFF"
	elif mavlink.MAV_CMD_NAV_LAND == code:
		return "MAV_CMD_NAV_LAND"
	elif mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH == code:
		return "MAV_CMD_NAV_RETURN_TO_LAUNCH"
	elif mavlink.MAV_CMD_NAV_LOITER_TIME == code:
		return "MAV_CMD_NAV_LOITER_TIME"
	elif mavlink.MAV_CMD_NAV_LOITER_TURNS == code:
		return "MAV_CMD_NAV_LOITER_TURNS"
	elif mavlink.MAV_CMD_NAV_LOITER_UNLIM == code:
		return "MAV_CMD_NAV_LOITER_UNLIM"
	elif mavlink.MAV_CMD_NAV_WAYPOINT == code:
		return "MAV_CMD_NAV_WAYPOINT"
	elif mavlink.MAV_CMD_CONDITION_YAW == code:
		return "MAV_CMD_CONDITION_YAW"
	elif mavlink.MAV_CMD_CONDITION_DISTANCE == code:
		return "MAV_CMD_CONDITION_DISTANCE"
	elif mavlink.MAV_CMD_CONDITION_CHANGE_ALT == code:
		return "MAV_CMD_CONDITION_CHANGE_ALT"
	elif mavlink.MAV_CMD_CONDITION_DELAY == code:
		return "MAV_CMD_CONDITION_DELAY"
	elif mavlink.MAV_CMD_NAV_LAST == code:
		return "MAV_CMD_NAV_LAST"
	elif mavlink.MAV_CMD_NAV_PATHPLANNING  == code:
		return "MAV_CMD_NAV_PATHPLANNING "
	elif mavlink.MAV_CMD_DO_SET_PARAMETER == code:
		return "MAV_CMD_DO_SET_PARAMETER"
	elif mavlink.MAV_CMD_DO_SET_HOME == code:
		return "MAV_CMD_DO_SET_HOME"
	elif mavlink.MAV_CMD_DO_CHANGE_SPEED == code:
		return "MAV_CMD_DO_CHANGE_SPEED"
	elif mavlink.MAV_CMD_DO_JUMP == code:
		return "MAV_CMD_DO_JUMP"
	elif mavlink.MAV_CMD_DO_SET_MODE == code:
		return ""
	elif mavlink.MAV_CMD_CONDITION_LAST == code:
		return "MAV_CMD_CONDITION_LAST"

	elif mavlink.MAV_CMD_DO_MOUNT_CONTROL == code:
		return "MAV_CMD_DO_MOUNT_CONTROL"
	elif mavlink.MAV_CMD_DO_MOUNT_CONFIGURE == code:
		return "MAV_CMD_DO_MOUNT_CONFIGURE"
	elif mavlink.MAV_CMD_DO_DIGICAM_CONTROL == code:
		return "MAV_CMD_DO_DIGICAM_CONTROL"
	elif mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE == code:
		return "MAV_CMD_DO_DIGICAM_CONFIGURE"
	elif mavlink.MAV_CMD_DO_CONTROL_VIDEO == code:
		return "MAV_CMD_DO_CONTROL_VIDEO"
	elif mavlink.MAV_CMD_DO_REPEAT_SERVO == code:
		return "MAV_CMD_DO_REPEAT_SERVO"
	elif mavlink.MAV_CMD_DO_SET_SERVO == code:
		return "MAV_CMD_DO_SET_SERVO"
	elif mavlink.MAV_CMD_DO_REPEAT_RELAY == code:
		return "MAV_CMD_DO_REPEAT_RELAY"
	elif mavlink.MAV_CMD_DO_SET_RELAY == code:
		return "MAV_CMD_DO_SET_RELAY"
	elif mavlink.MAV_CMD_MISSION_START == code:
		return "MAV_CMD_MISSION_START"
	elif mavlink.MAV_CMD_OVERRIDE_GOTO == code:
		return "MAV_CMD_OVERRIDE_GOTO"
	elif mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN == code:
		return "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN"
	elif mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS == code:
		return "MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS"
	elif mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS == code:
		return "MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS"
	elif mavlink.MAV_CMD_PREFLIGHT_CALIBRATION == code:
		return "MAV_CMD_PREFLIGHT_CALIBRATION"
	elif mavlink.MAV_CMD_DO_LAST == code:
		return "MAV_CMD_DO_LAST"
	else:
		return "UNKNOWN"

class Contour:
    ''' Provides detailed parameter informations about a contour

        Create a Contour instant as follows: c = Contour(src_img, contour)
                where src_img should be grayscale image.

        Attributes:

        c.area -- gives the area of the region
        c.parameter -- gives the perimeter of the region
        c.moments -- gives all values of moments as a dict
        c.centroid -- gives the centroid of the region as a tuple (x,y)
        c.bounding_box -- gives the bounding box parameters as a tuple => (x,y,width,height)
        c.bx,c.by,c.bw,c.bh -- corresponds to (x,y,width,height) of the bounding box
        c.aspect_ratio -- aspect ratio is the ratio of width to height
        c.equi_diameter -- equivalent diameter of the circle with same as area as that of region
        c.extent -- extent = contour area/bounding box area
        c.convex_hull -- gives the convex hull of the region
        c.convex_area -- gives the area of the convex hull
        c.solidity -- solidity = contour area / convex hull area
        c.center -- gives the center of the ellipse
        c.majoraxis_length -- gives the length of major axis
        c.minoraxis_length -- gives the length of minor axis
        c.orientation -- gives the orientation of ellipse
        c.eccentricity -- gives the eccentricity of ellipse
        c.filledImage -- returns the image where region is white and others are black
        c.filledArea -- finds the number of white pixels in filledImage
        c.convexImage -- returns the image where convex hull region is white and others are black
        c.pixelList -- array of indices of on-pixels in filledImage
        c.maxval -- corresponds to max intensity in the contour region
        c.maxloc -- location of max.intensity pixel location
        c.minval -- corresponds to min intensity in the contour region
        c.minloc -- corresponds to min.intensity pixel location
        c.meanval -- finds mean intensity in the contour region
        c.leftmost -- leftmost point of the contour
        c.rightmost -- rightmost point of the contour
        c.topmost -- topmost point of the contour
        c.bottommost -- bottommost point of the contour
        c.distance_image((x,y)) -- return the distance (x,y) from the contour.
        c.distance_image() -- return the distance image where distance to all points on image are calculated
        '''
    def __init__(self,img,cnt):
        self.img = img
        self.cnt = cnt
        self.size = len(cnt)

        # MAIN PARAMETERS

        #Contour.area - Area bounded by the contour region'''
        self.area = cv2.contourArea(self.cnt)

        # contour perimeter
        self.perimeter = cv2.arcLength(cnt,True)

        # centroid
        self.moments = cv2.moments(cnt)
        if self.moments['m00'] != 0.0:
            self.cx = self.moments['m10']/self.moments['m00']
            self.cy = self.moments['m01']/self.moments['m00']
            self.centroid = (self.cx,self.cy)
        else:
            self.centroid = "Region has zero area"

        # bounding box
        self.bounding_box=cv2.boundingRect(cnt)
        (self.bx,self.by,self.bw,self.bh) = self.bounding_box

        # aspect ratio
        self.aspect_ratio = self.bw/float(self.bh)

        # equivalent diameter
        self.equi_diameter = np.sqrt(4*self.area/np.pi)

        # extent = contour area/boundingrect area
        self.extent = self.area/(self.bw*self.bh)


        ### CONVEX HULL ###

        # convex hull
        self.convex_hull = cv2.convexHull(cnt)

        # convex hull area
        self.convex_area = cv2.contourArea(self.convex_hull)

        # solidity = contour area / convex hull area
        self.solidity = self.area/float(self.convex_area)


        ### ELLIPSE  ###
        self.ellipse = cv2.fitEllipse(cnt)

        # center, axis_length and orientation of ellipse
        (self.center,self.axes,self.orientation) = self.ellipse

        # length of MAJOR and minor axis
        self.majoraxis_length = max(self.axes)
        self.minoraxis_length = min(self.axes)

        # eccentricity = sqrt( 1 - (ma/MA)^2) --- ma= minor axis --- MA= major axis
        self.eccentricity = np.sqrt(1-(self.minoraxis_length/self.majoraxis_length)**2)


        ### CONTOUR APPROXIMATION ###

        self.approx = cv2.approxPolyDP(cnt,0.02*self.perimeter,True)


        ### EXTRA IMAGES ###

        # filled image :- binary image with contour region white and others black
        self.filledImage = np.zeros(self.img.shape[0:2],np.uint8)
        cv2.drawContours(self.filledImage,[self.cnt],0,255,-1)

        # area of filled image
        filledArea = cv2.countNonZero(self.filledImage)

        # pixelList - array of indices of contour region
        self.pixelList = np.transpose(np.nonzero(self.filledImage))

        # convex image :- binary image with convex hull region white and others black
        self.convexImage = np.zeros(self.img.shape[0:2],np.uint8)
        cv2.drawContours(self.convexImage,[self.convex_hull],0,255,-1)


        ### PIXEL PARAMETERS
      
        # mean value, minvalue, maxvalue
        self.minval,self.maxval,self.minloc,self.maxloc = cv2.minMaxLoc(self.img,mask = self.filledImage)
        self.meanval = cv2.mean(self.img,mask = self.filledImage)


        ### EXTREME POINTS ###

        # Finds the leftmost, rightmost, topmost and bottommost points
        self.leftmost = tuple(self.cnt[self.cnt[:,:,0].argmin()][0])
        self.rightmost = tuple(self.cnt[self.cnt[:,:,0].argmax()][0])
        self.topmost = tuple(self.cnt[self.cnt[:,:,1].argmin()][0])
        self.bottommost = tuple(self.cnt[self.cnt[:,:,1].argmax()][0])
        self.extreme = (self.leftmost,self.rightmost,self.topmost,self.bottommost)

    ### DISTANCE CALCULATION
  
    def distance_image(self,point=None):
      
        '''find the distance between a point and adjacent point on contour specified. Point should be a tuple or list (x,y)
            If no point is given, distance to all point is calculated and distance image is returned'''
        if type(point) == tuple:
            if len(point)==2:
                self.dist = cv2.pointPolygonTest(self.cnt,point,True)
                return self.dist
        else:
            dst = np.empty(self.img.shape)
            for i in xrange(self.img.shape[0]):
                for j in xrange(self.img.shape[1]):
                    dst.itemset(i,j,cv2.pointPolygonTest(self.cnt,(j,i),True))

            dst = dst+127
            dst = np.uint8(np.clip(dst,0,255))

            # plotting using palette method in numpy
            palette = []
            for i in xrange(256):
                if i<127:
                    palette.append([2*i,0,0])
                elif i==127:
                    palette.append([255,255,255])
                elif i>127:
                    l = i-128
                    palette.append([0,0,255-2*l])
            palette = np.array(palette,np.uint8)
            self.h2 = palette[dst]
            return self.h2
	

	
