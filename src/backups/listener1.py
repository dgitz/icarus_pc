#!/usr/bin/python
import roslib
roslib.load_manifest('rgbdslam')
import rospy
import math
import tf
import time

def mainloop():
	initiallocation = [41.8704040000,-87.6492930000,30.0]
	curlocation = initiallocation
	rospy.init_node('listening',anonymous=True)
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	curtime = time.time()
	curx = 0.0
	cury = 0.0
	curz = 0.0
	while not rospy.is_shutdown():
		tempstr = "New Latitude: {:.10f} Longitude: {:.10f} Alt: {:.4f}".format(curlocation[0],curlocation[1],curlocation[2]) 
		#print tempstr
		tempstr = "Init Latitude: {:.10f} Longitude: {:.10f} Alt: {:.4f}".format(initiallocation[0],initiallocation[1],initiallocation[2])
		print tempstr
		tempstr = "Delta(m) x: {:.4f} y: {:.4f} z: {:.4f}".format(curx,cury,curz)
		#print tempstr
		
		try:
			(pos,rot) = listener.lookupTransform('/map','/camera_link',rospy.Time(0))
			curx = pos[0]
			cury = pos[1]
			curz = pos[2]
			#curlocation[0] = curx/110540.0 + initiallocation[0]
			curlocation[0] = .1+initiallocation[0]  #Why is this line changing the value of initiallocation?
			curlocation[2] = curz #Altitude Transform from Ground Level			
			
			
		except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
			print "Error"
			#curlocation = initiallocation
			continue
if __name__ == '__main__':
        mainloop()
