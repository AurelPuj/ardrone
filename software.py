#!/usr/bin/env python

from __future__ import print_function

import sys
import cv2
import roslib
import dlib

# ROS library
roslib.load_manifest('my_pkg')

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus
import time
import threading
#import face_recognition
# Some Constants
COMMAND_PERIOD = 10 #ms
#camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH , 352);
camera.set(cv2.CAP_PROP_FRAME_HEIGHT , 288);

tracker = dlib.correlation_tracker()

# Create the haar cascade
faceCascade = cv2.CascadeClassifier('/home/aurelien/catkin_ws/src/my_pkg/src/haarcascade_frontalface_default.xml')

class image_converter(threading.Thread):

	def __init__(self):
		self.trackingFace = 0
		self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=100)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
		self.x_decal = 0
		self.z_decal = 0


	def callback(self,data):

		rectangleColor = (0,165,255)

		try: 
			camera_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#ret, camera_frame = camera.read()
			#(rows,cols,channels) = camera_frame.shape
			camera_frame=cv2.resize(camera_frame,(150, 75))
			gray = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
			(OUTPUT_SIZE_HEIGHT,OUTPUT_SIZE_WIDTH,channels) = camera_frame.shape

		except CvBridgeError as e:
			print(e)


		if not self.trackingFace:

			maxArea = 0
			x = 0
			y = 0
			w = 0
			h = 0

			faces = faceCascade.detectMultiScale(gray, 1.05, 8)

			for (_x,_y,_w,_h) in faces:
				if  _w*_h > maxArea:
				    x = _x
				    y = _y
				    w = _w
				    h = _h
				    maxArea = w*h

	    #If one or more faces are found, draw a rectangle around the
	    #largest face present in the picture
			if maxArea > 0 :

				#Initialize the tracker
				tracker.start_track(camera_frame,dlib.rectangle( x-10,y-20,x+w+10,y+h+20))

				self.x_decal = ((OUTPUT_SIZE_WIDTH / 2) - (x+w/2))
				self.z_decal = ((OUTPUT_SIZE_HEIGHT / 2)- (y+h/2))

				#Set the indicator variable such that we know the
				#tracker is tracking a region in the image
				self.trackingFace = 1


		#Check if the tracker is actively tracking a region in the image
		if self.trackingFace:

		    #Update the tracker and request information about the
		    #quality of the tracking update
			trackingQuality = tracker.update(camera_frame)

		    #If the tracking quality is good enough, determine the
		    #updated position of the tracked region and draw the
		    #rectangle
			if trackingQuality >= 8:
		    	
				tracked_position =  tracker.get_position()

				t_x = int(tracked_position.left())
				t_y = int(tracked_position.top())
				t_w = int(tracked_position.width())
				t_h = int(tracked_position.height())
				cv2.rectangle(camera_frame, (t_x, t_y),(t_x + t_w , t_y + t_h),rectangleColor ,2)
				self.x_decal = ((OUTPUT_SIZE_WIDTH /2)-(t_x + t_w/2))
				self.z_decal = (((OUTPUT_SIZE_HEIGHT /2)-t_y + t_h/2))


			else:
		        #If the quality of the tracking update is not
		        #sufficient (e.g. the tracked region moved out of the
		        #screen) we stop the tracking of the face and in the
		        #next loop we will find the largest face in the image
		        #again
				self.trackingFace = 0
				self.x_decal = 0
				self.z_decal = 0
		
		cv2.imshow("Camera", camera_frame)
		cv2.waitKey(3)

class BasicDroneController():

	def __init__(self,arg):
		# Holds the current drone status
		self.status = -1
		self.velox = 0
		self.veloy = 0
		self.veloz = 0
		self.arg = arg
		self.init = 0
		self.t=0

		# Subscribe to the /ardrone/navdata topic, of message type navdata,
		# and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand = rospy.Publisher('/ardrone/land',Empty,queue_size=100)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=100)
		self.pubReset = rospy.Publisher('/ardrone/reset',Empty,queue_size=100)
		self.pubCalib = rospy.Publisher('/ardrone/imu/calibrate',Empty,queue_size=100)

		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.velox = navdata.vx
		self.veloy = navdata.vy
		self.veloz = navdata.vz


	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if self.status == DroneStatus.Landed:	
			self.pubTakeoff.publish(Empty())

	def SendCalib(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubCalib.publish(Empty())
	
	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendReset(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubReset.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self, arg, land=0):
		# Called by the main program to set the current command		

		if arg.trackingFace == 0 or self.init == 0 or land == 1:

			self.command.linear.x = 0
			self.command.linear.y = 0
			self.command.linear.z = 0
			self.command.angular.z = 0
			self.init = 1

		elif arg.x_decal > 0 and self.command.linear.y != 0.1 and time.time() - self.t < 2:
			self.t = time.time()
			self.command.linear.y = 0.1

		elif arg.x_decal < 0 and self.command.linear.y != -0.1 and time.time() - self.t < 2:
			self.t = time.time()
			self.command.linear.y = -0.1
		
		elif time.time() - self.t < 1.20:
			self.command.linear.y = 0

		else :
			self.t = time.time()

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)


def main(args):
	
	rospy.init_node('Control', anonymous=True)

	ic = image_converter()
	
	controller = BasicDroneController(ic)
		
	t= time.time()
	while time.time()-t < 10 :
		controller.SendTakeoff()

	stop = 0
	while stop != 1:
		controller.SetCommand(ic)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') :
        	stop = 1

	t= time.time()	
	while time.time()-t < 10:
		controller.SetCommand(ic,1)
		controller.SendLand()
	
	try:
		rospy.spin()

	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
