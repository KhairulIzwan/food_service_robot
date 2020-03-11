#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np
import time

from my_robot_common.centroidtracker import CentroidTracker

class ObjectTracker():
	def __init__(self):
		# initialize our centroid tracker and frame dimensions
		self.ct = CentroidTracker()
		(self.H, self.W) = (None, None)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Preview"

		# load our serialized model from disk
		rospy.loginfo("Loading model...")
		# net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

		# Initializing your ROS Node
		rospy.init_node('object_tracking_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic """
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)

		# Subscribe to the camera info topic """
		self.imgInfo_sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.getCameraInfo)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Refresh the image on the screen
		self.displayImg()

	# Get the width and height of the image """
	def getCameraInfo(self, msg):
		self.W = msg.width
		self.H = msg.height

	# Convert the raw image to OpenCV format """
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# OPTIONAL -- clone image
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	# Refresh the image on the screen """
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	def shutdown(self):
		try:
			rospy.loginfo("Object Tracker Node [ONLINE]...")

		finally:
			cv2.destroyAllWindows()

def main(args):
	ot = ObjectTracker()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logerr("Object Tracker Node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("Object Tracker Node [ONLINE]...")
	main(sys.argv)

