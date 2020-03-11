#!/usr/bin/env python

#Title: Python Subscriber for Tank Color Calibration (Selection for HSV Range)
#Author: Khairul Izwan Bin Kamsani - [12-02-2020]
#Description: Tank Color Range Subcriber Nodes (Python)

from __future__ import print_function
from __future__ import division

import sys
import rospy
import os
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

class range_detector_node:
	def __init__(self):
		self.filter = "HSV"
		
		self.img = np.zeros((240,320,3), np.uint8)

		# Initializing your ROS Node
		rospy.init_node('color_range_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Color Range Apps Node"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)

		# Publish as color range topic
		# self.rangeColor_pub = rospy.Publisher("/color_range", IntList, queue_size=10)

	def callback(self, data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Determine the range_filter using the setup_trackbars() helper function
		self.setup_trackbars()

		# Convert the image colorspace
		self.cvtColorspace()

		# Extract the require color value
		self.get_trackbar_values()

		# Publish Data
		# self.pubData()

		# Threshold the image
		self.imgThresh()

		# Refresh the image on the screen
		self.displayImg()

	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

		except CvBridgeError as e:
			print(e)

	def setup_trackbars(self):
		self.cv_window_trackbar = "Trackbars"
		cv2.namedWindow(self.cv_window_trackbar, 0)

		for i in ["MIN", "MAX"]:
			v = 0 if i == "MIN" else 255

			for j in self.filter.upper():
				cv2.createTrackbar("%s_%s" % (j, i), self.cv_window_trackbar, v, 255, self.callback_trackbars)

	def callback_trackbars(self, value):
		pass

	def get_trackbar_values(self):
		self.values = []

		for i in ["MIN", "MAX"]:
			for j in self.filter.upper():
				v = cv2.getTrackbarPos("%s_%s" % (j, i), self.cv_window_trackbar)
				self.values.append(v)

	""" Convert the image colorspace """
	def cvtColorspace(self):
		self.frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
		self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

	""" Refresh the image on the screen """
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.imshow(self.cv_window_trackbar, self.img)
		cv2.waitKey(1)

	def imgThresh(self):
		self.thresh = cv2.inRange(self.frame_to_thresh, (self.values[0], self.values[1], self.values[2]), (self.values[3], self.values[4], self.values[5]))
		self.cv_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.thresh)
		self.img[:] = [(self.values[3] - self.values[0]) // 2, (self.values[4] - self.values[1]) // 2, (self.values[5] - self.values[2]) // 2]

	def pubData(self):
		msg = IntList()
		msg.v1_min = self.values[0]
		msg.v2_min = self.values[1]
		msg.v3_min = self.values[2]
		msg.v1_max = self.values[3]
		msg.v2_max = self.values[4]
		msg.v3_max = self.values[5]
		self.rangeColor_pub.publish(msg)

	def shutdown(self):
		try:
			rospy.loginfo("Range detector node [OFFLINE]...")
		finally:
			cv2.destroyAllWindows()

#def usage():
#	print("Please specify type of color filter:")
#	print("1. RGB")
#	print("2. HSV")
#	print("%s [COLOR_FILTER]" % sys.argv[0])

def main(args):
#	vn = range_detector_node(sys.argv[1])
	vn = range_detector_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Color Range Apps node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
#	if len(sys.argv) < 1:
#		print(usage())
#		sys.exit(1)
#	else:
	rospy.loginfo("Color Range Apps node [ONLINE]...")
	main(sys.argv)
