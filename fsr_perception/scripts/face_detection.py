#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

#remove or add the library/libraries for ROS
import rospy
import sys
import cv2
import imutils
import argparse

#remove or add the message type
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest

class TankFaceDetector:

	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node("face_detector_node", anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Create the Subsciber (image_raw)
		self.sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback_image)
		
		# Create the Subsciber (camera_info)
		self.sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.callback_camerainfo)
		
		# Create the Publisher (roi)		
		self.pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

		# Path to input Haar cascade for face detection
		self.faceCascade = cv2.CascadeClassifier("/home/khairulizwan/catkin_ws/src/food_service_robot/fsr_perception/library/haarcascade_frontalface_default.xml")
		
	def callback_camerainfo(self, data):
		# Get the image width and height
		self.W = data.width
		self.H = data.height
	
	def callback_image(self, data):
		# Convert ros --> opencv
		self.convert_ros_to_opencv_img(data)
		
		# Detect face
		self.track()

		# loop over the face bounding boxes and draw them
		for rect in self.rects:
			cv2.rectangle(self.frameClone, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)

			roi=RegionOfInterest()
			roi.x_offset=rect[0]
			roi.y_offset=rect[1]
			roi.width=rect[2]
			roi.height=rect[3]

			self.pub.publish(roi)

		cv2.imshow("Face Detector", self.frameClone)
		cv2.waitKey(1)
	
	def convert_ros_to_opencv_img(self, ros_image):
		self.cv_image = self.bridge.imgmsg_to_cv2(ros_image)
		
		# Clone the original image for displaying purpose later
		self.frameClone = self.cv_image.copy()

	def track(self):
		# Create an empty arrays for save rects value later
		self.rects = []
		
		# Detect all faces in the input frame
		faceRects = self.faceCascade.detectMultiScale(self.cv_image,
			scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30),
			flags = cv2.CASCADE_SCALE_IMAGE)

		# Loop over the face bounding boxes
		for (fX, fY, fW, fH) in faceRects:
			# Extract the face ROI and update the list of bounding boxes
			faceROI = self.cv_image[fY:fY + fH, fX:fX + fW]
			self.rects.append((fX, fY, fX + fW, fY + fH))

	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

def main(args):
	tfd = TankFaceDetector()
	try:
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")

	cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.loginfo("[INFO] Tank Face Detector [ONLINE]")
	main(sys.argv)
