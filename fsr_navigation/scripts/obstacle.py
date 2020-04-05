#!/usr/bin/env python

#Title: Python Subscriber for Laser Scan (RPLiDar)
#Author: Khairul Izwan Bin Kamsani - [12-02-2020]
#Description: Distance Detection Subcriber Nodes (Python)

from __future__ import print_function
from __future__ import division

#remove or add the library/libraries for ROS
import sys
import rospy
import os
import cv2
import imutils
import math

#remove or add the message type
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.01
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

e = """
Communications Failed
"""

class Obstacle():
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('tank_obstacle_node', anonymous=True)
		
		# shutdown treat
		rospy.on_shutdown(self.shutdown)

		# Subscribe to the raw camera image topic
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)

		# Publish the cmd_vel topic
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def callback(self, data):
		self.get_scan(data)

		self.obstacle()

	def get_scan(self, scan):
		self.scan_filter = []

		samples = len(scan.ranges)
		print(samples)

		samples_view = 1

		if samples_view > samples:
			samples_view = samples

		if samples_view is 1:
			self.scan_filter.append(scan.ranges[0])

		else:
			left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
			right_lidar_samples_ranges = samples_view//2

			left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
			right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
			self.scan_filter.extend(left_lidar_samples + right_lidar_samples)

		for i in range(samples_view):
			if self.scan_filter[i] == float('Inf'):
				self.scan_filter[i] = 3.5
			elif math.isnan(self.scan_filter[i]):
				self.scan_filter[i] = 0

	def obstacle(self):
		self.twist = Twist()
		turtlebot_moving = True

		min_distance = min(self.scan_filter)

		if min_distance < SAFE_STOP_DISTANCE:
			if turtlebot_moving:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				turtlebot_moving = False
				rospy.loginfo('Stop!')
		else:
			self.twist.linear.x = LINEAR_VEL
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			turtlebot_moving = True
			rospy.loginfo('Distance of the obstacle : %f', min_distance)

	def shutdown(self):
		try:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			rospy.loginfo("Obstacle Detection. [OFFLINE]...")
		finally:
			pass

def main(args):
	vn = Obstacle()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Obstacle Detection. [OFFLINE]...")

if __name__ == '__main__':
	rospy.loginfo("Obstacle Detection. [ONLINE]...")
	main(sys.argv)
