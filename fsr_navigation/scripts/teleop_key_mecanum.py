#!/usr/bin/env python

#Title: Python Publisher for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [28-01-2020]
#Description: Tank Navigation Publisher Nodes (Python) ** originally from turtlebot3_teleop

#remove or add the library/libraries for ROS
import rospy
import sys
import select
import os

#remove or add the message type
from geometry_msgs.msg import Twist

"""Add changes here!"""
from std_msgs.msg import Int32

if os.name == 'nt':
	import msvcrt
else:
	import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Food Service Robot!
---------------------------
Moving around:			Special:
q	w	e
a	s	d		,	.
z	x	c

w :	forward
x :	backward
a :	left
d :	right
q :	45 forward left
e :	45 forward right
z :	45 backward left
c :	45 backward right
, :	rotate left
. :	rotate right

space key, s : force stop
b : increase speed
n : decrease speed

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
	if os.name == 'nt':
		return msvcrt.getch()

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(target_linear_vel, target_angular_vel):
	return "currently:\tvel %s " % target_linear_vel

def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output

def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input

	return input

def checkLinearLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

	return vel

def checkAngularLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

	return vel

if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('tank_teleop')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	pub_servo_dir = rospy.Publisher('/cmd_servo_dir', Int32, queue_size=10)

	turtlebot3_model = rospy.get_param("model", "burger")

	status = 0
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0
	servoDirVal = 0

	try:
		print(msg)
		while(1):
			key = getKey()

			# increase speed (linear only!)
			if key == 'b' :
				target_linear_vel = checkLinearLimitVelocity(
					target_linear_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))

			# decrease speed (linear only!)
			elif key == 'n' :
				target_linear_vel = checkLinearLimitVelocity(
					target_linear_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_vel,target_angular_vel))

			# w :	forward
			elif key == 'w' :
				servoDirVal = 1
				status = status + 1

			# x :	backward
			elif key == 'x' :
				servoDirVal = 2
				status = status + 1

			# a :	left
			elif key == 'a' :
				servoDirVal = 3
				status = status + 1

			# d :	right
			elif key == 'd' :
				servoDirVal = 4
				status = status + 1

			# q :	45 forward left
			elif key == 'q' :
				servoDirVal = 5
				status = status + 1

			# e :	45 forward right
			elif key == 'e' :
				servoDirVal = 6
				status = status + 1

			# z :	45 backward left
			elif key == 'z' :
				servoDirVal = 7
				status = status + 1

			# c :	45 backward right
			elif key == 'c' :
				servoDirVal = 8
				status = status + 1

			# , :	rotate left
			elif key == ',' :
				servoDirVal = 9
				status = status + 1

			# . :	rotate right
			elif key == '.' :
				servoDirVal = 10
				status = status + 1

			elif key == ' ' or key == 's' :
				target_linear_vel   = 0.0
				control_linear_vel  = 0.0
				target_angular_vel  = 0.0
				control_angular_vel = 0.0
				print(vels(target_linear_vel, target_angular_vel))

			else:
				servoDirVal = 0
				if (key == '\x03'):
					break

			if status == 20 :
				print(msg)
				status = 0

			twist = Twist()

			control_linear_vel = makeSimpleProfile(
					control_linear_vel, target_linear_vel, 
					(LIN_VEL_STEP_SIZE/2.0))

			twist.linear.x = control_linear_vel
			twist.linear.y = 0.0
			twist.linear.z = 0.0

			control_angular_vel = makeSimpleProfile(
					control_angular_vel, target_angular_vel,
					(ANG_VEL_STEP_SIZE/2.0))

			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = control_angular_vel

			pub.publish(twist)

			servoDir = Int32()
			servoDir.data = servoDirVal
			pub_servo_dir.publish(servoDir)

	except:
		print(e)

	finally:
		twist = Twist()

		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0

		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0

		pub.publish(twist)

		servoDir = Int32()
		servoDir.data = servoDirVal
		pub_servo_dir.publish(servoDir)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
