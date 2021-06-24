#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270 				# Hokuyo 10LX has 270 degrees scan
DISTANCE_RIGHT_THRESHOLD = 0.5 	# (m)
VELOCITY = 0.1 					# meters per second

# Controller parameters
kp = 0.7
kd = 

# Other global variables
error = 0.0
prev_error = 0.0

def control(error):
	global kp
	global kd
	global VELOCITY

	# TO-DO: Implement controller
	# ---

	# ---

	# Set maximum thresholds for steering angles
	if steering_angle > 0.25:
		steering_angle = 0.25
	elif steering_angle < -0.25:
		steering_angle = -0.25

	print "Steering Angle is = %f" % steering_angle 

	# TO-DO: Publish the message
	# ---

	# ---

def get_index(angle, data):
	# 	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---

	# ---

def distance(angle_right, angle_lookahead, data):
	global ANGLE_RANGE
	global DISTANCE_RIGHT_THRESHOLD

	# TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
	# ---

	# ---

	print "Distance from right wall : %f" % distance_r

	# Calculate error
	error = 

	return error, distance


def follow_center(angle_right,angle_lookahead_right, data):

	angle_left = 180 + angle_right
	angle_lookahead_left = 180 - angle_lookahead_right 

	er, dr = distance(angle_right, angle_lookahead_right, data)
	el, dl = distance(angle_left, angle_lookahead_left, data)

	# Find Centerline error
	# ---

	# ---

	print "Centerline error = %f " % centerline_error

	return centerline_error

def callback(data):

	# Pick two rays at two angles
	angle_right = 
	angle_lookahead = 

	# To follow right wall
	#er, dr = distance(angle_right,angle_lookahead, data)

	# To follow the centerline
	ec = follow_center(angle_right,angle_lookahead, data)

	control(ec)

if __name__ == '__main__':
	print("Wall following started")
	rospy.init_node('wall_following',anonymous = True)

	# TO-DO: Implement the publishers and subscribers
	# ---

	# ---

	rospy.spin()
