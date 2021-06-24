#!/usr/bin/env python3

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
VELOCITY = 0.5 					# meters per second
frequency=10

# Controller parameters
kp = 0.65
kp_v=0.5
kd = 0.001

# Other global variables
error = 0.0
prev_error = 0.0

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def control(error):
	global kp
	global kd
	global VELOCITY
	global prev_error

	# TO-DO: Implement controller
	# ---
	steering_angle= kp*error+kd*(prev_error-error)
	velocity=VELOCITY-(kp_v*abs(error))
	# ---

	# Set maximum thresholds for steering angles
	if steering_angle > 0.5:
		steering_angle = 0.5
		
	elif steering_angle < -0.5:
		steering_angle = -0.5
	
	prev_error=error

	print ("Steering Angle is =",steering_angle )
	print ("Velocity=", velocity)

	# TO-DO: Publish the message
	# ---
	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = steering_angle
	pub.publish(msg)
	# ---

def get_index(angle, data):
	# 	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---
	scans_per_degree = int(len(data.ranges)/ANGLE_RANGE)
	#additional_degrees= 0
	#indexes= [int(angle+45-additional_degrees)*scans_per_degree,int(angle+45+additional_degrees)*scans_per_degree]
	indexes= int(angle+45)*scans_per_degree
	return indexes
	# ---

def distance(angle_right, angle_lookahead, data):
	global ANGLE_RANGE
	global DISTANCE_RIGHT_THRESHOLD
	# TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
	# ---
	theta= ((45* math.pi)/180)
	b_ind = get_index(angle_right, data)
	a_ind = get_index(angle_lookahead, data)
	alpha = math.atan(((data.ranges[a_ind] * math.cos(theta)) - data.ranges[b_ind]) / (data.ranges[a_ind] * math.sin(theta)))
	distance_r= data.ranges[b_ind] * math.cos(alpha)
	
	# ---

	print ("Distance from right wall :", distance_r)
	print ("Alpha:", alpha)
	# Calculate error
	error = DISTANCE_RIGHT_THRESHOLD - (distance_r + (((1/frequency)*VELOCITY)* math.sin(alpha)))
	print ("error:", error)

	return error, distance_r


def follow_center(angle_right,angle_lookahead_right, data):

	angle_left = 180 + angle_right
	angle_lookahead_left = 180 - angle_lookahead_right 

	er, dr = distance(angle_right, angle_lookahead_right, data)
	el, dl = distance(angle_left, angle_lookahead_left, data)

	# Find Centerline error
	# ---

	# ---

	print ("Centerline error =",centerline_error)

	return centerline_error

def callback(data):

	# Pick two rays at two angles
	angle_right = 0
	angle_lookahead = 45

	# To follow right wall
	er, dr = distance(angle_right,angle_lookahead, data)

	# To follow the centerline
	#ec = follow_center(angle_right,angle_lookahead, data)

	control(er)

if __name__ == '__main__':
	print("Wall following started")
	rospy.init_node('wall_following_t2',anonymous = True)
	rospy.Rate(frequency)
	# TO-DO: Implement the publishers and subscribers
	# ---
	rospy.Subscriber("/scan",LaserScan,callback)
	# ---

	rospy.spin()
