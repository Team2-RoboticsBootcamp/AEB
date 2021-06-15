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

from matplotlib import pyplot as plt
# Vehicle parameters
ANGLE_RANGE = 270           # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 3      # Distance threshold before collision (m)
VELOCITY = 0.5              # Maximum Velocity of the vehicle
TIME_THRESHOLD = 1          # Time threshold before collision (s)
STEERING_ANGLE = 0          # Steering angle is uncontrolled


# P-Controller Parameters
kp_dist = 0.5
kp_ttc = 1

dist_error = 0.0
time_error = 0.0
velocity = VELOCITY
# counter = 0

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def dist_control(distance):
	global kp_dist
	global VELOCITY
	global DISTANCE_THRESHOLD 
	global STEERING_ANGLE
	# global counter
	# TO-DO: Calculate Distance to Collision Error
	# ---
	dist_error = max(distance-DISTANCE_THRESHOLD,0)

	velocity = min(kp_dist*dist_error,VELOCITY)
	# ---
	# if counter % 2 == 0:
	#     plt.plot(counter, distance,'.')
	#     # plt.axis("equal")
	#     plt.draw()
	#     plt.pause(0.00000000001)

	# counter += 1

	# plt.ion()
	# plt.show()

	print("Distance before collision is = ", distance)
	print("Vehicle velocity= ", velocity)
	print("Distance error= ", dist_error)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def TTC_control(distance):
	global kp_ttc
	global TIME_THRESHOLD
	global VELOCITY
	global STEERING_ANGLE
	global time_error
	global velocity
	# TO-DO: Calculate Time To Collision Error
	# ---

	velocity = min(max(velocity-kp_ttc*time_error,0),VELOCITY)
	TTC = distance/(velocity)
	

	if TTC < TIME_THRESHOLD:
		time_error = velocity
	# ---
		
	print("Time to collision in seconds is = ", TTC)
	print('time error', time_error)
	print("Vehicle velocity = ", velocity)

	msg = AckermannDriveStamped()
	msg.drive.speed = velocity
	msg.drive.steering_angle = STEERING_ANGLE
	pub.publish(msg)

def get_index(angle, data):
	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---
	# Index starts at -45 degrees robot frame and has 10 scans per degree
	# We want to get the index for +-5 degrees around the desired angle
	# angle_min=data.angle_min*180/math.pi
	# angle_max=data.angle_max*180/math.pi
	# angle_increment=data.angle_increment*180/math.pi
	# print('len', len(data.ranges))
	scans_per_degree = int(len(data.ranges)/ANGLE_RANGE)
	# print('scans_per_degree',scans_per_degree)
	additional_degrees= 5
	indexes= [int(angle+45-additional_degrees)*scans_per_degree,int(angle+45+additional_degrees)*scans_per_degree]
	# print('indexes', indexes)
	return indexes
	# ---
	
# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data): 
	global ANGLE_RANGE
	
	angle_front = 90   # Range of angle in the front of the vehicle we want to observe
	avg_dist = 0
	
	# Get the corresponding list of indices for given range of angles
	index_front = get_index(angle_front, data)

	# TO-DO: Find the avg range distance
	# ---
	#Clip the data to sensor min and max range
	data.ranges = np.clip(data.ranges, data.range_min, data.range_max)
	# print('dist1', data.ranges[index_front[0]])
	# print('dist2', data.ranges[index_front[1]])
	#Calc mean
	avg_dist = np.mean(data.ranges[index_front[0]:index_front[1]])
	# ---
	
	# print("Average Distance = ", avg_dist)

	return avg_dist

    
def callback(data):

	# TO-DO: Complete the Callback. Get the distance and input it into the controller
	avg_dist = get_distance(data)
	# dist_control(avg_dist)
	TTC_control(avg_dist)
	# ---

	# ---

if __name__ == '__main__':


	print("AEB started")
	rospy.init_node('aeb',anonymous = True)
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()
