import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int64
from std_msgs.msg import Float32
import math
# from race.msg import drive_param
global m
m = 0
global velocity
global angle

def camera_callback(data):
    global m
    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # cv2.imshow("Original", cv_image)
    canny_image = canny(cv_image)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=1, maxLineGap=5)
    averaged_lines = average_slope_intercept(cv_image, lines)
    line_image, x_up, x_low = display_lines(cv_image, averaged_lines)
    combo_image = cv2.addWeighted(line_image, 1, cv_image, 0.6, 1)
    angle =  math.atan2((480 - 288), (x_low - x_up)) - np.pi/2
    # print(x_up, x_low, 'xs')
    if angle > 0:
        m = - 0.15
    else:
        m = 0.15
    # print(m, 'm')
    # cv2.imshow('frame',cropped_image)
    cv2.imshow('processed', cropped_image)
    cv2.moveWindow('processed',700,0)
    cv2.imshow('canny',canny_image)
    # cv2.imshow('lines', lines)
    # cv2.moveWindow('lines',1400,0)
    # cv2.imshow('averaged_lines', averaged_lines)
    # cv2.moveWindow('averaged_lines',0,500)
    cv2.imshow('combo_image', combo_image)
    #cv2.moveWindow('combo_image',1200,0)
    cv2.imshow('ROI',cropped_image)
    cv2.waitKey(1)

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(gray, 0, 150)
    return canny

def region_of_interest(image):
    h = image.shape[0]
    triangle = np.array([
    [(0, 480), (0, 288), (639, 288), (639, 480)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(mask, image)
    return masked_image

def make_coordinates(image, line_parameters):
    # if line_parameters is not None:
    slope, intercept = line_parameters
    # print(line_parameters)
    y1 = image.shape[0]
    y2 = 288

    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []


    # if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)

        slope = parameters[0]
        intercept = parameters[1]

        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))

    # if left_fit != [] and right_fit != []:
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)
    # else:
    #     left_fit_average = left_fit_average
    #     right_fit_average = right_fit_average
    return left_line, right_line

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    x_upper = 0
    x_lower = 0

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            x_lower += x1
            x_upper += x2
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10) # 10 is the line width
        x_upper = x_upper/2
        x_lower = x_lower/2

    return line_image, x_upper, x_lower


if __name__ == '__main__':
    rospy.init_node('lane_follower')
    bridge_object = CvBridge()
    image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image,camera_callback)
    pub = rospy.Publisher('vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10)
    moveTurtlebot3_object = MoveTurtlebot3()

    vel = AckermannDriveStamped()
    velocity = 0.3
    vel.drive.speed = velocity
    # angle = m

    # print(np.float32(m)*0.01)



    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # angle = m+
        vel.drive.steering_angle = -m
        print(m)
        # print(m, vel.drive.steering_angle)
        # print(vel.drive.steering_angle)
        pub.publish(vel)
        # print(vel.drive.steering_angle)
        print('===========')
        rate.sleep()
    vel.drive.speed = 0.0
    vel.drive.steering_angle = 0.00
    pub.publish(vel)
