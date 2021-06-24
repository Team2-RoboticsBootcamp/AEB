#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import rospy


def plot_x(msg):
    global counter
    if counter % 10 == 0:
        plt.plot(counter, msg.speed, '.')
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("vesc/ackermann_cmd_mux/input/teleop", ackermann_msgs/AckermannDriveStamped, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()

