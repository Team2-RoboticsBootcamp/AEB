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
global previously_known_polynomials
global lane_center
lane_center = 0
global velocity
velocity = 0

def camera_callback(data):
    global m
    global lane_center
    global velocity


    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

    #print('original image shape',cv_image.shape)
    imgFinal = cv_image.copy()

    grayscale = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)

    imgThres,imgCanny,imgColor = thresholding (cv_image)

    #wrap
    src = np.float32([[170,330],[500,330],[50,450],[630,450]])
    width, height =480,640 # 380,700
    drc = np.float32([[0,0],[width,0],[0,height],[width,height]])
    matrix = cv2.getPerspectiveTransform(src,drc)
    wrap_output = cv2.warpPerspective(imgThres,matrix,(width,height))

    #oldWrap
    # frameWidth = 640
    # frameHeight = 480
    # src = [24, 55, 12, 100]
    # imgWarp = perspective_warp(imgThres, dst_size=(frameWidth, frameHeight), src=src)


    imgSliding, curves, lanes, ploty, lane_points = sliding_window(wrap_output,draw_windows=True)
    lane_center = (lane_points[0] + lane_points[1])/2 - 640/2 + 88

    #center =0
    try:
        curverad = get_curve(imgFinal, curves[0], curves[1])
        lane_curve = np.mean([curverad[0], curverad[1]])
        imgFinal = draw_lanes(cv_image, curves[0], curves[1], frameWidth, frameHeight, src=src)

        print('check')
        # ## Average
        currentCurve = lane_curve // 50
        if  int(np.sum(arrayCurve)) == 0:averageCurve = currentCurve
        else:
            averageCurve = np.sum(arrayCurve) // arrayCurve.shape[0]
        if abs(averageCurve-currentCurve) >200: arrayCurve[arrayCounter] = averageCurve
        else :arrayCurve[arrayCounter] = currentCurve
        arrayCounter +=1
        if arrayCounter >=noOfArrayValues : arrayCounter=0
        cv2.putText(imgFinal, str(int(averageCurve)), (frameWidth//2-70, 70), cv2.FONT_HERSHEY_DUPLEX, 1.75, (0, 0, 255), 2, cv2.LINE_AA)

    except:
       lane_curve=00
       pass

    imgFinal = drawLines(imgFinal,lane_curve)

    imgThres = cv2.cvtColor(imgThres,cv2.COLOR_GRAY2BGR)
    imgBlank = np.zeros_like(cv_image)
    # imgStacked = stackImages(0.7, ([imgColor, imgCanny, imgThres],
                                    # [wrap_output,imgSliding,imgFinal]
                                    # ))


    # cv2.imshow("PipeLine",imgStacked)
    # cv2.imshow("Result", imgFinal)
    text = "Velocity = " + str(velocity)
    text2 = "Steering angle = " + str(m)
    cv2.imshow('input',imgSliding)
    cv2.putText(imgFinal, text, (40,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.putText(imgFinal, text2, (40,80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow('canny',imgFinal)
   # cv2.moveWindow('canny',700,0)

    cv2.imshow('color',imgColor)
    #cv2.moveWindow('color',1400,0)

    cv2.imshow('combined',imgThres)
    #cv2.moveWindow('combined',0,500)

    cv2.imshow('wrap',wrap_output)
    #cv2.moveWindow('wrap',700,500)
    cv2.waitKey(1)

    # if lane_center > 0:
    #     m = 0.35
    # elif lane_center == 0:
    #     m = 0
    # else:
    #     m = - 0.35

    m = np.clip(lane_center/160,-0.5,0.5)



    # cv2.imshow('frame',cropped_image)

def canny(image):
    #gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(image, 0, 150)
    return canny




def colorFilter(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lowerYellow = np.array([18,94,140])
    upperYellow = np.array([48,255,255])
    lowerWhite = np.array([0, 0, 200])
    upperWhite = np.array([255, 255, 255])
    maskedWhite= cv2.inRange(hsv,lowerWhite,upperWhite)
    maskedYellow = cv2.inRange(hsv, lowerYellow, upperYellow)
    combinedImage = cv2.bitwise_or(maskedWhite,maskedYellow)
    return combinedImage

def thresholding(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((5,5))
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 0)
    imgCanny = cv2.Canny(imgBlur, 50, 100)
    #imgClose = cv2.morphologyEx(imgCanny, cv2.MORPH_CLOSE, np.ones((10,10)))
    imgDial = cv2.dilate(imgCanny,kernel,iterations=1)
    imgErode = cv2.erode(imgDial,kernel,iterations=1)

    imgColor = colorFilter(img)
    combinedImage = cv2.bitwise_or(imgColor, imgErode)

    return combinedImage,imgCanny,imgColor





def get_hist(img):
    hist = np.sum(img[img.shape[0]//2:,:], axis=0)
    return hist


left_a, left_b, left_c = [], [], []
right_a, right_b, right_c = [], [], []


def sliding_window(img, nwindows=15, margin=50, minpix=1, draw_windows=True):
    global left_a, left_b, left_c, right_a, right_b, right_c
    left_fit_ = np.empty(3)
    right_fit_ = np.empty(3)
    out_img = np.dstack((img, img, img)) * 255

    histogram = get_hist(img)
    # find peaks of left and right halves
    midpoint = int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Set height of windows
    window_height = np.int(img.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window + 1) * window_height
        win_y_high = img.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        if draw_windows == True:
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                          (100, 255, 255), 1)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                          (100, 255, 255), 1)
            # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    #        if len(good_right_inds) > minpix:
    #            rightx_current = np.int(np.mean([leftx_current +900, np.mean(nonzerox[good_right_inds])]))
    #        elif len(good_left_inds) > minpix:
    #            rightx_current = np.int(np.mean([np.mean(nonzerox[good_left_inds]) +900, rightx_current]))
    #        if len(good_left_inds) > minpix:
    #            leftx_current = np.int(np.mean([rightx_current -900, np.mean(nonzerox[good_left_inds])]))
    #        elif len(good_right_inds) > minpix:
    #            leftx_current = np.int(np.mean([np.mean(nonzerox[good_right_inds]) -900, leftx_current]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    if leftx.size and rightx.size:
        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        left_a.append(left_fit[0])
        left_b.append(left_fit[1])
        left_c.append(left_fit[2])

        right_a.append(right_fit[0])
        right_b.append(right_fit[1])
        right_c.append(right_fit[2])

        left_fit_[0] = np.mean(left_a[-10:])
        left_fit_[1] = np.mean(left_b[-10:])
        left_fit_[2] = np.mean(left_c[-10:])

        right_fit_[0] = np.mean(right_a[-10:])
        right_fit_[1] = np.mean(right_b[-10:])
        right_fit_[2] = np.mean(right_c[-10:])

        # Generate x and y values for plotting
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])

        left_fitx = left_fit_[0] * ploty ** 2 + left_fit_[1] * ploty + left_fit_[2]
        right_fitx = right_fit_[0] * ploty ** 2 + right_fit_[1] * ploty + right_fit_[2]

        left_x = left_fit_[0] *img.shape[0]** 2 + left_fit_[1] *img.shape[0]+ left_fit_[2]
        right_x = right_fit_[0] *img.shape[0] ** 2 + right_fit_[1] *img.shape[0] + right_fit_[2]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 100]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 100, 255]

        return out_img, (left_fitx, right_fitx), (left_fit_, right_fit_), ploty,(left_x,right_x)
    else:
        return img,(0,0),(0,0),0,(img.shape[0]/2,img.shape[0]/2)


def get_curve(img, leftx, rightx):
    ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
    y_eval = np.max(ploty)
    ym_per_pix = 1 / img.shape[0]  # meters per pixel in y dimension
    xm_per_pix = 0.1 / img.shape[0]  # meters per pixel in x dimension
    # print(ploty.shape)
    # print(leftx.shape)
    # print(rightx.shape)
    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * right_fit_cr[0])

    car_pos = img.shape[1] / 2
    l_fit_x_int = left_fit_cr[0] * img.shape[0] ** 2 + left_fit_cr[1] * img.shape[0] + left_fit_cr[2]
    r_fit_x_int = right_fit_cr[0] * img.shape[0] ** 2 + right_fit_cr[1] * img.shape[0] + right_fit_cr[2]
    lane_center_position = (r_fit_x_int + l_fit_x_int) / 2
    center = (car_pos - lane_center_position) * xm_per_pix / 10
    # Now our radius of curvature is in meters
    #rospy.loginfo(center)
    return (l_fit_x_int, r_fit_x_int, center,lane_center_position)


def draw_lanes(img, left_fit, right_fit,frameWidth,frameHeight,src):
    ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
    color_img = np.zeros_like(img)

    left = np.array([np.transpose(np.vstack([left_fit, ploty]))])
    right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))])
    points = np.hstack((left, right))

    # cv2.fillPoly(color_img, np.int_(points), (0, 200, 255))
    # inv_perspective = inv_perspective_warp(color_img,(frameWidth,frameHeight),dst=src)
    # inv_perspective = cv2.addWeighted(img, 0.5, inv_perspective, 0.7, 0)

    drcx = np.float32([[170,330],[500,330],[50,450],[630,450]])
    width, height = 480,640 # 700,380
    srcx = np.float32([[0,0],[width,0],[0,height],[width,height]])
    matrix = cv2.getPerspectiveTransform(srcx,drcx)
    inv_perspective = cv2.warpPerspective(color_img,matrix,(width,height))
    inv_perspective = cv2.addWeighted(img, 0.5, inv_perspective, 0.7, 0)

    return inv_perspective


def drawLines(img,lane_curve):
    myWidth = img.shape[1]
    myHeight = img.shape[0]
    #print(myWidth,myHeight)
    for x in range(-30, 30):
        w = myWidth // 20
        cv2.line(img, (w * x + int(lane_curve // 100), myHeight - 30),
                 (w * x + int(lane_curve // 100), myHeight), (0, 0, 255), 2)
    cv2.line(img, (int(lane_curve // 100) + myWidth // 2, myHeight - 30),
             (int(lane_curve // 100) + myWidth // 2, myHeight), (0, 255, 0), 3)
    cv2.line(img, (myWidth // 2, myHeight - 50), (myWidth // 2, myHeight), (0, 255, 255), 2)

    return img



def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def textDisplay(curve,img):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, str(curve), ((img.shape[1]//2)-30, 40), font, 1, (255, 255, 0), 2, cv2.LINE_AA)
    directionText=' No lane '
    if curve > 10:
        directionText='Right'
    elif curve < -10:
        directionText='Left'
    elif curve <10 and curve > -10:
        directionText='Straight'
    elif curve == -1000000:
        directionText = 'No Lane Found'
    cv2.putText(img, directionText, ((img.shape[1]//2)-35,(img.shape[0])-20 ), font, 1, (0, 200, 200), 2, cv2.LINE_AA)


if __name__ == '__main__':
    rospy.init_node('lane_follower')
    bridge_object = CvBridge()
    image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image,camera_callback)
    pub = rospy.Publisher('vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10)
    moveTurtlebot3_object = MoveTurtlebot3()

    vel = AckermannDriveStamped()

    # angle = m

    # print(np.float32(m)*0.01)

#    global lane_center
#    global velocity

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # angle = m+
        velocity =  np.clip((0.6 - abs(m)*1.8),0.1,0.3)
        vel.drive.speed = velocity
        vel.drive.steering_angle = -m
        # print(m)
        # print(m, vel.drive.steering_angle)
        # print(vel.drive.steering_angle)
        pub.publish(vel)
        # print(vel.drive.steering_angle)
        print('lane error', lane_center)
        print('Steering angle', m)
        print('Velocity', vel.drive.speed)
        print('\n \n \n \n \n \n')
        rate.sleep()

    vel.drive.speed = 0.0
    vel.drive.steering_angle = 0.00
    pub.publish(vel)
    sleep(5)
