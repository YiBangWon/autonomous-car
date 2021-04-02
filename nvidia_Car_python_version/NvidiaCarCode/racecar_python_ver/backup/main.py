#!/usr/bin/env python
import cv2
import threading
import Queue as que
import time
import numpy as np

import roslib
import sys
import rospy

import importlib
import cPickle
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from rosservice import ROSServiceException

from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge, CvBridgeError

from ackermann_msgs.msg import AckermannDriveStamped

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

q1 = que.Queue()
bridge = CvBridge()

cv_image = None
ack_publisher = None
car_run_speed = 0.5

def img_callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def auto_drive(pid, slope_cnt):
    global car_run_speed
    w = 0
    if slope_cnt > 6:
        car_run_speed = 0
    else:
        if -0.065 < pid and pid < 0.065:
            w = 1.2
            #w=1
        else:
            # w = 0.3
            w = 0.5

        if car_run_speed < 1.0 * w:
            car_run_speed += 0.002 * 10
        else:
            car_run_speed -= 0.003 * 10

    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = ''
    ack_msg.drive.steering_angle = pid
    ack_msg.drive.speed = car_run_speed
    ack_publisher.publish(ack_msg)
    print 'speed:: '
    print car_run_speed

def msgCallback(data):
    # print(len(data.ranges))

    find1 = 0
    find2 = 0
    find3 = 0
    find4 = 0

    findAng = []

    for i in range(45,135):
        if data.ranges[i] > 0.5 and data.ranges[i] < 1:
            findAng.append(i)
    rospy.loginfo(findAng)

    # for i in range(0,90):
    #     if data.ranges[i] > 0.5 and data.ranges[i] < 1:
    #         find1 = find1 + 1
    #     if data.ranges[i] > 1 and data.ranges[i] < 2:
    #         find3 = find3 + 1
    # for i in range(90,180):
    #     if data.ranges[i] > 0.5 and data.ranges[i] < 1:
    #         find2 = find2 + 1
    #     if data.ranges[i] > 1 and data.ranges[i] < 2:
    #         find4 = find4 + 1

    # rospy.loginfo("left: %d, left_Far:%d  right: %d,  right_Far: %d", find1, find3, find2, find4)

    # for i in range(data.ranges):
    #     print(data.ranges[i])

def main():
    global cv_image
    global ack_publisher
    rospy.sleep(3)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    # image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",Image,img_callback)

    # rospy.init_node('auto_xycar', anonymous=True)
    rospy.init_node('auto_xycar', log_level=rospy.DEBUG)

    #ack_publisher = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    # record the origin
    #out = cv2.VideoWriter('/home/nvidia/Desktop/outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    lidar_sub = rospy.Subscriber("/scan", LaserScan, msgCallback)

    # record the processed
    #out2 = cv2.VideoWriter('/home/nvidia/Desktop/oripy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    while cv_image != None:
      # edges_img_for_stop, img1, x_location, slope_cnt = process_image(cv_image)
      img1, x_location = process_image(cv_image)
      # cv2.imshow('result', img1)
      # cv2.imshow("stopLine", edges_img_for_stop)
      if x_location != None:
          pid = round(pidcal.pid_control(int(x_location)), 6)
          # print pid
          # rospy.loginfo(pid)
          auto_drive(pid, slope_cnt)
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      # cv2.imshow("origin", cv_image)
      #out.write(img1)
      #out2.write(cv_image)


    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")

    # rospy.spin()
    # cv2.destroyAllWindows()

def process_image(frame):
    # cv2.imshow("defalut", frame)
    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # cv2.imshow("g", blur_gray)
    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # cv2.imshow("edges_img", edges_img)
    # warper
    img = warper.warp(edges_img)
    # cv2.imshow("ww", img)
    img1, x_location = slidewindow.slidewindow(img)
    # cv2.imshow("e1", img1)
    return img1, x_location

# def process_image(frame):
#
#     # grayscle
#     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     # blur
#     kernel_size = 5
#     blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
#     # canny edge
#     low_threshold = 60#60
#     high_threshold = 70# 70
#     edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
#
#     # edges_img_for_stop = edge_img[width/2 - 50 : width/2 + 50, height/2 + 100: height/2 + 150]
#
#     lines = cv2.HoughLinesP(edges_img_for_stop, 1, np.pi/180, 100, 100, 10)
#
#     for i in xrange(len(lines)):
#         for x1,y1,x2,y2 in lines[i]:
#             cv2.line(img,(x1,y1),(x2,y2),(0,0,255),3)
#         #slope = (atan2(y1 - y2, x1 - x2)*180 / np.pi)
#         #print("slope:: ")
#         #print(slope)
#         #if(slope > 170 && slope < 190 )
#         #    stop_slope_cnt++
#
#     #warper
#     img = warper.warp(edges_img)
#     img1, x_location = slidewindow.slidewindow(img)
#
#     return edges_img_for_stop, img1, x_location, stop_slope_cnt

if __name__ == '__main__':
    main()
