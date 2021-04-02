#!/usr/bin/env python
import cv2
import threading
import Queue as que
import time
import numpy as np

import roslib
import sys
import rospy
import signal
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
edges_img_for_stop = None


white_count = 0;
black_count = 0;
white = 0;
black = 0;
previous_color = 1;
first_detected = 0;
it_is_not_pattern = 0;


def filter_colors_white(img_ori):
    lower_white_color = np.array([200, 200, 200])
    upper_white_color = np.array([255, 255, 255])
    white_image = cv2.inRange(img_ori, lower_white_color, upper_white_color)
    return white_image


def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)

def img_callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def main():
    global cv_image
    global ack_publisher
    global edges_img_for_stop
    rospy.sleep(3)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    rospy.init_node('auto_xycar', anonymous=True)
    #signal.signal(signal.SIGINT, signal_handler)

    cap = cv2.VideoCapture("/home/nvidia/Desktop/test1.avi")

    # while True:
    #     ret, fram = cap.read()
    #
    #     if ret:
    #         cv2.imshow('video', fram)
    #         edges_img_for_stop, img1, x_location = process_image(cv_image)
    #         cv2.imshow("result", img1)
    #
    #
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #               break
    # cap.release()

    while cv_image != None:
      edges_img_for_stop, img1, x_location = process_image(cv_image)
      cv2.imshow('origin21600', cv_image)
      # cv2.imshow('stopLine', edges_img_for_stop)
      if x_location != None:

          image_roi=cv_image[480*0.75 : 480*0.95, 640 * 0.1: 640*0.95]
          image_red = filter_colors_white(image_roi)

          white_count = 0;
          black_count = 0;
          white = 0;
          black = 0;
          previous_color = 1;
          first_detected = 0;
          it_is_not_pattern = 0;


          if red_line_ignore_time < 0 :
              for i in range(1, 6):
                for j in range(1, 7):
                    if image_white.item(96*0.8, 544*0.12*i + (-3+j), 0) >= 190 & image_white.item(96*0.8, 544*0.12*i + (-3+j), 0) <= 255 :
                        if image_white.item(96*0.8, 544*0.12*i + (-3+j), 1) >= 190 & image_white.item(96*0.8, 544*0.12*i + (-3+j), 1) <= 255 :
                             if image_white.item(96*0.8, 544*0.12*i + (-3+j), 2) >= 190 & image_white.item(96*0.8, 544*0.12*i + (-3+j), 2) <= 255 :
                                 white_count = white_count + 1

                    elif image_white.item(96*0.8, 544*0.12*i + (-3+j), 0) == 0:
                        if image_white.item(96*0.8, 544*0.12*i + (-3+j), 1) == 0:
                            if image_white.item(96*0.8, 544*0.12*i + (-3+j), 2) == 0:
                                black_count = black_count + 1

              if white_count >= 4 & first_detected == 0:
                first_detected = 1
                white = white + 1
                previous_color = 1
              elif black_count >= 4 & first_detected == 0:
                first_detected = 1
                black  = black + 1
                previous_color = 0
              elif first_detected != 0 & previous_color %2 == 0 & white_count >= 4:
                white = white + 1
                previous_color = previous_color + 1
              elif first_detected != 0 & previous_color %2 == 1 & white_count >= 4:
                black = black + 1
                previous_color = previous_color + 1
              elif black_count >= 4 or white_count >= 4:
                it_is_not_pattern = 1

              black_count = 0
              white_count = 0


              if it_is_not_pattern == 1 || black < 2 || white < 2 || abs(white - black) > 1 :
                  for i in range(1, 6):
                    for j in range(1, 7):
                        if image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 0) >= 190 & image_white.item(96*0.55, 544*0.14+(544*(i-1)*0.11) + (-3+j), 0) <= 255 :
                            if image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 1) >= 190 & image_white.item(96*0.55, 544.14+(544*(i-1)*0.11) + (-3+j), 1) <= 255 :
                                 if image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 2) >= 190 & image_white.item(96*0.55, 544*0.14+(544*(i-1)*0.11) + (-3+j), 2) <= 255 :
                                     white_count = white_count + 1

                        elif image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 0) == 0:
                            if image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 1) == 0:
                                if image_white.item(96*0.55, 544*0.14 + (544*(i-1)*0.11) + (-3+j), 2) == 0:
                                    black_count = black_count + 1

              if white_count >= 4 & first_detected == 0:
                first_detected = 1
                white = white + 1
                previous_color = 1
              elif black_count >= 4 & first_detected == 0:
                first_detected = 1
                black  = black + 1
                previous_color = 0
              elif first_detected != 0 & previous_color %2 == 0 & white_count >= 4:
                white = white + 1
                previous_color = previous_color + 1
              elif first_detected != 0 & previous_color %2 == 1 & white_count >= 4:
                black = black + 1
                previous_color = previous_color + 1
              elif black_count >= 4 || white_count >= 4:
                it_is_not_pattern = 1

              black_count = 0
              white_count = 0


              if it_is_not_pattern != 1 & black >= 2 & white >= 2 & abs(white - black) <= 1 :
              red_line_count = red_line_count + 1;
              red_line_ignore_time = 500;

              it_is_not_pattern = 0;

         # pid = round(pidcal.pid_control(int(x_location)), 6)
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      cv2.imshow("result", img1)

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")


    cv2.destroyAllWindows()

def process_image(frame):
    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 20
    high_threshold = 70
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # edges_img_for_stop = edges_img[330 : 480, 60: 600]

    # lines = cv2.HoughLinesP(edges_img_for_stop, 1, np.pi/180, 100, 100, 10)

    #for line in lines:
    #    x1, y1, x2, y2 in line[0]:
    #    cv2.line(edges_img_for_stop, (x1, y1), (x2, y2), (0,255,0), 2)
    # warper
    img = warper.warp(edges_img)
    img1, x_location = slidewindow.slidewindow(img)
#
    # return edges_img_for_stop, img1, x_location
    return img1,x_location

if __name__ == '__main__':
    main()
