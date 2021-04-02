#!/usr/bin/env python
# import torch
# import cv2
# import threading
# import Queue as que
# import time
# import numpy as np

# import roslib
# import sys
# import rospy
#
# import importlib
# import cPickle
# import genpy.message
# from rospy import ROSException
# import sensor_msgs.msg
# import actionlib
# import rostopic
# import rosservice
# from rosservice import ROSServiceException
#
# from slidewindow import SlideWindow
# from warper import Warper
# from pidcal import PidCal
#
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import LaserScan
#
# from cv_bridge import CvBridge, CvBridgeError
#
# from ackermann_msgs.msg import AckermannDriveStamped
#
# warper = Warper()
# slidewindow  = SlideWindow()
# pidcal = PidCal()
#
# q1 = que.Queue()
# bridge = CvBridge()
#
# cv_image = None
# ack_publisher = None
# # car_run_speed = 1.0
# car_run_speed = 0
# # car_run_speed = 0.5
#
# isFindFarObject = False
# isFindNearObject = False
# farObjcetAngle = 90
# nearObjcetAngle = 90
#
# rightObjectDetect_WhenTurnLeft = False
#
# fast_drive_cnt = 200
#
# previous_pid = 0
# def img_callback(data):
#     global cv_image
#     try:
#       cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#     except CvBridgeError as e:
#       print(e)
#
# def auto_drive(pid):
#     global car_run_speed
#     global fast_drive_cnt
#     w = 0
#
#
#     # if -0.065 < pid and pid < 0.065:
#     if -0.075 < pid and pid < 0.075:
#         # w = 1.0
#         w = 0
#         # w = 3.0
#         fast_drive_cnt = fast_drive_cnt - 1
#     else:
#         w = 0
#         # w = 0.0
#         fast_drive_cnt = 200
#
#     if car_run_speed < 1.0 * w:
#         # car_run_speed += 0.0005 * 10
#         car_run_speed += 0.0000 * 10
#     else:
#         # car_run_speed -= 0.003 * 10
#         # car_run_speed -= 0.0033 * 10
#         car_run_speed -= 0.000 * 10
#
#     if fast_drive_cnt == 30: # slow down when short line
#         car_run_speed -= 0.0
#
#     if fast_drive_cnt == 80: # slow down when long line
#         car_run_speed -= 0.0
#
#     if fast_drive_cnt < 0: # more slow down when long line
#         fast_drive_cnt = 200
#         car_run_speed = 0.0
#
#     ack_msg = AckermannDriveStamped()
#     ack_msg.header.stamp = rospy.Time.now()
#     ack_msg.header.frame_id = ''
#     ack_msg.drive.steering_angle = pid
#     ack_msg.drive.speed = car_run_speed
#     # ack_msg.drive.speed = 0
#     ack_publisher.publish(ack_msg)
#     print 'speed:: '
#     print car_run_speed
#     rospy.loginfo("pid: %f", pid)
#
# def goSlow(pid):
#     global car_run_speed
#     w = 0
#
#
#     ack_msg = AckermannDriveStamped()
#     ack_msg.header.stamp = rospy.Time.now()
#     ack_msg.header.frame_id = ''
#     ack_msg.drive.steering_angle = pid
#     # ack_msg.drive.speed = 0.5
#     ack_msg.drive.speed = car_run_speed
#     # ack_msg.drive.speed = 0
#     ack_publisher.publish(ack_msg)
#
# def msgCallback(data):
#     # print(len(data.ranges))
#     global isFindFarObject
#     global isFindNearObject
#     global farObjcetAngle
#     global nearObjcetAngle
#     global rightObjectDetect_WhenTurnLeft
#     global leftObjectDetect_WhenTurnRight
#
#     global car_run_speed
#     find1 = 0
#     find2 = 0
#     find3 = 0
#     find4 = 0
#
#     findFarAng = []
#     findNearAng = []
#
#     findFarFarAng = []
#
#     # for i in range(45,135):
#     #     if data.ranges[i] < 0.5:
#     #     # if data.ranges[i] > 0.5 and data.ranges[i] < 1:
#     #         findAng.append(i)
#     # rospy.loginfo(findAng)
#
#     # for i in range(68, 113):
#     # for i in range(52, 128):
#     #     if data.ranges[i] > 0.4 and data.ranges[i] < 0.65:
#
#     # good but not use
#     # for i in range(71, 109):
#     #     if data.ranges[i] > 0.8 and data.ranges[i] < 1.2:
#     #         findFarAng.append(i)
#     # for i in range(62, 118):
#     #     if data.ranges[i] > 0.6 and data.ranges[i] <= 0.8:
#     #         findFarAng.append(i)
#     # for i in range(52, 128):
#     #     if data.ranges[i] > 0.4 and data.ranges[i] <= 0.6:
#     #         findFarAng.append(i)
#     for i in range(70, 110):
#     # for i in range(75, 105):
#         # if data.ranges[i] > 1.2 and data.ranges[i] < 1.6:
#         if data.ranges[i] > 0.8 and data.ranges[i] < 1.2:
#             findFarAng.append(i)
#     # rospy.loginfo(findFarAng)
#     # for i in range(64, 117):
#     #     if data.ranges[i] > 0.6 and data.ranges[i] < 0.105:
#     #         findFarAng.append(i)
#
#     # for i in range(45, 135):
#     for i in range(20, 170):
#         if data.ranges[i] >= 0.15 and data.ranges[i] < 0.25:
#             findNearAng.append(i)
#
#     if len(findFarAng) > 5:
#         del findFarAng[:]
#
#     if len(findFarAng) > 1:
#         isFindFarObject = True
#         farObjcetAngle = sum(findFarAng) / len(findFarAng)
#
#         # car_run_speed = 0.7
#     else:
#         isFindFarObject = False
#
#     if len(findNearAng) > 1:
#         isFindNearObject = True
#         NearObjcetAngle = sum(findNearAng) / len(findNearAng)
#     else:
#         isFindNearObject = False
#
#     # to detect far object
#     # for i in range(70, 110):
#     #     if data.ranges[i] > 1.2 and data.ranges[i] < 1.6:
#     #         findFarFarAng.append(i)
#     # rospy.loginfo(findFarFarAng)
#
#     # to detect object when line chaging
#     # for i in range(110, 135):
#     #     if data.ranges[i] > 0.7 and data.ranges[i] < 1.1:
#     #         findFarFarAng.append(i)
#     # rospy.loginfo(findFarFarAng)
#
#     for i in range(118, 140):
#         if data.ranges[i] > 0.7 and data.ranges[i] < 1.1:
#     # if data.ranges[123] > 0.7 and data.ranges[123] < 1.1:
#             rightObjectDetect_WhenTurnLeft = True
#         else:
#             rightObjectDetect_WhenTurnLeft = False
#
#     for i in range(40, 62):
#         if data.ranges[i] > 0.7 and data.ranges[i] < 1.1:
#     # if data.ranges[123] > 0.7 and data.ranges[123] < 1.1:
#             leftObjectDetect_WhenTurnRight = True
#         else:
#             leftObjectDetect_WhenTurnRight = False
#     # rospy.loginfo(data.ranges[123])
#     # rospy.loginfo(findNearAng)
#
#     testStop = []
#     for i in range(86,94):
#         if data.ranges[i] > 2 and data.ranges[i] < 3:
#             testStop.append(i)
#
#
#     if len(testStop) > 4:
#         car_run_speed = 0.6
#
#     # rospy.loginfo(testStop)
#
#     # for i in range(0,90):
#     #     if data.ranges[i] > 0.5 and data.ranges[i] < 1:
#     #         find1 = find1 + 1
#     #     if data.ranges[i] > 1 and data.ranges[i] < 2:
#     #         find3 = find3 + 1
#     # for i in range(90,180):
#     #     if data.ranges[i] > 0.5 and data.ranges[i] < 1:
#     #         find2 = find2 + 1
#     #     if data.ranges[i] > 1 and data.ranges[i] < 2:
#     #         find4 = find4 + 1
#
#     # rospy.loginfo("left: %d, left_Far:%d  right: %d,  right_Far: %d", find1, find3, find2, find4)
#
#     # for i in range(data.ranges):
#     #     print(data.ranges[i])
#
# def filter_colors_yellow(img_ori):
#     # lower_red_color = np.array([0, 0, 110])
#     lower_yellow_color = np.array([100, 100, 0])
#     upper_yellow_color = np.array([200, 200, 100])
#     red_image = cv2.inRange(img_ori, lower_yellow_color, upper_yellow_color)
#     return red_image
#
# def filter_colors_white(img_ori):
#     lower_white_color = np.array([200, 200, 200])
#     upper_white_color = np.array([255, 255, 255])
#     white_image = cv2.inRange(img_ori, lower_white_color, upper_white_color)
#     return white_image
#
# def force_Stop():
#     global car_run_speed
#     car_run_speed = 0
#
# def main():
#     global cv_image
#     global ack_publisher
#
#     global farObjcetAngle
#     global nearObjcetAngle
#     global isFindFarObject
#     global isFindNearObject
#
#     global rightObjectDetect_WhenTurnLeft
#     global leftObjectDetect_WhenTurnRight
#     global car_run_speed
#
#     global previous_pid
#
#     # global fast_drive_cnt
#
#     rospy.sleep(3)
#     bridge = CvBridge()
#     image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
#     # image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed",Image,img_callback)
#
#     # rospy.init_node('auto_xycar', anonymous=True)
#     rospy.init_node('auto_xycar', log_level=rospy.DEBUG)
#
#     #ack_publisher = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
#     ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
#     # record the origin
#     out = cv2.VideoWriter('/home/nvidia/Desktop/test1.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))
#
#     lidar_sub = rospy.Subscriber("/scan", LaserScan, msgCallback)
#
#     # record the processed
#     #out2 = cv2.VideoWriter('/home/nvidia/Desktop/oripy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))
#     slope_cnt = 0
#     # previous_pid = 0
#     camera_check = False
#     frameCnt = 0
#     cameraCheck = False
#     # cameraCheck = True
#
#     red_line_ignore_time = 0
#     red_line_count = 0
#     stop_count_frame = 0
#
#     line_change_frame = 0
#     changing_line = 0
#
#     tempCnt = 0
#     accumulateObjectCnt = 0
#     accumulateObjectLocation = 0
#
#     reverse_move_Start = 0
#     reverse_move_End = 0
#     reverse_move_diff = 0
#
#     ignore_lidar_frame_cnt = 20
#
#     object_turning_angle = 0.14
#
#     while cv_image != None:
#       # edges_img_for_stop, img1, x_location, slope_cnt = process_image(cv_image)
#       img1, x_location = process_image(cv_image)
#       # cv2.imshow('result', img1)
#
#       frameCnt = frameCnt + 1
#
#       if frameCnt == 10:
#           myStart = input()
#           if myStart == 1:
#               # cv2.destroyAllWindows()
#               cameraCheck = True
#           elif myStart == 0:
#               break;
#       if cameraCheck == False:
#           cv2.imshow('result1111', img1)
#
#       # cv2.imshow('result1111', img1)
#
#       # if frameCnt < 25:
#       #     cv2.imshow('result1111', img1)
#       #     # rospy.loginfo("--> %d", frameCnt)
#       # elif frameCnt == 25:
#       #     myStart = input()
#       #     cv2.destroyAllWindows()
#       #     cameraCheck = True
#
#       red_line_ignore_time = red_line_ignore_time - 1
#
#
#       image_roi=cv_image[360:450, 110:560]
#       cv2.convertScaleAbs(image_roi, image_roi, 1, 30)
#
#
#       image_red = filter_colors_yellow(image_roi)
#       nonzero = cv2.countNonZero(image_red)
#       # rospy.loginfo("num: %d", nonzero)
#
#       if red_line_ignore_time < 0:
#           if nonzero > 100:
#             image_white = filter_colors_white(image_roi)
#             nonzero_white = cv2.countNonZero(image_red)
#             if(nonzero_white > 1500):
#                 red_line_count = red_line_count + 1
#                 red_line_ignore_time = 500
#                 cv2.imshow('hihi', image_white)
#                 #rospy.loginfo("detect line %d", red_line_count)
#       cv2.putText(img1, str(red_line_count), (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255))
#       # cv2.imshow('result', img1)
#       # for y in range(np.size(image_red, 1)):
#       #     for x in range(np.size(image_red, 0)):
#       #         cnt = cnt + 1
#
#       # cv2.imshow('22', image_roi)
#       # cv2.imshow('33', image_red)
#       # cv2.imshow("stopLine", edges_img_for_stop)
#       #
#       # if red_line_count > 3:
#       #     stop_count_frame = stop_count_frame + 1
#       #     if stop_count_frame > 10:
#       #         break
#
#       if changing_line == 1: # turn left
#           # if rightObjectDetect_WhenTurnLeft:
#           #     # force_Stop()
#           #     # changing_line = 3
#           #
#           #     reverse_move_End = frameCnt
#           #     rospy.loginfo("reverse@@ to right : %d", reverse_move_End - reverse_move_Start)
#           #     reverse_move_diff = reverse_move_End - reverse_move_Start
#           # else:
#           # pid = 0.14
#           pid = object_turning_angle/2
#           reverse_move_End = frameCnt
#           reverse_move_diff = reverse_move_End - reverse_move_Start
#           if reverse_move_diff > 12:
#               rospy.loginfo("~~~~~~overtime to right : %d", reverse_move_End - reverse_move_Start)
#               changing_line = 3
#       elif changing_line == 2:  # turn right
#           # if leftObjectDetect_WhenTurnRight:
#           #     # force_Stop()
#           #     # changing_line = 4
#           #
#           #     reverse_move_End = frameCnt
#           #     rospy.loginfo("reverse $$$ to left : %d", reverse_move_End - reverse_move_Start)
#           #     reverse_move_diff = reverse_move_End - reverse_move_Start
#           # else:
#           # pid = -0.07
#           pid = object_turning_angle/2 * (-1)
#           reverse_move_End = frameCnt
#           reverse_move_diff = reverse_move_End - reverse_move_Start
#           if reverse_move_diff > 12:
#               rospy.loginfo("~~~~~~overtime to left : %d", reverse_move_End - reverse_move_Start)
#               changing_line = 4
#
#       elif changing_line == 3:
#           if reverse_move_diff > 0:
#               # pid = -0.14
#               pid = object_turning_angle*-1
#               reverse_move_diff = reverse_move_diff - 1
#           else:
#               pid = 0
#               changing_line = 0
#       elif changing_line == 4:
#           if reverse_move_diff > 0:
#               # pid = 0.14
#               pid = object_turning_angle
#               # reverse_move_diff = reverse_move_diff - 1
#               reverse_move_diff = reverse_move_diff - 1
#           else:
#               pid = 0
#               changing_line = 0
#
#       angleRatio = 7
#       if cameraCheck:
#           if x_location != None:
#
#               checkPid = round(pidcal.pid_control(int(x_location)), 6)
#               # previous_pid = pid
#               # rospy.loginfo("x_lodation : %f  ****", x_location)
#
#               # ignore object when curved line
#               if changing_line == 0 and (-0.1 > checkPid or checkPid > 0.1):
#                   ignore_lidar_frame_cnt = 20
#                   # rospy.loginfo("ignore!!!! %f", checkPid)
#               else:
#                   ignore_lidar_frame_cnt = ignore_lidar_frame_cnt - 1
#
#               if ignore_lidar_frame_cnt < 0 and isFindFarObject and changing_line == 0:
#                   # force_Stop()
#                   # car_run_speed = 0.5
#                   if farObjcetAngle > 90:
#                       ## x_location = x_location - (113 - farObjcetAngle + 10)*angleRatio
#                       # x_location = x_location - pow((117 - farObjcetAngle), 2)/angleRatio
#                       #rospy.loginfo("move to left! %d", (113 - farObjcetAngle + 10)*angleRatio)
#                       # rospy.loginfo("move to left! %d  ---  %d", x_location, pow((113 - farObjcetAngle), 2)/angleRatio)
#                       changing_line = 1
#                       # pid = 0.14
#                       pid = object_turning_angle/2
#
#                       reverse_move_Start = frameCnt
#                       rospy.loginfo("turn left")
#                   else:
#                       ## x_location = x_location + (nearObjcetAngle - 68 +  10)*angleRatio
#                       # x_location = x_location + pow((farObjcetAngle - 64), 2)/angleRatio
#                       #rospy.loginfo("move to right! %d", (nearObjcetAngle - 68 + 10)*angleRatio)
#                       # rospy.loginfo("move to right! %d ---  %d", x_location, pow((nearObjcetAngle - 68), 2)/angleRatio)
#                       changing_line = 2
#                       # pid = -0.07
#                       pid = object_turning_angle/2 * (-1)
#                       reverse_move_Start = frameCnt
#                       rospy.loginfo("turn right")
#
#               # if isFindNearObject:
#               #    # changing_line = False
#               #    accumulateObjectCnt = 0
#               #    accumulateObjectLocation = 0
#               #    rospy.loginfo("    @ @ @ NEAR @@@@@@@")
#
#               elif changing_line == 0: ## TODO
#                  pid = round(pidcal.pid_control(int(x_location)), 6)
#                  previous_pid = pid
#
#               # pid = round(pidcal.pid_control(int(x_location)), 6)
#               if pid > 0.17:
#                   previous_pid = pid
#               if pid < -0.17:
#                   previous_pid = pid
#
#               auto_drive(pid)
#           else:
#               goSlow(previous_pid)
#       if cv2.waitKey(1) & 0xFF == ord('q'):
#           break
#       # cv2.imshow("origin", cv_image)
#       # out.write(cv_image)
#       #out2.write(cv_image)
#
#
#     try:
#       rospy.spin()
#     except KeyboardInterrupt:
#       print("Shutting down")
#
#     # rospy.spin()
#     # cv2.destroyAllWindows()
#
# def process_image(frame):
#     # cv2.imshow("defalut", frame)
#     # grayscle
#     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     # blur
#     kernel_size = 5
#     blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
#     # cv2.imshow("g", blur_gray)
#     # canny edge
#     low_threshold = 60#60
#     high_threshold = 70# 70
#     edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
#     # cv2.imshow("edges_img", edges_img)
#     # warper
#     img = warper.warp(edges_img)
#     # cv2.imshow("ww", img)
#     img1, x_location = slidewindow.slidewindow(img)
#     # cv2.imshow("e1", img1)
#     return img1, x_location
#
# # def process_image(frame):
# #
# #     # grayscle
# #     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
# #     # blur
# #     kernel_size = 5
# #     blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
# #     # canny edge
# #     low_threshold = 60#60
# #     high_threshold = 70# 70
# #     edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
# #
# #     # edges_img_for_stop = edge_img[width/2 - 50 : width/2 + 50, height/2 + 100: height/2 + 150]
# #
# #     lines = cv2.HoughLinesP(edges_img_for_stop, 1, np.pi/180, 100, 100, 10)
# #
# #     for i in xrange(len(lines)):
# #         for x1,y1,x2,y2 in lines[i]:
# #             cv2.line(img,(x1,y1),(x2,y2),(0,0,255),3)
# #         #slope = (atan2(y1 - y2, x1 - x2)*180 / np.pi)
# #         #print("slope:: ")
# #         #print(slope)
# #         #if(slope > 170 && slope < 190 )
# #         #    stop_slope_cnt++
# #
# #     #warper
# #     img = warper.warp(edges_img)
# #     img1, x_location = slidewindow.slidewindow(img)
# #
# #     return edges_img_for_stop, img1, x_location, stop_slope_cnt
#
# if __name__ == '__main__':
#     main()
