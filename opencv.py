
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os

color_dist = {'red': {'Lower': np.array([0, 25, 138]), 'Upper': np.array([19, 255, 255])},
              'light_red': {'Lower': np.array([178, 100, 136]), 'Upper': np.array([255, 255, 255])},
              'blue': {'Lower': np.array([108, 140, 120]), 'Upper': np.array([114, 160, 140])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},
              }
#颜色定义
# 调用摄像头
# cap = cv2.VideoCapture(0)
# 输入视频
cap = cv2.VideoCapture(0)


while True:
    # 读取视频帧，ret标志读取的结果，frame为读取到的视频帧图像
    ret, frame = cap.read()

    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像

    inRange_hsv_blue = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv_green = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    #寻找蓝色点的集合
    xy1=np.column_stack(np.where( inRange_hsv_blue==255))
    if(len(xy1)>100):
        os.system("play ~/ucar_ws/src/mp3/0longhair.mp3");


    # 寻找外部的点
    cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    c = max(cnts, key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box_list = box.tolist()

    # 将点画在
    cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)

    cv2.imshow('camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
