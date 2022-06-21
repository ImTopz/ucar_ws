import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os

color_dist = {
              'red': {'Lower': np.array([178, 165, 175]), 'Upper': np.array([180, 180, 180])},
               #ok blue
              'blue': {'Lower': np.array([108, 140, 120]), 'Upper': np.array([114, 160, 140])},
              #green ok
              'green': {'Lower': np.array([70, 190, 120]), 'Upper': np.array([75, 200, 130])},
              #gray ok
              'gray': {'Lower': np.array([19, 10, 140]), 'Upper': np.array([34, 14, 150])},
              }
#颜色定义

cap = cv2.VideoCapture(0)

rospy.init_node('detect', anonymous=True)
while True:
    # 读取视频帧，ret标志读取的结果，frame为读取到的视频帧图像
    ret, frame = cap.read()

    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像

    inRange_hsv_blue = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv_green = cv2.inRange(hsv, color_dist['green']['Lower'], color_dist['green']['Upper'])
    inRange_hsv_red = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
    inRange_hsv_gray = cv2.inRange(hsv, color_dist['gray']['Lower'], color_dist['gray']['Upper'])
    #寻找蓝色点的集合
    xy1=np.column_stack(np.where( inRange_hsv_blue==255))
    if (len(xy1) > 100):
        print("识别到了蓝色")
    if (len(xy2) > 100):
        print("识别到了红色")
    if (len(xy3) > 100):
        print("识别到了绿色")
    if (len(xy4) > 100 and len(xy1) == 0 and len(xy2) == 0 and len(xy3) == 0):
        print("识别到了灰色")
    cv2.imshow('camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
