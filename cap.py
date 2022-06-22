import numpy as np
import cv2
import os

def detect(cap):
    frame = cap.read()
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
    inRange_hsv_blue = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv_green = cv2.inRange(hsv, color_dist['green']['Lower'], color_dist['green']['Upper'])
    inRange_hsv_red = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
    inRange_hsv_gray = cv2.inRange(hsv, color_dist['gray']['Lower'], color_dist['gray']['Upper'])
    # 分别识别四种颜色
    xy1 = np.column_stack(np.where(inRange_hsv_blue == 255))
    xy2 = np.column_stack(np.where(inRange_hsv_red == 255))
    xy3 = np.column_stack(np.where(inRange_hsv_green == 255))
    xy4 = np.column_stack(np.where(inRange_hsv_gray == 255))
    if (len(xy1) > 100):
        os.system("play")
    if (len(xy2) > 100):
        os.system("play")
    if (len(xy3) > 100):
        os.system("play")
    if (len(xy4) > 100 and len(xy1) == 0 and len(xy2) == 0 and len(xy3) == 0):
        os.system("play")
