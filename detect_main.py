#!/usr/bin/env python2
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Int8
import os

color_dist = {
    'red': {'Lower': np.array([178, 161, 200]), 'Upper': np.array([181, 179, 217])},
    # ok blue
    'blue': {'Lower': np.array([106, 183, 121]), 'Upper': np.array([114, 210, 142])},
    # green ok
    'green': {'Lower': np.array([79, 180, 120]), 'Upper': np.array([90, 220, 140])},
    # gray ok
    'gray': {'Lower': np.array([120, 10, 130]), 'Upper': np.array([140, 30, 140])},
}


def detect(cap):
    frame = bridge.imgmsg_to_cv2(cap, "bgr8")
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    inRange_hsv_blue = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv_green = cv2.inRange(hsv, color_dist['green']['Lower'], color_dist['green']['Upper'])
    inRange_hsv_red = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
    inRange_hsv_gray = cv2.inRange(hsv, color_dist['gray']['Lower'], color_dist['gray']['Upper'])
    xy1 = np.column_stack(np.where(inRange_hsv_blue == 255))
    xy2 = np.column_stack(np.where(inRange_hsv_red == 255))
    xy3 = np.column_stack(np.where(inRange_hsv_green == 255))
    xy4 = np.column_stack(np.where(inRange_hsv_gray == 255))
    if (len(xy1) > 100):
        return 1
    if (len(xy2) > 100):
        return 2
    if (len(xy3) > 100):
        return 3
    if (len(xy4) > 100 and len(xy1) == 0 and len(xy2) == 0 and len(xy3) == 0): #gray
        return 4


class Car:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/image_view/output', Image, self.image_callback)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Arm, queue_size=1)

    # def sendtopic(self,mission):
    # self.cmd_vel_pub.publish(twist)

    def image_callback(self, cap):
        print("starting 2")
        if (detect(cap) == 1):
            os.system("play ~/ucar-master/src/mp3/shuiguo.mp3")
            #mp3_play
            arm_pub.publish(1)
            car.image_sub.unregister()

        if (detect(cap) == 2):
            print("blue")
            os.system("play ~/ucar-master/src/mp3/shuiguo.mp3")
            arm_pub.publish(2)
            #mp3_play1
            car.image_sub.unregister()
        if (detect(cap) == 3):
            print("blue")
            os.system("play ~/ucar-master/src/mp3/shucai.mp3")
            arm_pub.publish(3)
            # mp3_play1
            car.image_sub.unregister()
        if (detect(cap) == 4):
            print("blue")
            os.system("play ~/ucar-master/src/mp3/roulei.mp3")
            arm_pub.publish(4)
            # mp3_play1
            car.image_sub.unregister()


if __name__ == "__main__":
    print('starting')
    flag =0
    rospy.init_node('detect')
    rospy.init_node('arm')
    arm_pub=rospy.Publisher('/Arm_mode', int, queue_size=10)
    car = Car()
    bridge = CvBridge()
    # ros_frame.step = 1920
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')
