#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import time

import numpy as np
import rospy
from std_msgs.msg import Int8
import tf
# from tf_conversions import transformations
from nav_msgs.msg import OccupancyGrid
import math
import threading
# from tf2_py import  *
# import tf2_py
# from tf2_tools import _euler_from_quaternion_msg

#import signal

np.set_printoptions(threshold=np.inf)
got_map = False


rospy.init_node("get_car_position", anonymous=True)
rat = rospy.Rate(1)
tf_listener = tf.TransformListener()

stop=False
# def my_handler(signum, frame):
#     global stop
#     stop = True
#     rospy.loginfo("ctrl c stop")

# 设置相应信号处理的handler
#signal.signal(signal.SIGINT, my_handler)  # 读取Ctrl+c信号


def run():
    global tf_listener,rat,stop

    tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    #---CESHI DAI MA ----
    #JI XIE SHOU 
    # arm_pub = rospy.Publisher("/arm_msg",data_class=Int8,queue_size=1)
    # stop_pub = rospy.Publisher("/stop_pos",data_class=Int8,queue_size=1)
    # cam_pub = rospy.Publisher("/ucar_1",data_class=Int8,queue_size=1)
    # data = Int8()
    # data.data = 1
    # rt = rospy.Rate(5)
    # while not stop:
    #     cam_pub.publish(data)
    #     rt.sleep()
    # rospy.loginfo("send_arm_smgs")
    # time.sleep(0.5)

    # data.data = 3
    # for i in range(5):
    #     stop_pub.publish(data)
    #     rt.sleep()
    # rospy.loginfo("send_stop_smgs")
    # time.sleep(0.5)

 

    while not rospy.is_shutdown() and not stop:
        try:
            # arm_pub.publish(data)
            pos,ori = get_car_position()
        
            rospy.loginfo("car pos :{:.3f},{:.3f},{:.3f},ori:{:.5f},{:.5f},{:.5f},{:.5f}".format(pos[0],pos[1],pos[2],ori[0],ori[1],ori[2],ori[3]))
            distance = math.sqrt((pos[0] - 1.008)**2 + (pos[1] + 3.273)**2)
            rospy.loginfo("distance of F is " + str(distance))
            rat.sleep()
        except Exception as e1:
            print(str(e1) + "1")
        time.sleep(1)


def get_car_position():
    global tf_listener
    try:
        (trans, rot) = tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)  # 将四元数转换为欧拉角
        # euler = _euler_from_quaternion_msg(rot)

        x = trans[0]
        y = trans[1]
        th = euler[2] / math.pi * 180
        rospy.loginfo(str(rot))
        return (x, y, th), rot
    except Exception as e1:
        rospy.loginfo(str(e1))

    return None


if __name__ == "__main__":
    try:
        run()
    except Exception as e1:
        rospy.loginfo(str(e1))
