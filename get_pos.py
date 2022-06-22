#!/usr/bin/env python
import json
import math
import time

import numpy as np
import rospy

import tcp_data, tcp_msg_data
import tf
from tcp_msg_data import image_data, car_position
from tf_conversions import transformations
from nav_msgs.msg import OccupancyGrid
import threading

import signal

np.set_printoptions(threshold=np.inf)
got_map = False


rospy.init_node("get_car_position", anonymous=True)
rat = rospy.Rate(1)
tf_listener = tf.TransformListener()

stop=False
def my_handler(signum, frame):
    global stop
    stop = True
    rospy.loginfo("ctrl c stop")

# 设置相应信号处理的handler
signal.signal(signal.SIGINT, my_handler)  # 读取Ctrl+c信号


def run():
    global tf_listener,rat,stop

    tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    while not rospy.is_shutdown() and not stop:
        try:

            pos = get_car_position()
            rospy.loginfo("car pos :{:.3f},{:.3f}".format(pos[0],pos[1]))
            rat.sleep()
        except Exception as e1:
            print(str(e1))
        time.sleep(1)


def get_car_position():
    global tf_listener
    try:
        (trans, rot) = tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        euler = transformations.euler_from_quaternion(rot)  # 将四元数转换为欧拉角

        x = trans[0]
        y = trans[1]
        th = euler[2] / math.pi * 180
        return (x, y, th)
    except Exception as e1:
        rospy.loginfo(str(e1))

    return None


if __name__ == "__main__":
    try:
        run()
    except Exception as e1:
        rospy.loginfo(str(e1))
