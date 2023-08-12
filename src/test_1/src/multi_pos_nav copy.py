#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import time

import numpy as np
import rospy


import tf
#---新家的定义
from threading import *
from geometry_msgs.msg import  PoseWithCovarianceStamped
from std_msgs.msg import Int8
from std_msgs.msg import Int16

from tf_conversions import transformations
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from tf_conversions import transformations
import tf2_ros
from tf2_geometry_msgs import PointStamped
from laser_geometry import laser_geometry
import tf


#目标点数组 
#0-1 x yzuobiao
#2 shi fou you 4 yuanshu 
#3-6 siyuanshu
# target_pos = [[1.174, -0.368 , 0,0,0,0,1],
#               [2.099, -0.467 , 0,0,0,0,1],
#               [1.705, -1.109, 0,0,0,0,1],
#               [1.994, -3.378, 0,0,0,0,1],
#             [2.01, -3.176, 1, -92.96854,0.00000,0.65386,-0.75661]]

target_pos = [
    [2.01, -3.176, 1, 0,0.00000,0.65386,-0.75661]]

stop_pos = [[0,0,0,0,0,0,1],[0,0,0,0,0,0,1],[0,0,0,0,0,0,1]] #最后的停止区域

min_dist = 0.1 #目标点最小距离


np.set_printoptions(threshold=np.inf)
rospy.init_node("multi_pos_nav", anonymous=True)
rat = rospy.Rate(5)
tf_listener = tf.TransformListener()

framdid = "map"
move_base = SimpleActionClient("move_base", MoveBaseAction)
tf_buffer = tf2_ros.Buffer()
tf_car_world = tf.TransformListener(tf_buffer)


stop=False
def my_handler(signum, frame):
    global stop
    stop = True
    rospy.loginfo("ctrl c stop")


def run():
    global tf_listener,rat,min_dist,target_pos

    tf_listener.waitForTransform("/map", "/base_link", rospy.Time(),rospy.Duration(5.0))

    for i in range(len(target_pos)):
        tpos = target_pos[i]
        rospy.loginfo("goto pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
        goto_xy(tpos)
        time.sleep(0.05)
        while not rospy.is_shutdown() and not stop:

            pos = get_car_position()[:2]
            state = move_base.get_state()
           # curr_dis = math.dist(pos, tpos)
            curr_dis = get_distance(pos, tpos)
            if GoalStatus.ACTIVE != state or curr_dis < min_dist:

                rospy.loginfo("reach pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                break
            rat.sleep()

        if rospy.is_shutdown() or stop:
            exit(0)
        time.sleep(0.05)


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


def get_distance(pos1,pos2):
    return math.sqrt((pos1[0] - pos2[0])** 2 + (pos1[1] - pos2[1]) **2)


def goto_xy(pos):
    global move_base
    pos_data = MoveBaseGoal()

    pos_data.target_pose.header.stamp = rospy.Time.now()
    pos_data.target_pose.header.frame_id = "map"
    pos_data.target_pose.pose.position.x = pos[0]
    pos_data.target_pose.pose.position.y = pos[1]

    
    pos_data.target_pose.pose.orientation.x = pos[3]
    pos_data.target_pose.pose.orientation.y = pos[4]
    pos_data.target_pose.pose.orientation.z= pos[5]
    pos_data.target_pose.pose.orientation.w = pos[6]
    move_base.send_goal(pos_data)

if __name__ == "__main__":
    try:
        run()
    except Exception as e1:
        rospy.loginfo(str(e1))
    rospy.loginfo("finish nav")
