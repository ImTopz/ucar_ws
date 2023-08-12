#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time

import numpy as np
import rospy

from init_car_pos import  my_handler, stop_pos_callback, get_distance,init_car_pose


#---新家的定义
from threading import *
from geometry_msgs.msg import  PoseWithCovarianceStamped
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_srvs.srv import Empty
# from tf_conversions import transformations
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
# from tf_conversions import transformations
import tf2_ros
# from tf2_geometry_msgs import PointStamped
from laser_geometry import laser_geometry
import tf

target_pos = [
    # [0.077,-0.064, 0, 0, 0, -0.70433, 0.71004]
    [0.966,-3.305,1,0.00000,0.00000,-0.70676,0.70745],
    [2.894,-2.537,2,0.00000,0.00000,-0.00392,0.99999],
    [5.036,-2.560,3,0.00000,0.00000,0.69771,0.71638]]

stop_pos = [ [0.077,-0.064, 0, 0, 0, -0.70433, 0.71004],[0,0,0,0,0,0,1],[0,0,0,0,0,0,1]] #最后的停止区域

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

def get_car_position():
    global tf_listener
    try:
        (trans, rot) = tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)  # 将四元数转换为欧拉角

        x = trans[0]
        y = trans[1]
        th = euler[2] / math.pi * 180
        return (x, y, th)
    except Exception as e1:
        rospy.loginfo(str(e1))

def run():
    global tf_listener,rat,stop, move_base

    tf_listener.waitForTransform("/map","/base_link",rospy.Time(),rospy.Duration(5.0))

    for i in range(len(target_pos)):
        tpos = target_pos[i]
        goto_xy(tpos)
        time.sleep(0.5)

        while not rospy.is_shutdown() and not stop:
            pos = get_car_position()[:2]
            state = move_base.get_state()
            curr_dis = get_distance(pos, tpos)
            if curr_dis<0.3:
                break
            rat.sleep()


        # if rospy.is_shutdown() or stop:
        #     exit(0)
    goto_xy(stop_pos[0])
    while not rospy.is_shutdown():
        state = move_base.get_state()
        if GoalStatus.ACTIVE == state:
            rospy.loginfo(" reach")
            exit(0)
        


if __name__ == "__main__":
    try:
        rospy.init_node("multi_pos_nav", anonymous=True)
        time.sleep(0.1)
        tf_listener = tf.TransformListener()
        stop_pos_id = -1
        framdid = "map"
        move_base = SimpleActionClient("move_base", MoveBaseAction)
        tf_buffer = tf2_ros.Buffer()
        tf_car_world = tf.TransformListener(tf_buffer)


        #初始化车的位置
        init_pos_pub = rospy.Publisher("initialpose", data_class=PoseWithCovarianceStamped,queue_size=10)

        #停车位接听事件
        stop_pos_sub = rospy.Subscriber("qr_res",data_class=Int16,callback=stop_pos_callback,queue_size=10)


        tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))


        stop = False

        main_thread = Thread(target=run)
        main_thread.start()

        rospy.spin()
    except Exception as e1:
        rospy.loginfo(str(e1))
    rospy.loginfo("finish nav")