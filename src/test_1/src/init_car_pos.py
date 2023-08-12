#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import time

import numpy as np
import rospy




#---新家的定义
from threading import *
from geometry_msgs.msg import  PoseWithCovarianceStamped
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
import tf2_ros
# from tf2_geometry_msgs import PointStamped
from laser_geometry import laser_geometry
import tf



#目标点数组 



           

#min_dist = 0.1 #目标点最小距离

framdid = "map"
arm_finish = False

px =-0.180
py =-0.210
ow = 0.71738
oz = 0.6966770699183099

stop=False
def my_handler(signum, frame):
    global stop
    stop = True
    rospy.loginfo("ctrl c stop")

def stop_pos_callback(data):
    global stop_pos_id
    if stop_pos_id == -1:
        stop_pos_id = data.data
    rospy.loginfo("rev stop pos callback")

def arm_finish_callback(ddata):
    global arm_finish
    rospy.loginfo("rev arm finish callback")
    if ddata.data > 0:
        arm_finish = True



def run():
    global tf_listener,rat,min_dist,target_pos,arm_finish,arm_sub,stop_pos_id,clear_costmap_pub,dete_pub,order_pub

    rospy.loginfo("init_car_pose")

    init_car_pose()

  

            

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

#设置小车起始位置在0，0
def init_car_pose():
    global init_pos_pub,clear_costmap_pub,ow,oz,px,py
    r = rospy.Rate(5)
    for i in range(3):
        pos = PoseWithCovarianceStamped()
        pos.header.stamp = rospy.get_rostime()
        pos.header.frame_id = "map"
        pos.pose.pose.position.x = px
        pos.pose.pose.position.y = py
        pos.pose.pose.orientation.x = 0
        pos.pose.pose.orientation.y = 0
        pos.pose.pose.orientation.z = oz
        pos.pose.pose.orientation.w = ow

        init_pos_pub.publish(pos)
        rospy.loginfo("init pose time {}".format(i))
        r.sleep()

    time.sleep(0.5)
    clear_costmap_pub.call() #qingli  daijia ditu 
    time.sleep(0.5)

if __name__ == "__main__":
    global tf_listener, rat, arm_sub,move_base,tf_buffer,tf_car_world,\
    init_pos_pub,stop_pos_id,reach_arm_pub,stop_pos_sub,clear_costmap_pub,dete_pub,order_pub
     
    try:

        rospy.init_node("init_car_pos", anonymous=True)
        time.sleep(0.1)
        tf_listener = tf.TransformListener()
        stop_pos_id = -1
        framdid = "map"
        move_base = SimpleActionClient("move_base", MoveBaseAction)
        tf_buffer = tf2_ros.Buffer()
        tf_car_world = tf.TransformListener(tf_buffer)


        #初始化车的位置
        init_pos_pub = rospy.Publisher("initialpose", data_class=PoseWithCovarianceStamped,queue_size=10)

		
	    #清理代价地图
        clear_costmap_pub = rospy.ServiceProxy("move_base/clear_costmaps",Empty)

        tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

        stop = False

       	run()
        
    except Exception as e1:
        rospy.loginfo(str(e1))
    rospy.loginfo("finish nav")
