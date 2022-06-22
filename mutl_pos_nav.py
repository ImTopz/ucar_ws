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
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from tf_conversions import transformations
import tf2_ros
from tf2_geometry_msgs import PointStamped
from laser_geometry import laser_geometry
import tf


#目标点数组
target_pos = [[0, 0.4],
              [0.3, 0.4],
              [0.3, 0],
              [0.6, 0],
              [0.6, 0.4],
              [1, 0.4],
              [1, 0],
              [0, 0],
              [0, 0],
              [0,0]]
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
            curr_dis = math.dist(pos, tpos)
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



def goto_xy(pos):
    global move_base
    pos_data = MoveBaseGoal()

    pos_data.target_pose.header.stamp = rospy.Time.now()
    pos_data.target_pose.header.frame_id = "map"
    pos_data.target_pose.pose.position.x = pos[0]
    pos_data.target_pose.pose.position.y = pos[1]

    # pos.target_pose.pose.orientation.x = 0.5
    # pos.target_pose.pose.orientation.y = 0.5
    pos_data.target_pose.pose.orientation.w = 1
    move_base.send_goal(pos_data)

if __name__ == "__main__":
    try:
        run()
    except Exception as e1:
        rospy.loginfo(str(e1))
    rospy.loginfo("finish nav")
  
