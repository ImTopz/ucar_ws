# coding=utf-8
#! /usr/bin/env python3

import rospy
from threading import Lock
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist

class Aruco:
    def __init__(self):
        
        self.pose_x = 0.0
        self.pose_z = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0

        self.aruco_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray,self._get_info)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._info_lock = Lock()

    def _get_info(self, msg):
        with self._info_lock:
            self.pose_x = 0.0
            self.pose_z = 0.0
            self.ori_z = 0.0
            self.ori_w = 0.0

    def stop(self):
        while True:
            speed = Twist()
            pose_x = self.pose.x
            pose_z = self.pose.z
            ori_w = self.ori.w
            if abs(pose_x) > 0.1:
                speed.line.y = 2 if pose_x > 0 else -2
            if abs(pose_z) > 0.5:
                speed.line.x = 2
            if abs(ori_w) > 0.1:
                speed.angular.z = 1
            if speed.line.y == 0 and speed.line.x == 0 and speed.angular.z == 0:
                break
            self.cmd_pub.publish(speed) 
                
        
if __name__ == "__main__":
    pass