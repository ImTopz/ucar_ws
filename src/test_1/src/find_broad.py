#!/usr/bin/env python3
# coding=utf-8
from __future__ import print_function

import rospy, math
# import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
import tf2_ros
# from tf.transformations import euler_from_quaternion
# from sklearn.cluster import DBSCAN
# from threading import Event, Lock


class TargetBoardFinder:
    def __init__(
        self,
        target_frame="map",
        scan_topic="/scan",
        tf_buffer=None,
        edge_dist_threshold=0.08,
        cluster_dist_threshold=0.2,
        cluster_min_samples=10,
        visualize=False,
    ):
        pass

    def _scan_callback(self, scan_msg):
        # 获取角度最小值、最大值和分辨率
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment

        # 获取激光雷达测量到的距离和强度值
        ranges = scan_msg.ranges
        intensities = scan_msg.intensities

        # 计算所有激光点的笛卡尔坐标
        coords = []
        for i in range(len(ranges)):
            # 如果距离为nan或者强度为0，则忽略该点
            if math.isnan(ranges[i]) or intensities[i] == 0:
                continue

            # 计算当前激光点的角度
            angle = angle_min + angle_increment * i

            # 将激光点转换为笛卡尔坐标
            x = ranges[i] * math.cos(angle)
            y = ranges[i] * math.sin(angle)

            # 添加到坐标列表中
            coords.append([x, y])

        # 将坐标列表转换为Numpy数组并进行处理
        coords_np = np.array(coords)

        # 进行坐标转换等处理
        # 创建tf2_ros.TransformListener对象
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # 等待获取转换关系
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # 获取从目标坐标系到激光雷达坐标系的转换关系
                transform = tfBuffer.lookup_transform('laser', 'map', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()

        for pos in coords_np:
            # 创建一个geometry_msgs.msg.PointStamped对象，表示一个点在map坐标系中的坐标
            point_in = geometry_msgs.msg.PointStamped()
            point_in.header.frame_id = 'map'
            point_in.header.stamp = rospy.Time(0)
            point_in.point.x = pos[0]
            point_in.point.y = pos[1]
            point_in.point.z = 0.0

            # 使用tf2库将点从map坐标系转换到laser坐标系
            point_out = tfBuffer.transform(point_in, 'laser')

            # 打印转换后的点坐标
            rospy.loginfo("Point in map frame: (%.2f, %.2f, %.2f)", point_in.point.x, point_in.point.y, point_in.point.z)
            rospy.loginfo("Point in laser frame: (%.2f, %.2f, %.2f)", point_out.point.x, point_out.point.y, point_out.point.z)

        

        # 发布处理后的点云数据
        # ...

    def list(self):
        rospy.init_node('laser_scan')
        rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        rospy.spin()

if __name__ == "__main__":
    find_board = TargetBoardFinder()
    find_board.list()