# coding=utf-8
#! /usr/bin/env python3
import rospy
import tf2_ros
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
import numpy as np

class Aruco:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 选择要使用的 ArUco 字典
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

        # 创建 ArUco 检测器
        self.parameters = aruco.DetectorParameters_create()

    def run(self):
        # 初始化摄像头（根据摄像头编号或文件路径进行设置）
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()

            #轴对称翻转照片，否则二维码是镜像的
            frame = cv2.flip(frame,1)

            #加载aruco字典，本次比赛使用的是4x4的aruco码
            
            #将图像转换为灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            #检测aruco码的角点信息
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            # #如果检测到aruco码，输出其编号
            # if ids is not None:
            #     qr_pub.publish(ids[0])
            #     print("ok")
            if ids is not None:
                for i in range(len(ids[0])):
                    #绘制出aruco码的外框    
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    aruco_corners = corners[i][0]

                    center_x = int((corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4)
                    center_y = int((corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4)
                    aruco_center = (center_x, center_y)
                    # print(aruco_center, aruco_corners)

                    self.transfor(aruco_corners, aruco_center)
                break
            # cv2.imshow("frame",frame)

            # if ids is not None:
                # print(ids[0], rejectedImgPoints)

            # 按下 'q' 键退出循环

        # 释放摄像头资源并关闭窗口
        cap.release()
        cv2.destroyAllWindows()

    def transfor(self, aruco_corners, aruco_center):
        # 定义ArUco标记的坐标系名称和小车坐标系名称
        aruco_frame = 'aruco_frame'
        car_frame = 'odom'
        # 等待获取坐标变换
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(car_frame, aruco_frame, rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("等待坐标变换...")
        
        # 获取变换矩阵 T_aruco_to_car
        T_aruco_to_car = np.array([
            [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z],
            [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 0],
            [0, 0, 0, 1]
        ])
        # 转换ArUco标记中心点的坐标到小车坐标系下
        aruco_center_point = PointStamped()
        aruco_center_point.header.frame_id = aruco_frame
        aruco_center_point.point.x, aruco_center_point.point.y = aruco_center
        aruco_center_point.point.z = 0.0  # 假设在ArUco标记平面上
        transformed_center_point = tf_buffer.transform(aruco_center_point, car_frame)

        # 创建一个PoseStamped消息来保存相对于小车的位姿信息
        relative_pose = PoseStamped()
        relative_pose.header.frame_id = car_frame
        relative_pose.pose.position.x = transformed_center_point.point.x
        relative_pose.pose.position.y = transformed_center_point.point.y
        relative_pose.pose.position.z = transformed_center_point.point.z
        print(relative_pose.pose.position)

if __name__ == "__main__":
    rospy.init_node('aruco_to_car_relative_coordinate')
    A = Aruco()
    A.run()

