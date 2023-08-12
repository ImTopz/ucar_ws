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
    #[3.265,-0.074,0,0.00000,0.00000,-0.71853,0.69549],#-0.71853,0.69549
    [1.923, -3.115, 1, 0.00000,0.00000,-0.66966,0.74267], #ling renwu 
    [3.100,-2.300, 2, 0.00000,0.00000,0.03647,0.99933]]#jixie bi 
    #[0.532,-1.5,3,0.00000,0.00000, 0.71733,0.69673]] #终点前中途点
    #0.501,-3.014,0,0.00000,0.00000,0.00000,0.00000]]
    #[3.021,-2.383, 3, 0.00000,0.00000,0.00702,0.99998]] #erweima 

#最后的停止区域
stop_pos = [[0.96,-0.75,0,0.00000,0.00000,0.71705,0.69702], #D1
                [0.520, -0.85,0,0.00000,0.00000, 0.71733,0.69673],#D2
                [0.000,-0.85,0,0.00000,0.00000,0.71733,0.67046]]  #D3

#min_dist = 0.1 #目标点最小距离

framdid = "map"
arm_finish = False


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

    rospy.loginfo("ready")

    init_car_pose()

    rat = rospy.Rate(1)

    for i in range(len(target_pos)):
        tpos = target_pos[i]
        rospy.loginfo("goto pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
        goto_xy(tpos)
        misson = tpos[2]
        time.sleep(0.05)
        while not rospy.is_shutdown() and not stop:

            pos = get_car_position()[:2]
            state = move_base.get_state()
           # curr_dis = math.dist(pos, tpos)
            curr_dis = get_distance(pos, tpos)
            if misson == 0:
                 if curr_dis < 0.3: #距离
                    rospy.loginfo("reach misson 0 pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                    break
            if misson == 1:
                rospy.loginfo("wait for misson 1 pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                if curr_dis < 0.35: #距离
                    rospy.loginfo("reach misson 1 pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                    break
            if misson == 2 :
                if curr_dis < 0.4: #距离
                    rospy.loginfo("reach misson 2 pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                    break
            if misson == 3:
                if curr_dis < 0.8: #距离
                    rospy.loginfo("reach misson 2 pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                    break
            if GoalStatus.ACTIVE != state:

                rospy.loginfo("reach pos [{:.3f},{:.3f}]".format(tpos[0], tpos[1]))
                break
            rat.sleep()

        if rospy.is_shutdown() or stop:
            exit(0)

          #----完成任务-----
        if misson == 1: #任务1，等待拍照
            rospy.loginfo("----wait for cam----")
            arrived = Int8()
            arrived.data = 1
            dete_pub.publish(arrived)
            time.sleep(1) #停留时间
           
        elif misson == 2: #任务2，到机械臂位置并等待完成
            arrived = Int8()
            arrived.data = 3
            rospy.loginfo("publish car reach")
            tr = rospy.Rate(5)
            for t in range(5):
                reach_arm_pub.publish(arrived)  #  发布到位信号
                tr.sleep()
            clear_costmap_pub.call()
            rospy.loginfo("wait for arm")
            curr_time = time.time()
            arm_sended = False #防止任务没识别到
            while not arm_finish: # 等待机械臂完成
                time.sleep(0.5)            
                if time.time() - curr_time > 5: #机械手等待时间
                    if arm_sended == False:
                        arm_sended = True
                        for t in range(3):
                            order = Int8()
                            order.data = 1
                            order_pub.publish(order)
                            tr.sleep()
            
                if rospy.is_shutdown() or stop:
                    exit(0)
                    break
            rospy.loginfo("got arm result")
        #elif misson == 3:
          
            rospy.loginfo("wait for stop pos")
            # while not rospy.is_shutdown():
            #     if stop_pos_id > -1:
            #         break
            #     time.sleep(0.1)
              
              
            #     if rospy.is_shutdown() or stop:
            #         exit(0)
            #----suiji-----
            stop_pos_id = 2 #结束位置编号 1开始
            #time.sleep(3)
            rospy.loginfo("got stop pos")

            goto_xy(stop_pos[stop_pos_id - 1]) #到停止位
            rospy.loginfo("goto stop pos")
            while not rospy.is_shutdown():
                state = move_base.get_state()
                if GoalStatus.ACTIVE != state:
                    rospy.loginfo("wait for reach")
                else:
                    rospy.loginfo("~~~~~~~~~~~~~~~car reach ohhhhhhhhhh~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                    exit(0)

                        
            if rospy.is_shutdown() or stop:
                exit(0)
            time.sleep(0.05)

        time.sleep(0.05)
    rospy.loginfo("exit mission thread")
       # elif misson == 3: #erwei ma 
            

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

#设置小车起始位置在0，0
def init_car_pose():
    global init_pos_pub,clear_costmap_pub
    r = rospy.Rate(5)
    for i in range(3):
        pos = PoseWithCovarianceStamped()
        pos.header.stamp = rospy.get_rostime()
        pos.header.frame_id = "map"
        pos.pose.pose.position.x = 0
        pos.pose.pose.position.y = 0
        pos.pose.pose.orientation.x = 0
        pos.pose.pose.orientation.y = 0
        pos.pose.pose.orientation.z = 0
        pos.pose.pose.orientation.w = 1

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

        rospy.init_node("multi_pos_nav", anonymous=True)
        time.sleep(0.1)
        tf_listener = tf.TransformListener()
        stop_pos_id = -1
        framdid = "map"
        move_base = SimpleActionClient("move_base", MoveBaseAction)
        tf_buffer = tf2_ros.Buffer()
        tf_car_world = tf.TransformListener(tf_buffer)

       # 发送车到达机械手的消息
        reach_arm_pub = rospy.Publisher("/ucar_msg", data_class=Int8, queue_size=1)  #
        # 接受手臂消息
        arm_sub = rospy.Subscriber("/arm_msg", data_class=Int8, callback=arm_finish_callback, queue_size=10)

        #初始化车的位置
        init_pos_pub = rospy.Publisher("initialpose", data_class=PoseWithCovarianceStamped,queue_size=10)

        #停车位接听事件
        stop_pos_sub = rospy.Subscriber("qr_res",data_class=Int16,callback=stop_pos_callback,queue_size=10)
		
	    #qqqingli daijjia ditu 
        clear_costmap_pub = rospy.ServiceProxy("move_base/clear_costmaps",Empty)

        #垃圾分为
        dete_pub = rospy.Publisher("ucar_1", data_class=Int8,queue_size=1)

        #垃圾分类预防
        order_pub =  rospy.Publisher("mission", data_class=Int8,queue_size=1)
        

        tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

        

        #----
        # r = rospy.Rate(5)
        # while True:
        #     arrived = Int8()
        #     arrived.data = 3
        #     reach_arm_pub.publish(arrived)
        #     r.sleep()
        #     rospy.loginfo("send_car_reach")
         

        #-----
        stop = False

        main_thread = Thread(target=run)
        main_thread.start()

        rospy.spin()
    except Exception as e1:
        rospy.loginfo(str(e1))
    rospy.loginfo("finish nav")
