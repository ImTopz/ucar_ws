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
from std_msgs.msg import String
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


#chushihua xiaoche zuobiao 
flag=0
px =-0.180
py =-0.210
ow = 0.71738
oz = 0.6966770699183099

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
    [4.174, 4.549, 1, 0.00000,0.00000,-0.0131,1.0], #B
    [4.349,2.821, 2, 0.00000,0.00000,0.04716,0.99933],   #C
    [4.163,0.711, 3, 0.00000,0.00000,-0.0131,0.99933]]   #D

    #[0.532,-1.5,3,0.00000,0.00000, 0.71733,0.69673]] #终点前中途点
    #0.501,-3.014,0,0.00000,0.00000,0.00000,0.00000]]
    #[3.021,-2.383, 3, 0.00000,0.00000,0.00702,0.99998]] #erweima 

#最后的停止区域
stop_pos = [[0.815,-0.175,0,0.00000,0.00000,-0.74529,0.66674]]  #E
           

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
    rat=rospy.Rate(1)

   
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
            rospy.loginfo("----goto C----")
            arrived = Int8()
            arrived.data = 1
            #dete_pub.publish(arrived)
            time.sleep(1) #停留时间
           
        elif misson == 2: #任务2，到机械臂位置并等待完成
 	    rospy.loginfo("goto D")
            arrived = Int8()
            arrived.data = 3
   	    time.sleep(1) #停留时间
            #clear_costmap_pub.call()

        elif misson == 3:
          
            rospy.loginfo("goto E")
            # while not rospy.is_shutdown():
            #     if stop_pos_id > -1:
            #         break
            #     time.sleep(0.1)
              
              
            #     if rospy.is_shutdown() or stop:
            #         exit(0)
            #----suiji-----
            stop_pos_id = 1 #结束位置编号 1开始
            time.sleep(1)  #wait 1 secoond then goto end 
            rospy.loginfo("got stop pos")

            goto_xy(stop_pos[stop_pos_id - 1]) #到停止位
            rospy.loginfo("goto stop pos")
            os.system("play /home/ucar/ucar_ws/src/1.mp3")
            os.system("play /home/ucar/ucar_ws/src/2.mp3")
            os.system("play /home/ucar/ucar_ws/src/3.mp3")
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
    global init_pos_pub,clear_costmap_pub,px,py,ow,oz
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
def voice_detect_callback(data):
    print("callback")
    if(data.data==1):
        global flag
        flag=1
        print("flag : %d"%flag)
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
        
        #语音识别
        voice_sub=rospy.Subscriber('awake_flag',data_class=Int8,callback=voice_detect_callback,queue_size=10)
    
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
