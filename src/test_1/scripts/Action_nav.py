#!/usr/bin/python2
# coding=utf-8
import rospy
from NavNode import RosNavNode
import traceback
import time

POSE_F = True
POSE_M = True
POSE_B = True
POSE_C = True
POSE_D = True
POSE_E = True
POSE_A = True
POSE_FI = True

def main():
    global POSE_F
    node = RosNavNode()
    rospy.loginfo("[+] ===== 初始化完毕,等待语音唤醒[+],唤醒词:小张小张！ =====")
    # while True:
    #     try:
    #         state = rospy.get_param('nav_state') #rospy.set_param('nav_state',0)
    #         if(state == 1):
    #             break
    #     except:
    #         pass

    
    
    rospy.loginfo("[+] =====  开始导航 =====")
    node.init_pose()
    node.get_init_pose()

    GOAL_LIST = ['D','C','B','F1','F2','Final'] #需要到达的goal
    PICTURE_LIST = ['F1','F2','E','D','C','B']
    # GOAL_LIST = ['F1', 'F2','Final']
    GOAL_STATUS = { 'F1':POSE_F,
                    'F2':POSE_F,
                    'M1':POSE_M,
                    'M2':POSE_M,
                    'M3':POSE_M,
                    'M4':POSE_M,
                    'E':POSE_E,
                    'D':POSE_D,
                    'C':POSE_C,
                    'B':POSE_B,
                    'Final':POSE_FI,
                    'A':POSE_A
                    } #确认是否需要到达
    flag = True
    start_time = time.time()

    for _goal in GOAL_LIST:
        current_time = time.time()    
        if GOAL_STATUS[_goal]:
            rospy.loginfo("正在前往{}区～".format(_goal))
            node.get_pose("{}".format(_goal))
            node.send_goal()
            rospy.loginfo("正在等待到达{}区~".format(_goal))
            while True:
                rospy.sleep(0.01)
                if node.check_if_pose_near(node.pos_x,node.pos_y,radius=0.5):
                    break
            if _goal in PICTURE_LIST:
                node.mission(goal=_goal)
                while node.camera.status:
                    rospy.sleep(0.1)
            elif _goal[0] == 'F' and not node.detect_F_flag:
                node.mission(goal=_goal)
                while node.camera.status:
                    rospy.sleep(0.1)
        current_cost = time.time() - current_time
        rospy.loginfo("到达{}区,并完成任务花费时间:{}秒".format(_goal, current_cost))
    rospy.loginfo("[+] ====== 开始检测 ======")
    end_time = time.time()
    run_time = end_time - start_time
    # rospy.loginfo("停车前运行时间：{} 秒".format(run_time))
    node.arrive_goal()
    while not node.lidar_stop.flag_stop:
        rospy.sleep(0.1)
    end_time = time.time()

    # 计算运行时间
    run_time = end_time - start_time

    rospy.loginfo("运行时间：{} 秒".format(run_time))


if __name__ == "__main__":
    try:
        main()
    except:
        rospy.loginfo("Error!")
        traceback.print_exc()
