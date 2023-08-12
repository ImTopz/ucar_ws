#!/usr/bin/env python2

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Int8
import os

color_dist = {
    #
    # 'red': {'Lower': np.array([0, 190, 230]), 'Upper': np.array([5, 200, 246])},
    #  #'red_1': {'Lower': np.array([170, 190, 230]), 'Upper': np.array([180, 200, 246])},

    # # ok blue
    # 'blue': {'Lower': np.array([100, 220, 172]), 'Upper': np.array([110, 250, 182])},
    # # green ok
    # 'green': {'Lower': np.array([77, 252, 146]), 'Upper': np.array([80, 255, 163])},
    # # gray ok
    # 'gray': {'Lower': np.array([120, 0, 160]), 'Upper': np.array([140, 3, 170])},

    #-----------------------------------------
    'red': {'Lower': np.array([0, 100, 100]), 'Upper': np.array([30, 255, 255])},
    'red1': {'Lower': np.array([250, 100, 100]), 'Upper': np.array([255, 255, 255])},
    # ok blue
    'blue': {'Lower': np.array([160, 20, 80]), 'Upper': np.array([240, 255, 255])},
    # green ok
    'green': {'Lower': np.array([51, 50, 80]), 'Upper': np.array([90, 255, 255])},
    # gray ok
    'gray': {'Lower': np.array([120, 0, 0]), 'Upper': np.array([150, 255, 255])},
}


def detect(cap):
    frame = bridge.imgmsg_to_cv2(cap, "bgr8")
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    inRange_hsv_blue = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    inRange_hsv_green = cv2.inRange(hsv, color_dist['green']['Lower'], color_dist['green']['Upper'])
    inRange_hsv_red = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
    inRange_hsv_gray = cv2.inRange(hsv, color_dist['gray']['Lower'], color_dist['gray']['Upper'])
    
    #--red buchong ---
    inRange_hsv_red_1 = cv2.inRange(hsv, color_dist['red1']['Lower'], color_dist['red']['Upper'])
    xy1_1 =  np.column_stack(np.where(inRange_hsv_red_1 == 255))

    xy1 = np.column_stack(np.where(inRange_hsv_blue == 255))
    xy2 = np.column_stack(np.where(inRange_hsv_red == 255))
    xy3 = np.column_stack(np.where(inRange_hsv_green == 255))
    xy4 = np.column_stack(np.where(inRange_hsv_gray == 255))

    
    #-----xiu gai wei zuida zhi shuchu ----
    cred = len(xy1)+ len(xy1_1)
    cblue = len(xy2) 
    cgreen = len(xy3)
  
    if cred > cblue and cred > cgreen:
        return 4
    elif cblue > cred and cblue > cgreen:
        return 3
    elif cgreen >  cred and cgreen > cblue:
        return 2
    
    td = (cred,cgreen,cblue)
    max_index = np.max(td)
    if td[max_index] < 100:
        return 1
        
    #---------

    # if (len(xy1)+ len(xy1_1) > 100):
    #     return 4
    # if (len(xy2) > 100):
    #     return 3
    # if (len(xy3) > 100):
    #     return 2
    # if (len(xy4) > 100 and len(xy1) == 0 and len(xy2) == 0 and len(xy3) == 0): #gray # !-ruguo Rgb-!
    #     return 1

    return 1

class Car:

  

    def __init__(self):
        self.arrive = -1
        #self.car_sub =  rospy.Subscriber('/ucar_1', Int8, self.arrive_callback)
        self.image_sub = rospy.Subscriber('/image_view/output', Image, self.image_callback)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Arm, queue_size=1)
    

    # def sendtopic(self,mission):
    # self.cmd_vel_pub.publish(twist)

    def arrive_callback(self,data):
        rospy.loginfo("car_reach")
        if data.data == 1:
            self.arrive = 1

    def image_callback(self, cap):
        print("你好")
        if self.arrive < 0:
            return 
        #print("starting 2")
        if (detect(cap) == 1):
            print("grey")
            os.system("play ~/ucar_ws/src/mp3/ganlaji.mp3")
            #mp3_play
            a.data=1
            arm_pub.publish(a)
            print("fabu chenggong!")
            car.image_sub.unregister()

        if (detect(cap) == 2):
            print("green")
            os.system("play ~/ucar_ws/src/mp3/shilaji.mp3")
            a.data=2
            arm_pub.publish(a)
            
            #mp3_play1
            car.image_sub.unregister()
        if (detect(cap) == 3):
            print("red")
            os.system("play ~/ucar_ws/src/mp3/youhailaji.mp3")
            a.data=3
            arm_pub.publish(a)
            print("ok!!!!!!!!!!!!!!")
            # mp3_play1
            car.image_sub.unregister()
        if (detect(cap) == 4):
            print("blue")
            os.system("play ~/ucar_ws/src/mp3/kehuishou.mp3")
            a.data=4
            arm_pub.publish(a)
            car.image_sub.unregister()



if __name__ == "__main__":
    print('starting')
    flag =0
    rospy.init_node('detect')
    arm_pub=rospy.Publisher('/mission', Int8, queue_size=1)
    car = Car()
    bridge = CvBridge()
    a=Int8()
    # ros_frame.step = 1920
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')

'''

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
 
    rospy.Subscriber("chatter", String, callback)
    print("start")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
'''
