import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
from cap import detect

color_dist = {
              'red': {'Lower': np.array([178, 165, 175]), 'Upper': np.array([180, 180, 180])},
               #ok blue
              'blue': {'Lower': np.array([108, 140, 120]), 'Upper': np.array([114, 160, 140])},
              #green ok
              'green': {'Lower': np.array([70, 190, 120]), 'Upper': np.array([75, 200, 130])},
              #gray ok
              'gray': {'Lower': np.array([19, 10, 140]), 'Upper': np.array([34, 14, 150])},
              }
class Car:
    def __init__(self):
        self.scanSubscriber=rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Arm, queue_size=1)
    def sendtopic(self,mission):
        Arm = Arm()
        Arm.
        self.cmd_vel_pub.publish(twist)

    def image_callback(self,cap):
        if (detect(cap) == 1):
            self.sendtopic(1)
        if (detect(cap) == 2):
            self.sendtopic(2)
        if (detect(cap) == 3):
            self.sendtopic(3)
        if (detect(cap) == 4):
            self.sendtopic(4)



if __name__==__"main__":
    print('starting')
    rospy.init_node('detect', anonymous=True)
    scanner=Car()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')


