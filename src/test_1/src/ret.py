#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import json

class Sound:
    def __init__(self):
        # self.message_sub = rospy.Subscriber('/yolov_predict', String, self.callback)
        # self.sound = list()
        self.start_publisher = rospy.Publisher('/yolov_start', Int8, queue_size=1)

    def start(self):
        message = Int8()
        message.data = 1
        self.start_publisher.publish(message)

if __name__ == "__main__":
    rospy.init_node("ds")
    s = Sound()
    s.start()
    rospy.spin()