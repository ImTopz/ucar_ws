# coding=utf-8
#!/usr/bin/python2
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import json
import os

class Sound:
    def __init__(self):
        self.path = "/home/ucar/ucar_ws/src/test_1/scripts/voices/"
        self.message_sub = rospy.Subscriber('/yolov_predict', String, self.callback)
        self.start_publisher = rospy.Publisher('/yolov_start', String, queue_size=1)

    def start_pub(self):
        message = String()
        message.data = "1"
        self.start_publisher.publish(message)
    
    def callback(self, msg):
        image_classnums =  json.loads(msg.data)
        GOAL_LIST = ['B','C','D','E']
        classes_v = ["cucumber","rice","maize","wheat"]
        goal_dict = {key:None for key in GOAL_LIST}
        classes_dict = {key:False for key in classes_v}

        classes_f = ["watermelon","maize","cucumber"]
        rospy.loginfo(image_classnums)
        # 开始
        # os.system("play {}".format(self.path + "任务完成.mp3"))
        for goal in GOAL_LIST:
            classnums = image_classnums[goal]
            max_value = max(classnums)
            max_index = classnums.index(max_value)
            if max_value != 0:
                goal_dict[goal] = classes_v[max_index]
                classes_dict[classes_v[max_index]] = True

        for goal in GOAL_LIST:
            # os.system("play {}".format(self.path + goal + ".mp3"))
            if not goal_dict[goal]:
                for classes in classes_dict:
                    if not classes_dict[classes]:
                        classes_dict[classes] = True
                        goal_dict[goal] = classes
                        break 
            # os.system("play {}".format(self.path + goal_dict[goal] + ".mp3"))
        rospy.loginfo(goal_dict)
        if image_classnums['F1'][-1] != 5:
            for index, i in enumerate(image_classnums['F2']):
                image_classnums['F1'][index] += i
        
        classnums = image_classnums['F1']
        max_value = max(classnums[:-1])
        max_index = classnums.index(max_value)
        rospy.loginfo("{} {}".format(classes_f[max_index], max_value))
        # os.system("play {}".format(self.path + "F" + ".mp3"))
        # os.system("play {}".format(self.path + classes_f[max_index] + ".mp3"))
        # os.system("play {}".format(self.path + str(int(max_value)) + "个.mp3"))
        rospy.sleep(2)

    def play(self, image_classnums, classes_dict):
        GOAL_LIST = ['B','C','D','E']
        classes_v = ["cucumber","rice","maize","wheat"]

        classes_f = ["watermelon","maize","cucumber"]
        rospy.loginfo(image_classnums)
        # 开始
        # os.system("play {}".format(self.path + "任务完成.mp3"))

        for goal in GOAL_LIST:
            # os.system("play {}".format(self.path + goal + ".mp3"))
            if image_classnums[goal] != None:
                for classes in classes_dict:
                    if not classes_dict[classes]:
                        classes_dict[classes] = True
                        image_classnums[goal] = classes
                        break 
            # print(image_classnums[goal], goal, classes_dict)
            # os.system("play {}".format(self.path + image_classnums[goal] + ".mp3"))
        classnums = image_classnums['F'][:-1]
        max_value = max(classnums)
        max_index = classnums.index(max_value)
        # os.system("play {}".format(self.path + "F" + ".mp3"))
        # os.system("play {}".format(self.path + classes_f[max_index] + ".mp3"))
        # os.system("play {}".format(self.path + str(int(max_value)) + "个.mp3"))
        rospy.sleep(2)

# if __name__ == "__main__":
    # rospy.init_node("sound")
    # s = Sound()
    # s.start()
    # rospy.spin()