# coding=utf-8
#!/usr/bin/python2
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from threading import Lock
import rospy
import shutil

class Camera:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.status = False
        self.lock = Lock()

    def start(self, goal, picture_num):
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
        self.goal = goal
        self.picture_num = picture_num
        # self.status = False

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.flip(img,1)
        file_name = './tmp_images/image_{}_{}.jpg'.format(self.goal, self.picture_num)
        target_file_name = './images/image_{}_{}.jpg'.format(self.goal, self.picture_num)
        cv2.imwrite(file_name, img)
        shutil.move(file_name, target_file_name)
        rospy.loginfo("图片{}_{} 成功保存".format(self.goal, self.picture_num))
        self.image_sub.unregister()
        with self.lock:
            self.status = False


if __name__ == "__main__":
    rospy.init_node("camera")
    c = Camera()
    rospy.spin()