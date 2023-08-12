#!/usr/bin/python2
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv_bridge
import cv2

def callback(data):
    global transport_info
    transport_info.publish(data)

def image_callback(data):
    global transport_image
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = cv2.flip(img,1)
    crop = bridge.cv2_to_imgmsg(img, 'bgr8')
    transport_image.publish(crop)

 
if __name__ == "__main__":
    rospy.init_node("transport_image_info")
    rospy.loginfo("start transport image info")
    # info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, callback)
    # transport_info = rospy.Publisher("/stereo/left/camera_info", CameraInfo, queue_size=2)
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    transport_image = rospy.Publisher("/aruco_image", Image, queue_size=2)
    rospy.spin()