# coding=utf-8
#!/usr/bin/python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Lidar_stop:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.flag = True
        self.odom_ori_w = 0
        self.top_num = 0
        self.right_num = 0
        self.flag_stop = False
    
    def callback_w(self, msg):
        self.odom_ori_w = msg.pose.pose.orientation.w

    def callback(self, msg):
        def get_lidar(data, angle, angle_increment, radius=5):
            from math import pi

            angle = angle / 360.0 
            index = int(angle * (2*pi) / angle_increment)
            behind_data = data[index-radius if index-radius>0 else 0 : index+radius if index+radius < len(data) else len(data)]
            return behind_data
        
        data = msg.ranges
        angle_increment = msg.angle_increment
        top_data = get_lidar(data, 180, angle_increment)
        self.top_num = len([i for i in top_data if i < 0.42])
        right_data = get_lidar(data, 90, angle_increment)
        self.right_num = len([i for i in right_data if i < 0.35])
        self.flag = False
        # print(top_data, right_data)

    def run(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.callback)
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.callback_w)
        while self.flag:
            rospy.sleep(0.1)
        while True:
            v = Twist()
            v.linear.x = 0.0
            if self.top_num < 2:
                v.linear.x = 0.25
            if self.right_num < 2:
                v.linear.y = -0.25
            if abs(self.odom_ori_w) > 0.02:
                v.angular.z = -0.5 if self.odom_ori_w > 0 else 0.5

            if v.linear.y == 0 and v.linear.x == 0 and v.angular.z == 0:
                rospy.loginfo("完成停车")
                self.flag_stop = True
                break
            
            self.cmd_vel_pub.publish(v)
            rospy.sleep(0.1)

        
if __name__ == "__main__":
    rospy.init_node("lidar")
    L = Lidar_stop()
    L.run()
    # rospy.spin()