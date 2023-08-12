#!/usr/bin/python2
#coding=utf-8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Lock
from Camera import Camera

class PID:
    def __init__(self):
        self.real_x, self.real_z = 0.0, 0.0
        self.goal_x, self.goal_z = 0.0, 0.0
        self.err, self.last_err = 0.0, 0.0
        self.Kp, self.Ki, self.Kd = 1.0, 1.0, 5.0
        self.Kp_z, self.Ki_z, self.Kd_z = 1.0, 1.0, 1.0
        self.z_err, self.z_last_err = 0.0, 0.0
        self.speed_x_out, self.speed_z_out = 0.0, 0.0
        self._goal_lock = Lock()
        self._speed_lock = Lock()

        self.capture_slow = False
        self.Camera = Camera()


    def carspeedCallback(self, car):

        self.real_x = car.twist.twist.linear.x
        self.real_z = car.twist.twist.angular.z

        with self._goal_lock:
            self.err = self.goal_x - self.real_x
            self.z_err = self.goal_z - self.real_z

        P = self.Kp * self.err
        self.last_err += self.err
        I = self.Ki * self.last_err

        D = self.Kd * (self.err- self.last_err)

        P_z = self.Kp_z * self.z_err
        self.z_last_err += self.z_err
        I_z = self.Ki_z * self.z_last_err

        D_z = self.Kd_z * (self.z_err - self.z_last_err)

        with self._speed_lock:
            self.speed_z_out = P_z + I_z + D_z
            self.speed_x_out = P + I + D

        self.last_err = self.err
        self.z_last_err = self.z_err

        # rospy.loginfo("goal: %f, real: %f, out: %f", goal_x, real_x, speed_x_out)
        # rospy.loginfo("goal_z: %f, real_z: %f, out_z: %f", goal_z, real_z, speed_z_out)

    def goalspeedCallback(self, goal):
        with self._goal_lock:
            self.goal_x = goal.linear.x
            self.goal_z = goal.angular.z

    def run(self):
        rospy.init_node("mypid")
        rospy.Subscriber("/odom", Odometry, self.carspeedCallback)
        rospy.Subscriber("/my/cmd_vel", Twist, self.goalspeedCallback)
        goal_speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        rospy.loginfo("启动PID")
        while not rospy.is_shutdown():
            my_vel = Twist()
            # my_vel.linear.x = self.goal_x
            # my_vel.angular.z = self.goal_z * 1.2
            with self._speed_lock:
                my_vel.linear.x = self.speed_x_out
                my_vel.angular.z = self.speed_z_out
            
            with self._goal_lock:
                rospy.loginfo("goal_v:{} {},current_v:{} {}".format(self.goal_x, self.goal_z, my_vel.linear.x, my_vel.angular.z))

            # if self.capture_slow:
            #     my_vel.linear.x = self.speed_x_out * 0.5
            #     my_vel.angular.z = self.speed_z_out * 0.5
            #     with self.camera.lock:
            #         self.camera.status = True
            #     self.Camera.start()

            goal_speed_pub.publish(my_vel)
            rate.sleep()

if __name__ == "__main__":
    try:
        pid = PID()
        pid.run()
    except rospy.ROSInterruptException:
        pass
