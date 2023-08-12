#!/usr/bin/python
# coding=utf-8
import rospy
from rospy import loginfo as loginfo
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped
import threading
import tf
from tf_conversions import transformations
import math

class control:
    def __init__(self):
        self.car_speed_sub = rospy.Subscriber("/odom", Odometry, self.car_speed_callback)
        self.car_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.car_pose_callback)
        self.goal_pose_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_pose_callback)
        self.goal_vel_sub = rospy.Subscriber("/my/cmd_vel", Twist, self.vel_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_info_callback)
        self.goal_path = rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.path_info_callback)

        self.goal_speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.goal_point1_pub = rospy.Publisher("/my_point1", PointStamped, queue_size=10)
        # self.goal_point2_pub = rospy.Publisher("/my_point2", PointStamped, queue_size=10)
        # self.goal_point3_pub = rospy.Publisher("/my_point3", PointStamped, queue_size=10)
        # self.server = Server(TutorialsConfig, self.callback)
        self.parm_init()
    
    def parm_init(self):
        self.teb_speed, self.teb_sheer = None, None
        self.real_x, self.real_z = None, None
        self.goal_x, self.goal_y = 0.0, 0.0
        self.car_x, self.car_y = 0.0, 0.0
        self.dO = 0.0
        self.A = 0.0
        self.B = 0.0
        self.Motor_speed, self.sheer_speed = 0.0, 0.0
        self.dx, self.dy = 0.0, 0.0
        self.Xb, self.Yb = 0.0, 0.0
        self.Path_x = [0.0 for _ in range(200)]
        self.Path_y = [0.0 for _ in range(200)]
        self.Dis_point = [0.0 for _ in range(200)]
        self.g_fP, self.g_fD = 7.0, 10.0
        self.err, self.last_err = 0, 0
        self.slowflag = 0
        self.curvity = 0
        self.g_nFroehand = 0
        self.point_A, self.point_B, self.point_C = 0, 0, 0
        
    def vel_callback(self, vel):
        self.teb_speed = vel.linear.x
        self.teb_sheer = vel.angular.z
    
    def car_speed_callback(self, car):
        self.real_x = car.twist.twist.linear.x
        self.real_z = car.twist.twist.angular.z
    
    def goal_pose_callback(self, goal):
        self.goal_x = goal.pose.position.x
        self.goal_y = goal.pose.position.y

    def car_pose_callback(self, car):
        car_x = car.pose.pose.position.x
        car_y = car.pose.pose.position.y 

    def imu_info_callback(self, imu_raw):
        x, y, z, w = imu_raw.orientation.x, imu_raw.orientation.y, imu_raw.orientation.z, imu_raw.orientation.w
        q = [x, y, z, w]
        _, _, self.dO = transformations.euler_from_quaternion(q)
        self.A = math.cos(math.radians(self.dO))
        self.B = math.sin(math.radians(self.dO))

    # def callback(self, config, level):
    #     rospy.loginfo("Reconfigure Request: %f %f %f", config.dir_P, config.rad_max, config.speed_max)
    #     self.Motor_speed = config.speed_max
    #     if self.Motor_speed > 4:
    #         self.Motor_speed = 4
    #     if self.Motor_speed < -4:
    #         self.Motor_speed = -4

    def path_info_callback(self, msg):
        size = len(msg.poses)
        for i in range(size):
            self.dx = msg.poses[i].pose.position.x - self.Xb
            self.dy = msg.poses[i].pose.position.y - self.Yb
            self.Path_x[i] = self.A * self.dx + self.B * self.dy
            self.Path_y[i] = -self.B * self.dx + self.A * self.dy
            self.Dis_point[i] = (self.Path_x[i] ** 2 + self.Path_y[i] ** 2) ** 0.5

        Max_temp = self.Dis_point[0]
        for i in range(1, size):
            if self.Dis_point[i] > Max_temp:
                Max_temp = self.Dis_point[i]
                self.g_nFroehand = i

        if self.Dis_point[self.g_nFroehand] > 0.9:
            for i in range(size):
                if self.Dis_point[i] > 0.5:
                    self.point_A = i
                    break
            self.point_B = self.point_A + 1
            if self.point_B > size - 1:
                self.point_B = size - 1
            self.point_C = self.point_B + 1
            if self.point_C > size - 1:
                self.point_C = size - 1
        else:
            self.point_A = size // 2
            self.point_B = self.point_A
            self.point_C = size - 1

        # My_point_x[0] = msg.poses[point_A].pose.position.x
        # My_point_y[0] = msg.poses[point_A].pose.position.y
        # My_point_z[0] = msg.poses[point_A].pose.position.z

        # My_point_x[1] = msg.poses[point_B].pose.position.x
        # My_point_y[1] = msg.poses[point_B].pose.position.y
        # My_point_z[1] = msg.poses[point_B].pose.position.z

        # My_point_x[2] = msg.poses[point_C].pose.position.x
        # My_point_y[2] = msg.poses[point_C].pose.position.y
        # My_point_z[2] = msg.poses[point_C].pose.position.z

        dis1 = ((self.Path_x[self.point_A] - self.Path_x[self.point_B]) ** 2 + (self.Path_y[self.point_A] - self.Path_y[self.point_B]) ** 2) ** 0.5
        dis2 = ((self.Path_x[self.point_A] - self.Path_x[self.point_C]) ** 2 + (self.Path_y[self.point_A] - self.Path_y[self.point_C]) ** 2) ** 0.5
        dis3 = ((self.Path_x[self.point_C] - self.Path_x[self.point_B]) ** 2 + (self.Path_y[self.point_C] - self.Path_y[self.point_B]) ** 2) ** 0.5
        dis = dis1 ** 2 + dis3 ** 2 - dis2 ** 2

        if dis1 * dis3 * dis2 != 0:
            cosA = dis / (2 * dis1 * dis3)  # Cosine rule to find the angle
            sinA = (1 - cosA ** 2) ** 0.5  # Find sine
            if sinA != 0:
                self.curvity = 0.5 * dis2 / sinA  # Use sine rule to find the radius of the circumscribed circle
            else:
                self.curvity = 0

            if self.curvity != 0:
                self.curvity = 1 / self.curvity  # Inverse of radius is curvature, smaller radius means larger curvature
        else:
            self.curvity = 0

        self.err = self.Path_y[self.point_A]

        different = self.err - self.last_err
        self.sheer_speed = self.g_fP * self.err + self.g_fD * different
        self.last_err = self.err

        if self.sheer_speed > 4:
            self.sheer_speed = 4
        if self.sheer_speed < -4:
            self.sheer_speed = -4

        if self.curvity > 1 and self.Path_y[self.point_A] ** 2 > 0.09:
            self.slowflag = 1
        if self.slowflag == 1 and self.Path_y[self.point_A] ** 2 > 0.01:
            self.slowflag = 0

        if self.slowflag == 0:
            self.Motor_speed = self.teb_speed
            self.sheer_speed = self.teb_sheer

        if self.slowflag == 1:
            self.Motor_speed = 0.6
            self.sheer_speed = self.sheer_speed

        goal_car_dis = 5
        goal_car_dis = ((self.car_x - self.goal_x) ** 2 + (self.car_y - self.goal_y) ** 2) ** 0.5
        if goal_car_dis < 0.4:
            self.sheer_speed = 0
            self.Motor_speed = 0

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            listener = tf.TransformListener()
            try:
                listener.waitForTransform("/odom", "/base_link", rospy.Time(0), rospy.Duration(3.0))
                (trans, rot) = listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
                self.Xb, self.Yb = trans[0], trans[1]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Transform Exception")

            my_vel = Twist()
            my_vel.linear.x = self.Motor_speed  # m/s
            my_vel.angular.z = self.sheer_speed  # rad/s
            rospy.loginfo(my_vel)
            self.goal_speed_pub.publish(my_vel)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("my_control")
    c = control()
    c.run()