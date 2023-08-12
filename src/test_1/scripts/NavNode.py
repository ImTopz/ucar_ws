# coding=utf-8
#!/usr/bin/python2
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from threading import Lock
import threading
import json
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from Camera import Camera
from Detect_Sound import Sound
from lidar_stop import Lidar_stop
from std_msgs.msg import String
import time

class RosNavNode:  #目标点发送的
    def __init__(self):
        rospy.init_node("Nav_Node",anonymous=False)
        self.client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.odom_subscriber = rospy.Subscriber('/odom',Odometry,self._get_info)
        rospy.loginfo("[+] ===== 启动声音模块 ======")
        self.sound = Sound()

        rospy.loginfo("[+] ===== 启动摄像头模块 ======")
        self.camera = Camera()
        rospy.loginfo("[+] ===== 成功启动摄像头模块 ======")
        self.lidar_stop = Lidar_stop()

        self.last_distance = 0
        self.odom_pose_x = 0.0
        self.odom_pose_y = 0.0
        self.odom_twist_linear = 0.0
        self.odom_twist_angular = 0.0 
        self.odom_ori_z = 0.0
        self.odom_ori_w = 0.0

        self.move_base_status = False
        self.move_base_sum = 1
        self.move_flag = True

        self._info_lock = Lock()
        self._status_lock = Lock()
        rospy.loginfo("[+] ===== 加锁完成 ======")
        self.client.wait_for_server()
        rospy.loginfo("[+] ===== 等待运动服务器 ======")
        self.pass_thres_radius = 0.05
        self.init_parm_detect()
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.move_base_sub = rospy.Subscriber('/move_base/status', MoveBaseActionGoal, self._get_status)
        self.clear_costmap_pub = rospy.ServiceProxy("move_base/clear_costmaps",Empty)
        self.my_vel_sub = rospy.Subscriber("/my/cmd_vel", Twist, self._slow_Callback)
        rospy.loginfo("[+] ===== 导航节点初始化完毕 ======")
    
    def init_parm_detect(self):
        self.GOAL_LIST = ['B','C','D','E']
        self.classes_v = ["cucumber","rice","maize","wheat"]
        self.classes_f = ["watermelon","maize","cucumber"]
        self.goal_dict = {key:None for key in self.GOAL_LIST}
        self.goal_dict['F'] = [0 for _ in range(len(self.classes_f)+1)]
        self.classes_dict = {key:False for key in self.classes_v}
        self.detect_lock = False
        self.detect_goal = ""
        self.fisrt_F = False
        self.fisrt_F2 = False
        self.detect_F_flag = False
        self.detect_other_flag = {key:False for key in self.GOAL_LIST}
        self.detect_other_image_0 = {key:False for key in self.GOAL_LIST}
        self.detect_other_lock = Lock()

        self.detect_end_status = False

        self.change_model_pub = rospy.Publisher('/yolov_change',String, queue_size=2)
        self.detect_sub = rospy.Subscriber('/yolov_predict_2', String, self._get_detect)
        self.end_pub = rospy.Publisher('/mission_end', String, queue_size=1)
        self.detect_end_pub = rospy.Subscriber('/detect_end', String, self._detect_end)

    def _detect_end(self, msg):
        self.detect_end_status = True

    def _get_detect(self, msg):
        data = json.loads(msg.data)
        goal = data.keys()[0]
        if goal[0] != "F":
            classnums = data[goal]
            max_value = max(classnums)
            max_index = classnums.index(max_value)
            if max_value != 0:
                self.goal_dict[goal[0]] = self.classes_v[max_index]
                self.classes_dict[self.classes_v[max_index]] = True
                with self.detect_other_lock:
                    self.detect_other_flag[goal] = True
                rospy.loginfo("接收到{}任务点 {}".format(goal, self.detect_other_flag[goal]))
        else:
            for index, i in enumerate(data[goal]):
                if i != 0:
                    self.goal_dict[goal][index] += i
            if not self.fisrt_F:
                if self.goal_dict[goal][-1] == 3:
                    self.detect_F_flag = True
                self.fisrt_F = True
            if goal == "F2" and not self.fisrt_F2:
                if data[goal][-1] == 1:
                    self.detect_F_flag = True
                self.fisrt_F2 = True

    def _get_info(self, msg):
        """
        获取base_driver发来的odometry消息
        """
        with self._info_lock:
            self.odom_pose_x = msg.pose.pose.position.x
            self.odom_pose_y = msg.pose.pose.position.y
            self.odom_ori_w = msg.pose.pose.orientation.w
            self.odom_ori_z = msg.pose.pose.orientation.z
            self.odom_twist_linear = math.sqrt(msg.twist.twist.linear.x **2 + msg.twist.twist.linear.y**2)
            self.odom_twist_angular = math.sqrt(msg.twist.twist.angular.x **2 + msg.twist.twist.angular.y**2)
    
    def _get_status(self, msg):
        with self._status_lock:
            try:
                flag = False
                for i in msg.status_list:
                    if i.status == 1:
                        self.move_flag = True
                        flag = True
                if not flag:
                    self.move_flag = False
                if msg.status_list[-1].status == 3 and not flag:
                    self.move_base_status = True
            except:
                pass

    def _slow_Callback(self, msg):
        v = Twist()
        v.linear.x = msg.linear.x
        v.angular.z = msg.angular.z


        if self.check_if_pose_slow():
            rospy.loginfo("减速拍照")
            v.linear.x *= 0.3
            v.angular.z *= 0.3
            if not self.camera.status:
                with self.camera.lock:
                    self.camera.status = True
                self.camera.start(self.detect_goal, 0)
                self.detect_other_image_0[self.detect_goal] = True

        self.cmd_vel_pub.publish(v)

    def check_if_pose_slow(self):
        try:
            if not self.detect_other_image_0[self.detect_goal]:
                with self._info_lock:
                    pose_x = self.odom_pose_x
                    pose_y = self.odom_pose_y
                if self.detect_goal == 'D':
                    distance = math.sqrt((pose_x - self.pos_x)**2 + (pose_y - self.pos_y)**2)
                    if distance < 1.5:
                        return True
                elif self.detect_goal == 'B':
                    distance = math.sqrt((pose_x - self.pos_x)**2 + (pose_y - self.pos_y)**2)
                    if distance < 1.5:
                        return True
                elif self.detect_goal == 'C':
                    distance = math.sqrt((pose_x - self.pos_x)**2 + (pose_y - self.pos_y)**2)
                    if distance < 1.3:
                        return True
                elif self.detect_goal == 'E':
                    distance = math.sqrt((pose_x - 2.829)**2 + (pose_y - (-1.177))**2)
                    if distance < 1.2:
                        return True

        except Exception as e:
            pass

        return False

    def init_pose(self):
        """
        矫正位姿
        :return:
        """
        rospy.loginfo("开始矫正位姿~~")

    def get_init_pose(self):
        """
        获取初始位置
        :return:
        """
        with self._info_lock:
            current_x = self.odom_pose_x
            current_y = self.odom_pose_y
        rospy.loginfo("current x : {}".format(current_x))
        rospy.loginfo("current y : {}".format(current_y))

    def get_pose(self, id):
        """
        从文件中读取中间点位置,
        点位置的命名格式为Pose_id,
        只需要输入字符串id即可,
        eg. id = 'A1'
        """
        # self.detect_goal = id if id != "Final" else 'E'
        self.detect_goal = id if id != "F1" else 'E'
        # if id == "D":
        #     self.change_model_pub.publish(String())
        if id == "F1":
            self.change_model_pub.publish(String())
        # if not self.detect_F_flag and (id[0] != 'M' and id[0] != 'F' ):
        #     self.detect_F_flag = True
        #     self.detect_lock = True
        with open("./pose/pose_{}.json".format(id), "r") as f:
            text = json.loads(f.read())
            self.pos_x = text["position"]["x"]
            self.pos_y = text["position"]["y"]
            self.pos_z = text["position"]["z"]
            self.ori_x = text["orientation"]["x"]
            self.ori_y = text["orientation"]["y"]
            self.ori_z = text["orientation"]["z"]
            self.ori_w = text["orientation"]["w"]

    def goal_pose(self):
        """
        构造 goal
        """
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = self.pos_x
        self.goal.target_pose.pose.position.y = self.pos_y
        self.goal.target_pose.pose.position.z = self.pos_z
        self.goal.target_pose.pose.orientation.x = self.ori_x
        self.goal.target_pose.pose.orientation.y = self.ori_y
        self.goal.target_pose.pose.orientation.z = self.ori_z
        self.goal.target_pose.pose.orientation.w = self.ori_w

    def send_goal(self):
        """
        发送 goal
        """
        if self.detect_F_flag and not self.detect_lock:
            return
        self.goal_pose()
        self.client.send_goal(self.goal)
        while self.move_flag:
            rospy.sleep(0.1)
        self.move_base_status = False

    def check_if_pose_near(self, x, y, radius=0.3):
        """
        检查是否接近目标位置，并开始逃逸动作
        """
        with self._info_lock:
            current_x = self.odom_pose_x
            current_y = self.odom_pose_y
        
        distance = math.sqrt((current_x - x)**2 + (current_y - y)**2)

            # self.escape_action(target_angle)
      
        # rospy.loginfo("current x: {}, current y: {}, distance: {}, status: {}".format(current_x, current_y, distance, self.move_base_status))
        
        if distance <= radius or self.move_base_status:
            rospy.loginfo("current x: {}, current y: {}, x :{}, y: {}, distance: {}, status: {}".format(current_x, current_y, x, y, distance, self.move_base_status))
            self.client.cancel_goal()
            return True
        elif self.detect_F_flag and not self.detect_lock:
            rospy.loginfo("已检测到F的所有检测板")
            self.client.cancel_goal()
            self.detect_lock = True
            self.change_model_pub.publish(String())
            return True
        return False

    def escape_action(self,target_angle):
        """
        执行朝目标点移动的逃逸行动
        """
        # 在这里编写你的逃逸行动代码
        # 可以使用ROS消息发布器来控制机器人的移动
        # 例如，发布适当的速度消息或调用移动控制服务

        # 示例代码：根据目标角度移动机器人
        linear_velocity = 0.5  # 线速度
        angular_velocity = 0.5  # 角速度
        

        # 创建ROS消息发布器
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.sleep(1.0)  # 等待ROS消息发布器启动

        # 发布速度消息，使机器人朝着目标角度移动
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_velocity
        twist_cmd.angular.z = angular_velocity * target_angle
        
        cmd_vel_pub.publish(twist_cmd)

        # 等待一段时间，使机器人移动到逃逸距离
        rospy.sleep(duration)

        # 发布停止命令，停止机器人移动
        twist_cmd = Twist()
        cmd_vel_pub.publish(twist_cmd)

    def get_image(self):
        self.image_sub = rospy.Subscriber('/image_view/output', Image, self.image_callback)

    def rotate(self, rotation_angle, rotation_speed = 2.5):
        """
        rotation_angle 正值左转，负值右转
        """
        rotation_cmd = Twist()
        if rotation_angle < 0:
            rotation_speed = -rotation_speed
        rotation_cmd.angular.z = rotation_speed
        # 计算旋转所需的时间
        rotation_duration = abs(rotation_angle) * 1.0 / abs(rotation_speed)
        end_time = rospy.Time.now() + rospy.Duration(rotation_duration)
        while rospy.Time.now() < end_time:
        # while True:
            self.cmd_vel_pub.publish(rotation_cmd)
            rospy.sleep(0.1)  # 控制发送频率

        #  if rotation_angle < 0:
            # rotation_speed = -rotation_speed
        # # 旋转机器人
        # current_theta = abs((self.odom_ori_w * rotation_speed / abs(rotation_speed) + rotation_angle * 1.0 / 360 + 1) % 2 - 1)
        # pre_num = 1
        # while True:
        #     ori_w = self.odom_ori_w
        #     ori_z = self.odom_ori_z
        #     rospy.loginfo("{}, {}, {}, {}".format(current_theta, ori_w, ori_z, abs(abs(ori_w) - current_theta)))
        #     # if pre_num < abs(abs(ori_w ) - current_theta):
        #         # break
        #     pre_num = abs(abs(ori_w) - current_theta)
        #     self.cmd_vel_pub.publish(rotation_cmd)
        #     rospy.sleep(0.1)

        # 停止机器人
        rotation_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(rotation_cmd)
       
    def rotate_and_capture(self, rotation_angle, picture_num, goal,rotation_speed = 3):
        try:
            with self.detect_other_lock:
                flag = self.detect_other_flag[goal]
            if flag:
                return
        except Exception as e:
            pass
        while self.camera.status:
            rospy.sleep(0.1)
        self.rotate(rotation_angle)
        with self.camera.lock:
            self.camera.status = True
        try:
            with self.detect_other_lock:
                flag = self.detect_other_flag[goal]
            if flag:
                self.camera.status = False
                return  True
            rospy.loginfo("{}{}点:{}".format(goal, picture_num, flag))
        except Exception as e:
            # print(e)
            pass
        self.camera.start(goal,picture_num)
        return False

    def mission(self,goal=None):
        """
        使机器人在指定点旋转一圈，并在旋转后捕捉图像。

        :param rotation_duration: 旋转持续时间（以秒为单位，默认为8秒）
        :param rotation_speed: 机器人旋转的速度（默认为0.785）
        """
        
        rospy.loginfo("[+] ====== 开始拍摄图片 =====")
        picture_num = 1
        if goal == "B":
            flag = self.rotate_and_capture(0,picture_num,goal)
            if flag:
                self.rotate(-4.047)
                return
            picture_num += 1
            self.rotate_and_capture(-1.547,picture_num,goal)
            while self.camera.status:
                rospy.sleep(0.1)
            self.rotate(-2.547)
        elif goal == "C":
            flag = self.rotate_and_capture(0,picture_num,goal)
            if flag:
                self.rotate(3.847)
            picture_num += 1
            self.rotate_and_capture(1.047,picture_num,goal)
            while self.camera.status:
                rospy.sleep(0.1)
            self.rotate(2.847)
        elif goal == "D":
            self.rotate_and_capture(0,picture_num,goal)
            while self.camera.status:
                rospy.sleep(0.1)
            self.rotate(3.54)
        elif goal == "F1":
            if not self.detect_F_flag:
                self.rotate_and_capture(0,picture_num,goal)

            picture_num += 1
            self.rotate_and_capture(2.74,picture_num,goal)
            
            picture_num += 1
            self.rotate_and_capture(1.94,picture_num,goal)

            while not self.fisrt_F:
                rospy.sleep(0.01)

            if not self.detect_F_flag:
                while self.camera.status:
                    rospy.sleep(0.1)
                self.rotate(2.38)
        elif goal == "F2":
            if self.detect_F_flag:
                return
            self.rotate_and_capture(-1.34,picture_num,goal)

            if self.detect_F_flag:
                return
            picture_num += 1
            self.rotate_and_capture(2.58,picture_num,goal)
    
    def arrive_goal(self):
        self.odom_subscriber.unregister()
        self.move_base_sub.unregister()
        self.end_pub.publish(String())
        self.lidar_stop.run()
        # self.sound.start_pub()
        while not self.detect_end_status:
            rospy.sleep(0.1)

        thread1 = threading.Thread(target = self.sound.play, args=(self.goal_dict, self.classes_dict))
        thread1.start()
        thread1.join()
        # self.sound.play(self.goal_dict, self.classes_dict)
    
