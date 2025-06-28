import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Bool
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from interface.msg import EventTime
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import threading

import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

class Commander(Node):
    def __init__(self, name):      
        super().__init__(name)
        # Create a node to publish navigation poses when using action
        self.node = rclpy.create_node('navigation_client')
        self.time_subscription = self.create_subscription(Float32, 'duration', self.time_callback, 10)
        self.events_subscription = self.create_subscription(String, 'events', self.events_callback, 10)
        self.specific_time_subscription = self.create_subscription(EventTime, 'specific_event_duration', self.specific_time_callback, 10)
        self.dynamic_pose_subscription = self.create_subscription(Float32MultiArray, 'dynamic_pose', self.dynamic_pose_callback, 10)
        self.cancel_behavior_subscription = self.create_subscription(Bool, 'cancel_behavior', self.cancel_behavior_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry,'odom', self.odom_callback, 10)
        # Create a publisher to publish single pose to Navigation2
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        # Create a publisher to publish 2D pose estimate to Navigation2
        self.initial_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.task_publisher = self.create_publisher(String, '/tasks', 10)

        self.robot_status_publisher = self.create_publisher(Int32, '/robot_status', 10)

        self.send_goal_publisher = self.create_publisher(Int32, '/send_goal', 10)

        self.send_goal = 0

        # Time and event perception loop
        self.timer_spin_once = self.create_timer(0.1, self.Spin_Once)
        # Loop robot status,send pose
        self.timer_status_spin = self.create_timer(0.1, self.Status_Spin)

        # self.timer_check_exception = self.create_timer(0.1, self.Check_Exception_Spin)
        self.move_exception_start_time = 0
        self.move_exception_timer_is_start = False
        # Robot status, check if there is a robot connected
        self.robot_status = False
        # Start Time
        self.duration_time = 0.0
        # Received event
        self.currunt_event = None

        self.start_position = [0.0, 0.0]
        # All scheduled tasks in the move strategy configuration file
        self.duration_config_poses = {}

        self.dynamic_pose_list = []

        self.specific_duration_config_poses = {}
        self.specific_time_dict = {}

        # All event tasks in the move strategy configuration file
        self.events_config_poses = {}     
        self.last_event = None
        # Completed tasks
        self.sent_pose_keys = []
        # Robot status information director
        self.current_robot_status = {'Is_move':False, 'Strategy':[], 'Start_time':0}
        self.pause_move = False
        self.strategy_cache = []

        self.position = None
        self.robot_yaw = 0.0
        # Odometry information
        self.position_odom = None
        self.orientation = None
        self.linear_velocity = None
        self.angular_velocity = None
        # Robot radius, used to determine whether the target pose has been reached
        self.robot_radius = 0.3
        # Index when using topic to publish coordinate points
        self.send_index = 0
        # Current cruising frequency
        self.cruise_frequency = 1
        # self.publish_2d_pose_estimate()
        # self.send_goal_thread = threading.Thread(target=self.callback)
        # self.send_goal_thread.start()
        # self.i = 0

        # Create a parameter for the source coordinate system name
        self.declare_parameter('source_frame', 'base_link')             
        self.source_frame = self.get_parameter(                     
            'source_frame').get_parameter_value().string_value
        # Create a parameter for the target coordinate system name
        self.declare_parameter('target_frame', 'map')             
        self.target_frame = self.get_parameter(                    
            'target_frame').get_parameter_value().string_value
        
        self.declare_parameter('init_x', 0.0)             
        self.init_x = self.get_parameter(                    
            'init_x').get_parameter_value().double_value
        
        self.declare_parameter('init_y', 0.0)             
        self.init_y = self.get_parameter(                    
            'init_y').get_parameter_value().double_value
        
        self.declare_parameter('init_yaw', 0.0)             
        self.init_yaw = self.get_parameter(                    
            'init_yaw').get_parameter_value().double_value

        # Create a buffer to save coordinate transformation information
        self.tf_buffer = Buffer()         
        # Create a listener for coordinate transformation
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        # Create a fixed cycle timer to process coordinate information
        self.tf_timer = self.create_timer(0.1, self.tf_timer)    

        # Load move strategy configuration file
        self.declare_parameter('move_strategy_config', '/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/commander/commander/config_r.yaml')   
        self.move_strategy_config = self.get_parameter(                     
            'move_strategy_config').get_parameter_value().string_value
        self.load_config(self.move_strategy_config)    

        self.declare_parameter('pcd_map', "/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/test.pcd")   
        self.pcd_map = self.get_parameter(                     
            'pcd_map').get_parameter_value().string_value
        
        self.declare_parameter('init_location', True)   
        self.init_location = self.get_parameter(                     
            'init_location').get_parameter_value().bool_value
        

        self.map = o3d.io.read_point_cloud(self.pcd_map)
        self.voxel_size = 1
        # self.downmap = self.map.voxel_down_sample(0.1)
        self.downmap = self.map.uniform_down_sample(every_k_points=50)
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar/pointcloud',  # 替换为实际的话题名称
            self.cloud_callback,
            10)
        
        self.current_cloud = o3d.geometry.PointCloud()

        # self.pc_visualizer = o3d.visualization.Visualizer()
        # self.pc_visualizer.create_window()

        self.trans_init = np.eye(4)

        self.init_done = False
        self.overlap_threshold = 80

    # Time and event perception loop
    def Spin_Once(self):
        self.check_time(self.duration_time)
        self.check_event(self.currunt_event)
        self.check_specific_time(self.specific_time_dict)
        self.check_dynamic_pose(self.dynamic_pose_list)
        # self.chech_strategy_cache()

    def chech_strategy_cache(self):
        if len(self.strategy_cache) != 0:
            if not self.current_robot_status['Is_move']:
                min_level = self.strategy_cache[0][2][0]
                index = 0
                for i in range(self.strategy_cache):
                    if self.strategy_cache[i][2][0] <= min_level:
                        min_level = self.strategy_cache[i][2][0]
                        index = i
                self.current_robot_status['Is_move'] = True
                self.current_robot_status['Strategy'] = self.strategy_cache[index]
                self.get_logger().info('The pose %s will be sent' % self.strategy_cache[index][0])    

    # Loop robot status,send pose
    def Status_Spin(self):
        if not self.robot_status:
            self.publish_2d_pose_estimate()
        if self.current_robot_status['Is_move'] and self.robot_status:
            to_move = False
            if 'delay' in self.current_robot_status['Strategy']:
                if self.duration_time - self.current_robot_status['Start_time'] > self.current_robot_status['Strategy']['delay']:
                    to_move = True
                else:
                    to_move = False
            else:
                to_move = True
            if to_move:
                self.move()


    def move(self):
        if not self.pause_move:
            # Single point navigation mode
            if self.current_robot_status['Strategy']['mode'][0] == 'singel-pose':
                # Navigate to pose
                if type(self.current_robot_status['Strategy']['pose']) == type(PoseStamped()):
                    self.send_goal_topic(self.current_robot_status['Strategy']['pose'])

                    msg = Int32()
                    value = 1
                    msg.data = value
                    self.robot_status_publisher.publish(msg)

                    msg = Int32()
                    value = self.send_goal
                    msg.data = value
                    self.send_goal_publisher.publish(msg)

                    euler_1 = tf_transformations.euler_from_quaternion([self.current_robot_status['Strategy']['pose'].pose.orientation.x, 
                                                                        self.current_robot_status['Strategy']['pose'].pose.orientation.y, 
                                                                        self.current_robot_status['Strategy']['pose'].pose.orientation.z, 
                                                                        self.current_robot_status['Strategy']['pose'].pose.orientation.w])

                    if math.sqrt((self.position.x - self.current_robot_status['Strategy']['pose'].pose.position.x) ** 2 + 
                                 (self.position.y - self.current_robot_status['Strategy']['pose'].pose.position.y) ** 2) <= self.robot_radius and math.fabs(euler_1[2] - self.robot_yaw) < 0.65:
                        if 'task' in self.current_robot_status['Strategy']:
                            msg = String()
                            msg.data = 'spin'
                            self.task_publisher.publish(msg)
                        self.current_robot_status['Is_move'] = False
                        self.current_robot_status['Strategy'] = {}
                        self.current_robot_status['Start_time'] = 0
                        self.get_logger().info('Goal sent successfully')

                        msg = Int32()
                        value = 2
                        msg.data = value
                        self.robot_status_publisher.publish(msg)
    
                        
                # Navigate through poses
                elif type(self.current_robot_status['Strategy']['pose']) == type(list()):
                    self.send_goal_topic(self.current_robot_status['Strategy']['pose'][self.send_index])

                    msg = Int32()
                    value = 1
                    msg.data = value
                    self.robot_status_publisher.publish(msg)

                    msg = Int32()
                    value = self.send_goal
                    msg.data = value
                    self.send_goal_publisher.publish(msg)

                    if self.send_index == len(self.current_robot_status['Strategy']['pose']):
                        tolerance = self.robot_radius
                    else:
                        tolerance = self.robot_radius * 1.5

                    euler_2 = tf_transformations.euler_from_quaternion([self.current_robot_status['Strategy']['pose'][self.send_index].pose.orientation.x, 
                                                                            self.current_robot_status['Strategy']['pose'][self.send_index].pose.orientation.y, 
                                                                            self.current_robot_status['Strategy']['pose'][self.send_index].pose.orientation.z, 
                                                                            self.current_robot_status['Strategy']['pose'][self.send_index].pose.orientation.w])

                    if math.sqrt((self.position.x - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.x) ** 2 + 
                                 (self.position.y - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.y) ** 2) <= tolerance :
                    # and math.fabs(euler_2[2] - self.robot_yaw) < 0.65:
                        self.send_index += 1
                        
                        if self.send_index == len(self.current_robot_status['Strategy']['pose']):
                            if 'task' in self.current_robot_status['Strategy']:
                                msg = String()
                                msg.data = 'spin'
                                self.task_publisher.publish(msg)
                            self.current_robot_status['Is_move'] = False
                            self.current_robot_status['Strategy'] = {}
                            self.current_robot_status['Start_time'] = 0
                            self.get_logger().info('Goal sent successfully')
                            self.send_index = 0

                            msg = Int32()
                            value = 2
                            msg.data = value
                            self.robot_status_publisher.publish(msg)

            # Automatic cruise control mode
            elif self.current_robot_status['Strategy']['mode'][0] == 'circle_cruise' or self.current_robot_status['Strategy']['mode'][0] == 'return_cruise':
                if type(self.current_robot_status['Strategy']['pose']) == type(list()):
                    self.send_goal_topic(self.current_robot_status['Strategy']['pose'][self.send_index])

                    msg = Int32()
                    value = 1
                    msg.data = value
                    self.robot_status_publisher.publish(msg)

                    if math.sqrt((self.position.x - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.x) ** 2 + (self.position.y - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.y) ** 2) <= self.robot_radius * 1.5:
                        if self.current_robot_status['Strategy']['mode'][0] == 'return_cruise':
                            if self.cruise_frequency % 2 == 1:
                                self.send_index += 1
                            else:
                                self.send_index -= 1

                            # Reverse traverse pose list
                            if self.send_index == len(self.current_robot_status['Strategy']['pose']):
                                self.cruise_frequency += 1
                                self.send_index -= 1
                            elif self.send_index == -1:
                                self.cruise_frequency += 1
                                self.send_index += 1
                        elif self.current_robot_status['Strategy']['mode'][0] == 'circle_cruise':
                            self.send_index += 1

                            if self.send_index == len(self.current_robot_status['Strategy']['pose']):
                                self.cruise_frequency += 1
                                self.send_index = 0
                            
                        # Complete cruise control and reset robot status
                        if self.cruise_frequency == self.current_robot_status['Strategy']['mode'][1] + 1 and self.current_robot_status['Strategy']['mode'][1] != 0:
                            self.current_robot_status['Is_move'] = False
                            self.current_robot_status['Strategy'] = {}
                            self.current_robot_status['Start_time'] = 0
                            self.get_logger().info('Goal sent successfully')
                            self.send_index = 0
                            self.cruise_frequency = 1

                            msg = Int32()
                            value = 2
                            msg.data = value
                            self.robot_status_publisher.publish(msg)

    def task(self):
        if 'task' in self.current_robot_status['Strategy']:
            if self.send_index in self.current_robot_status['Strategy']['task']:
                if math.sqrt((self.position.x - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.x) ** 2 + (self.position.y - self.current_robot_status['Strategy']['pose'][self.send_index].pose.position.y) ** 2) <= self.robot_radius * 1.5:
                    msg = String()
                    msg.data = 'spin'
                    self.task_publisher.publish(msg)
                    self.pause_move = True

    def Check_Exception_Spin(self):
        if self.current_robot_status['Is_move'] and self.robot_status:
            if math.sqrt(self.linear_velocity.x ** 2 + self.linear_velocity.y ** 2) < 0.1:
                if not self.move_exception_timer_is_start:
                    self.move_exception_start_time = time.time()
                    self.move_exception_timer_is_start = True
                    current_duration = time.time() - self.move_exception_start_time
                    if current_duration > 5.0:
                        self.pause_move = True
            else:
                self.move_exception_timer_is_start = False
                self.move_exception_start_time = 0.0
                self.pause_move = False

    def check_time(self, duration):
        if len(self.duration_config_poses) != 0:
            for key in self.duration_config_poses.keys():
                if math.fabs(key - duration) <= 0.3:
                    whether_to_move = False
                    if not self.current_robot_status['Is_move']:
                        whether_to_move = True
                    else:
                        if self.duration_config_poses[key]['level'][0] <= self.current_robot_status['Strategy']['level'][0]:
                            whether_to_move = True
                            if self.current_robot_status['Strategy'] not in self.strategy_cache:
                                self.strategy_cache.append(self.current_robot_status['Strategy'])
                        else:
                            whether_to_move = False
                            if self.duration_config_poses[key] not in self.strategy_cache:
                                self.strategy_cache.append(self.duration_config_poses[key])

                    if whether_to_move:
                        self.current_robot_status['Is_move'] = True
                        self.current_robot_status['Strategy'] = self.duration_config_poses[key]
                        self.current_robot_status['Start_time'] = self.duration_time
                        
                        self.get_logger().info('The pose %s will be sent' % self.current_robot_status['Strategy'])


    def check_event(self, event):
        if len(self.events_config_poses) != 0:
            for key in self.events_config_poses.keys():
                if key == event:
                    whether_to_move = False
                    if key != self.last_event:
                        self.last_event = key
                        whether_to_move = True
                    # else:
                    #     if self.events_config_poses[key]['level'][0] <= self.current_robot_status['Strategy']['level'][0]:
                    #         whether_to_move = True
                    #         if self.current_robot_status['Strategy'] not in self.strategy_cache:
                    #             self.strategy_cache.append(self.current_robot_status['Strategy'])
                    #     else:
                    #         whether_to_move = False
                    #         if self.events_config_poses[key] not in self.strategy_cache:
                    #             self.strategy_cache.append(self.events_config_poses[key])

                    if whether_to_move:        
                        # if event != self.current_robot_status['Strategy']:
                            self.currunt_event = None
                            self.current_robot_status['Is_move'] = True
                            self.current_robot_status['Strategy'] = self.events_config_poses[key]
                            self.current_robot_status['Start_time'] = self.duration_time
                        
                            self.get_logger().info('The pose %s will be sent' % self.events_config_poses[key]['pose'])
                            self.send_goal = int(key)
                            

    def check_specific_time(self, event_time_dict):
        if len(self.specific_duration_config_poses) != 0:
            for event in self.specific_duration_config_poses.keys():
                if event in event_time_dict.keys():
                    for key in self.specific_duration_config_poses[event].keys():
                        if math.fabs(event_time_dict[event] - key) <= 0.3:
                            whether_to_move = False
                            if not self.current_robot_status['Is_move']:
                                whether_to_move = True
                            else:
                                if self.specific_duration_config_poses[event][key]['level'][0] <= self.current_robot_status['Strategy']['level'][0]:
                                    whether_to_move = True
                                    if self.current_robot_status['Strategy'] not in self.strategy_cache:
                                        self.strategy_cache.append(self.current_robot_status['Strategy'])
                                else:
                                    whether_to_move = False
                                    if self.specific_duration_config_poses[event][key] not in self.strategy_cache:
                                        self.strategy_cache.append(self.specific_duration_config_poses[event][key])
                            
                            if whether_to_move:
                                self.current_robot_status['Is_move'] = True
                                self.current_robot_status['Strategy'] = self.specific_duration_config_poses[event][key]
                                self.current_robot_status['Start_time'] = self.duration_time
                      
                                self.get_logger().info('The pose %s will be sent' % self.specific_duration_config_poses)

    def check_dynamic_pose(self, dynamic_pose):
        pass

    # Time perception callback function
    def time_callback(self, msg):
        # self.get_logger().info('Received time: "%s"' % msg.data)
        self.duration_time = msg.data
        
    # Event perception callback function
    def events_callback(self, msg):
        # self.get_logger().info('Received event: "%s"' % msg.data)
        self.currunt_event = msg.data

    def specific_time_callback(self, msg):
        # self.get_logger().info(f"Received specific time: {msg}")
        self.specific_time_dict[msg.specific_event] = msg.time

    def dynamic_pose_callback(self, msg):
        self.get_logger().info(f"Received dynamic pose: {msg}")
        self.dynamic_pose_list = [msg.data[0], msg.data[1], msg.data[2]]

    def cancel_behavior_callback(self, msg):
        if msg.data:
            self.cancel_navigation_goal()

    def cloud_callback(self, msg):
        if not self.init_done and self.init_location:
            # 解析点云消息
            points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:,:3]
            self.get_logger().info("Received cloud")
            self.current_cloud = o3d.geometry.PointCloud()
            self.current_cloud.points = o3d.utility.Vector3dVector(points)

            
            self.icp(self.current_cloud, self.downmap)

    def icp(self, source, target):
        self.trans_init = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)
        self.get_logger().info('--------------------------------')
        
        # 下采样和法线计算
        # source = source.voxel_down_sample(0.1)
        source = source.uniform_down_sample(every_k_points=30)
        # target = target.voxel_down_sample(0.1)
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.get_logger().info(f'{source, target}')
        # 2. 点云配准
        self.get_logger().info('--------------------------------')
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, 0.2, self.trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )
        self.get_logger().info('--------------------------------')
        reg_p_trans = reg_p2p.transformation
        self.trans_init = reg_p_trans
        # 3. 打印相对位姿变化
        translation_vector = reg_p_trans[:3, 3]
        rotation_matrix = reg_p_trans[:3, :3]
        quat = R.from_matrix(rotation_matrix.copy()).as_quat()
        # 欧拉角
        theta_x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        theta_y = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        theta_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        # 弧度转度数
        theta_x_deg = np.degrees(theta_x)
        theta_y_deg = np.degrees(theta_y)
        theta_z_deg = np.degrees(theta_z)
        # print("Transformation matrix:", np.round(reg_p_trans, 3))
        # print("Rotation matrix:", np.round(rotation_matrix, 3))
        self.get_logger().info(f"Translation vector (x, y, z):{translation_vector}")
        self.get_logger().info("Euler Angle:\n\tx=%2f\n\ty=%2f\n\tz=%2f" %(theta_x_deg, theta_y_deg, theta_z_deg))
        # print("Quaternion:\n\tw=%2f\n\tx=%2f\n\ty=%2f\n\tz=%2f" %(quat[3], quat[0], quat[1], quat[2]))

        source.transform(reg_p_trans)

        kdtree1 = o3d.geometry.KDTreeFlann(source)
        kdtree2 = o3d.geometry.KDTreeFlann(target)

        distance_threshhold = self.voxel_size

        overlap_points_count = 0

        # for point in source.points:
        #     [_, index, _] = kdtree2.search_knn_vector_3d(point, 1)
        #     closet_point = target.points[index[0]]
        #     distance = np.linalg.norm(np.array(point) - np.array(closet_point))
        #     if distance < distance_threshhold:
        #         overlap_points_count += 1

        # overlap_ratio = overlap_points_count / len(source.points) * 100
        # self.get_logger().info(f"Overlap ratio:{overlap_ratio}%")
        # print("Overlap ratio:", overlap_ratio, "%")

        self.init_done = True

        # if overlap_ratio > self.overlap_threshold:
        self.init_x = translation_vector[0]
        self.init_y = translation_vector[1]
        self.init_yaw = theta_z_deg * 3.14 /180

    def euler_to_rotation_matrax(self, yaw):
        Ro = np.eye(3)
        Ro[0, 0] = math.cos(yaw)
        Ro[0, 1] = -math.sin(yaw)
        Ro[1, 0] = math.sin(yaw)
        Ro[1, 1] = math.cos(yaw)

        return Ro
    
    def transformation_matrix(self, x, y, yaw):
        Ro = self.euler_to_rotation_matrax(yaw)
        Tr = np.eye(4)
        Tr[:3, :3] = Ro
        Tr[0, 3] = x
        Tr[1, 3] = y
        return Tr

    # Odometer callback function
    def odom_callback(self, msg):
        self.position_odom = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_velocity = msg.twist.twist.linear
        self.angular_velocity = msg.twist.twist.angular
        # self.get_logger().info('Position: %s ' % self.position)

        # print("Position: [{}, {}, {}]".format(self.position.x, self.position.y, self.position.z))
        # print("Orientation: [{}, {}, {}]".format(self.orientation.x, self.orientation.y, self.orientation.z))
        # print("Linear Velocity: [{}, {}, {}]".format(self.linear_velocity.x, self.linear_velocity.y, self.linear_velocity.z))
        # print("Angular Velocity: [{}, {}, {}]".format(self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z))

    # Tf listening callback(between map and base_link)
    def tf_timer(self):
        try:
            # Obtain the current time of the ROS system
            now = rclpy.time.Time()   
            # Monitor the coordinate transformation from the source coordinate system to the target coordinate system at the current time                          
            trans = self.tf_buffer.lookup_transform(                
                self.target_frame,
                self.source_frame,
                now)
        # If coordinate transformation acquisition fails, enter an exception report
        except TransformException as ex:                            
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return
        # Obtain location information
        pos  = trans.transform.translation           
        # Obtain posture information (quaternion)
        quat = trans.transform.rotation                             
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
        #   % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))
        self.position = pos
        self.robot_yaw = euler[2]
        self.robot_status = True


    # Using topic to publish target pose
    def send_goal_topic(self, goal_pose):
        if type(goal_pose) == type(PoseStamped()):
            self.goal_pose_publisher.publish(goal_pose)

    # Using action to publish target pose
    def send_goal_action(self, goal_pose):
        if type(goal_pose) == type(PoseStamped()):
            self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for NavigateToPose action server...')
            self.action_client.wait_for_server()
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
        elif type(goal_pose) == type(list()):
            self.action_client = ActionClient(self.node, NavigateThroughPoses, '/navigate_through_poses')

            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for NavigateThroughPoses action server...')
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = goal_pose

        self.get_logger().info('Sending navigation goal...')
        self.action_client.wait_for_server()
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        time.sleep(1)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        if send_goal_future.result() is not None:
            self.get_logger().info('Goal sent successfully')
            goal_handle = send_goal_future.result()

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future)
            result = get_result_future.result()
            self.get_logger().info(f'Navigation result: {result.result}')
        else:
            self.get_logger().error('Failed to send navigation goal')

        self.action_client.destroy()

    # Create a single navigation point
    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = self.create_quaternion_from_yaw(float(yaw))
        return pose

    # Create quaternion from yaw
    def create_quaternion_from_yaw(self, yaw):
        quaternion = Quaternion()
        quaternion.z = float(math.sin(yaw / 2))
        quaternion.w = float(math.cos(yaw / 2))
        return quaternion
    
    # Cancel current move
    def cancel_navigation_goal(self):
        action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

        while not action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for NavigateToPose action server...')

        # Create cancellation request
        cancel_goal_msg = NavigateToPose.Goal()

        # Set the header for cancellation requests (usually requires setting a valid header)
        cancel_goal_msg.pose.header = Header()

        # Send cancellation request
        self.get_logger().info('Cancelling navigation goal...')
        cancel_goal_future = action_client.send_goal_async(cancel_goal_msg)
        rclpy.spin_until_future_complete(self.node, cancel_goal_future)

        if cancel_goal_future.result() is not None:
            self.get_logger().info('Navigation goal cancelled successfully')
            self.pause_move = True
        else:
            self.get_logger().error('Failed to cancel navigation goal')

        action_client.destroy()



    # Publish the initial position of the robot
    def publish_2d_pose_estimate(self):
        if self.init_done or not self.init_location:
            # Create a PoseWithCovarianceStamped message
            pose_estimate_msg = PoseWithCovarianceStamped()
            pose_estimate_msg.header.frame_id = 'map'

        
            ########################################################################################
            pose_estimate_msg.pose.pose.position.x = self.init_x
            pose_estimate_msg.pose.pose.position.y = self.init_y
            # pose_estimate_msg.pose.pose.orientation.w = 1.0  
            
            pose_estimate_msg.pose.pose.orientation = self.create_quaternion_from_yaw(self.init_yaw)
            ########################################################################################

            # self.initial_publisher.publish(pose_estimate_msg)
            self.get_logger().info('Published 2D Pose Estimate')


    def check_config(self, config_dict):
        format_error_list = []
        keys_to_check = ['pose', 'move', 'level']
        for key in config_dict.keys():
            if all(key in config_dict for key in keys_to_check):
                if key not in format_error_list:
                    format_error_list.append(key)
            else:
                # A two-dimensional list representing waypoints
                if type(config_dict[key]['pose']) == list and all([isinstance(item, list) for item in config_dict[key]['pose']]):
                    for temp_pose in config_dict[key]['pose']:
                        if len(temp_pose) != 3:
                            if key not in format_error_list:
                                format_error_list.append(key)
                        else:
                            for i in temp_pose:
                                if type(i) == type(int()) or type(i) == type(float()):
                                    pass
                                else:
                                    if key not in format_error_list:
                                        format_error_list.append(key)
                else:
                    if len(config_dict[key]['pose']) != 3:
                        if key not in format_error_list:
                            format_error_list.append(key)
                    else:
                        for i in config_dict[key]['pose']:
                            if type(i) == type(int()) or type(i) == type(float()):
                                pass
                            else:
                                if key not in format_error_list:
                                    format_error_list.append(key)

                if len(config_dict[key]['mode']) != 2:
                    if key not in format_error_list:
                        format_error_list.append(key)
                else:
                    if type(config_dict[key]['mode'][0]) == type(str()) and type(config_dict[key]['mode'][1]) == type(int()):
                        pass
                    else:
                        if key not in format_error_list:
                            format_error_list.append(key)

                if len(config_dict[key]['level']) != 2:
                    if key not in format_error_list:
                        format_error_list.append(key)
                else:
                    if type(config_dict[key]['level'][0]) == type(int()) and type(config_dict[key]['level'][1]) == type(str()):
                        pass
                    else:
                        if key not in format_error_list:
                            format_error_list.append(key)

        if len(format_error_list) == 0:
            self.get_logger().info('Configuration file check passed')
            return config_dict
        else:
            for error_key in format_error_list:
                self.get_logger().info('The "%s" config error, about to be deleted' % error_key)
                config_dict.pop(error_key)
            return config_dict

    def convert_to_navigation_pose(self, config_dict):
        for key in config_dict.keys():
            # A two-dimensional list representing waypoints
            if type(config_dict[key]['pose']) == list and all([isinstance(item, list) for item in config_dict[key]['pose']]):
                pose = []
                for temp_pose in config_dict[key]['pose']:
                    x, y, yaw = temp_pose
                    pose.append(self.create_pose_stamped(x, y, yaw))
            else: # A single-point
                x, y, yaw = config_dict[key]['pose']
                pose = self.create_pose_stamped(x, y, yaw)
            config_dict[key]['pose'] = pose
        
        return config_dict

    def load_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = f.read()
            result = yaml.load(data, Loader=yaml.FullLoader)
            if 'Duration' in result.keys():
                self.duration_config_poses = self.convert_to_navigation_pose(self.check_config(result['Duration']))
            if 'Events' in result.keys():
                self.events_config_poses = self.convert_to_navigation_pose(self.check_config(result['Events']))
            if 'SpecificDuration' in result.keys():
                for key in result['SpecificDuration']:
                    self.specific_duration_config_poses[key] = self.convert_to_navigation_pose(self.check_config(result['SpecificDuration'][key]))
              

def main(args=None):
    rclpy.init(args=args)
    node = Commander('commander')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()