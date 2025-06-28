import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import open3d as o3d
import numpy as np
import struct
from scipy.spatial.transform import Rotation as R

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
         # Create a parameter for the source coordinate system name
        self.declare_parameter('source_frame', 'livox_frame')             
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

        self.declare_parameter('pcd_map', "/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd")   
        self.pcd_map = self.get_parameter(                     
            'pcd_map').get_parameter_value().string_value
    
    
        self.map = o3d.io.read_point_cloud(self.pcd_map)
        self.voxel_size = 0.05
        self.downmap = self.map.voxel_down_sample(self.voxel_size)

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

        self.icp_start = False

    def cloud_callback(self, msg):
        if self.icp_start:
            # 解析点云消息
            points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:,:3]
            self.get_logger().info("Received cloud")
            self.current_cloud = o3d.geometry.PointCloud()
            self.current_cloud.points = o3d.utility.Vector3dVector(points)

            self.get_logger().info("Icp start")
            self.icp(self.current_cloud, self.downmap)

    def icp(self, source, target):
        self.trans_init = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)
        # 下采样和法线计算
        source = source.voxel_down_sample(self.voxel_size)
        # target = target.voxel_down_sample(0.1)
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # 2. 点云配准
        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, 0.2, self.trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
        )
        
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

        for point in source.points:
            [_, index, _] = kdtree2.search_knn_vector_3d(point, 1)
            closet_point = target.points[index[0]]
            distance = np.linalg.norm(np.array(point) - np.array(closet_point))
            if distance < distance_threshhold:
                overlap_points_count += 1

        overlap_ratio = overlap_points_count / len(source.points) * 100
        self.get_logger().info(f"Overlap ratio:{overlap_ratio}%")

        # self.init_done = True

        if overlap_ratio > self.overlap_threshold:
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

        self.init_x = pos.x
        self.init_y = pos.y
        self.init_yaw = euler[2]

     # Publish the initial position of the robot
    def publish_2d_pose_estimate(self):
        # if self.init_done or not self.init_location:
            # Create a PoseWithCovarianceStamped message
            pose_estimate_msg = PoseWithCovarianceStamped()
            pose_estimate_msg.header.frame_id = 'map'

        
            ########################################################################################
            pose_estimate_msg.pose.pose.position.x = self.init_x
            pose_estimate_msg.pose.pose.position.y = self.init_y
            # pose_estimate_msg.pose.pose.orientation.w = 1.0  
            
            pose_estimate_msg.pose.pose.orientation = self.create_quaternion_from_yaw(self.init_yaw)
            ########################################################################################

            self.initial_publisher.publish(pose_estimate_msg)
            self.get_logger().info('Published 2D Pose Estimate')

    # Create quaternion from yaw
    def create_quaternion_from_yaw(self, yaw):
        quaternion = Quaternion()
        quaternion.z = float(math.sin(yaw / 2))
        quaternion.w = float(math.cos(yaw / 2))
        return quaternion