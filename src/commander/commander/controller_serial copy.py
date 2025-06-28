import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int32

import serial
import serial.tools.list_ports
import time
import math

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ControllerSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

       
        # 创建一个订阅者来接收 cmd_vel 消息
        self.subscription = self.create_subscription(Twist, 'rm_cmd_vel', self.twist_callback, 10)  #rm_cmd_vel
        self.events_publisher = self.create_publisher(String, 'events', 10)
        self.subscription_task = self.create_subscription(String, 'tasks', self.tasks_callback, 10)
        self.subscription_status = self.create_subscription(Int32, '/robot_status', self.status_callback, 10)

        self.subscription_send_goal = self.create_subscription(Int32, '/send_goal', self.send_goal_callback, 10)


        self.reach_polygon_subscriber = self.create_subscription(
            Int16,
            '/reach_polygon',
            self.reach_polygon_callback,
            10)
        # self.subscription_map = self.create_subscription(
        #     PointCloud2,
        #     '/Laser_map',  # 替换为实际的话题名称
        #     self.map_callback,
        #     10)

        self.cloud_point = None
        self.map = None
        
        # self.pc_visualizer = o3d.visualization.Visualizer()
        # self.pc_visualizer.create_window()

        self.i = 0

        self.start_pose = None
        self.position = None


        self.timer_robot_spin = self.create_timer(0.1, self.robot_spin)
        
        self.timer_serial_read = self.create_timer(0.05, self.msg_read)

        # self.serial_isopen = False

        self.open_port()

        self.is_spin = False
        self.start_time = 0
        self.race_status = 0
        self.blood = 400
        self.bullet_allowance = 200

        self.time = 0

        self.icp_start = False


        # Create a parameter for the source coordinate system name
        self.declare_parameter('source_frame', 'base_link')             
        self.source_frame = self.get_parameter(                     
            'source_frame').get_parameter_value().string_value
        # Create a parameter for the target coordinate system name
        self.declare_parameter('target_frame', 'map')             
        self.target_frame = self.get_parameter(                    
            'target_frame').get_parameter_value().string_value
        # Create a buffer to save coordinate transformation information
        self.tf_buffer = Buffer()         
        # Create a listener for coordinate transformation
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        # Create a fixed cycle timer to process coordinate information
        # self.tf_timer = self.create_timer(0.1, self.tf_timer) 

        self.status = 0  

        self.map = o3d.io.read_point_cloud('/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd')#spr-rc
        self.voxel_size = 0.05
        self.downmap = self.map.voxel_down_sample(self.voxel_size)

        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar/pointcloud_ros',  # 替换为实际的话题名称ivox/lidar/pointcloud
            self.cloud_callback,
            10)
        
        self.current_cloud = o3d.geometry.PointCloud()

        self.trans_init = np.eye(4)

        self.init_done = False
        self.overlap_threshold = 80
            
        self.init_x = 0.0
                    
        self.init_y = 0.0
               
        self.init_yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.reach_polygon = 0

        self.send_goal = 0

    def reach_polygon_callback(self, msg):
            # 在这里处理接收到的消息
            self.get_logger().info(f'Received message on topic /reach_polygon: {msg.data}')
            self.reach_polygon = msg.data
      
         

    def status_callback(self, msg):
        self.status = msg.data

    def send_goal_callback(self, msg):
        self.send_goal = msg.data


    def map_callback(self, msg):
        # 解析点云消息
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:,:3]
        self.get_logger().info("Received map")
        self.map = o3d.geometry.PointCloud()
        self.map.points = o3d.utility.Vector3dVector(points)
        # 更新点云可视化
        # self.update_pointcloud(self.points)
        

    def cloud_callback(self, msg):
        if self.icp_start:
            # 解析点云消息
            points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:,:3]
            self.get_logger().info("Received cloud")
            self.current_cloud = o3d.geometry.PointCloud()
            self.current_cloud.points = o3d.utility.Vector3dVector(points)

            print("icp start")
            self.icp(self.current_cloud, self.downmap)

    def icp(self, source, target):
        self.trans_init = self.transformation_matrix(self.x, self.y, self.yaw)
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
        print("Overlap ratio:", overlap_ratio, "%")

        self.init_done = True

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

    def robot_spin(self):
        if self.is_spin:
            current_time = time.time()
            if self.start_time != 0:
                if current_time - self.start_time  < 30:
                    print([0XFF,2,6,0,0,0,0,0XFE])
                    try:
                        send_datas = [0XFF,40,160,1,0,0,0,0XFE]#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
                        self.ser.write(send_datas)
                        print('-' * 80)
                        print("已发送数据:")
                        print(send_datas)
                        print('-' * 80)
                        # self.time += 1
                    except Exception as exc:
                        print("发送异常", exc)
                else:
                        self.is_spin = False
                        self.start_time = 0
                        
                        try:
                            send_datas = [0XFF,2,2,0,0,0,0,0XFE]#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
                            self.ser.write(send_datas)
                            print('-' * 80)
                            print("已发送数据:")
                            print(send_datas)
                            print('-' * 80)

                        except Exception as exc:
                            print("发送异常", exc)

                        self.icp_start = True
                        
                        for i in range(5):
                            msg = String()
                            msg.data = 'back'
                            self.events_publisher.publish(msg)

    def tasks_callback(self, msg):
        self.get_logger().info('Received event: "%s"' % msg.data)
        #if msg.data == 'spin':
        self.is_spin = True
        self.start_time = time.time()
        self.start_pose = self.position

        self.x = self.init_x
        self.y = self.init_y
        self.yaw = self.init_yaw

    def msg_read(self):
        # while True:
            # if self.serial_isopen:
                try:
                    # 读取串口数据
                    self.ser.flushInput()
                    data = []
                    data = self.ser.read(7)
                    
                    if data[0] == 0xff and data[6] == 0xfe:
                        msg = String()
                        msg.data = str(data[1])
                        self.events_publisher.publish(msg)
                        # self.send_goal = data[1]
                        print(f'send goal:{data[1]}')
                        

                except Exception as exc:
                    print("receive error", exc)
                
        
    def twist_callback(self, msg):
        # 在这里处理接收到的 cmd_vel 消息
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        print(linear_x, angular_z)

        # if linear_x > 0.0:
        #     linear_x = 3
        # elif linear_x < 0.0:
        #     linear_x = 1
        # elif linear_x == 0.0:
        #     linear_x = 2

        # if angular_z > 0.0:
        #     angular_z = 3
        # elif angular_z < 0.0: 00
        #     angular_z = 1
        # elif angular_z == 0.0:
        if linear_x == 0.0:
            linear_x = 40
        elif linear_x < 0.1 and linear_x > 0.0:
            linear_x = 41
        elif linear_x > -0.1 and linear_x < 0.0:
            linear_x = 39
        else:   
            linear_x = int((linear_x + 4) * 10)
            

        
        if angular_z == 0.0:
            angular_z = 40
        elif angular_z < 0.1 and angular_z > 0.0:
            angular_z = 41
        elif angular_z > -0.1 and angular_z < 0.0:
            angular_z = 39
        else:
            angular_z = int((angular_z + 4) * 10)

       

        if not self.is_spin:

            self.sendTwist(linear_x, angular_z)

            # 打印接收到的线速度和角速度
            # self.get_logger().info('Linear Velocity: %f, Angular Velocity: %f', linear_x, angular_z)
            print(linear_x, angular_z)

    # 串口打开函数
    def open_port(self):
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
                # self.serial_isopen = True
                self.port_clean()
            else:
                self.ser.open()
        except Exception as exc:
            print("串口打开异常", exc)


    def port_clean(self):
        self.ser.flushInput()#清理缓冲区数据
        
 
    # 关闭串口
    def close_port(self):
    
        try:
            self.ser.close()
            if self.ser.isOpen():
                print("串口未关闭")
            else:
                print("串口已关闭")
        except Exception as exc:
            print("串口关闭异常", exc)


    def sendTwist(self, linear_x, angular_z):
        linear_x = int(linear_x)
        angular_z = int(angular_z)
       
        data =[0XFF,0,0,0,0,2,0,0XFE]

        
        data[1]=linear_x
        data[2]=angular_z
        if self.status == 2 and data[1] != 40 and data[2] != 40:
            data[3] = 1
        else:
            data[3]=self.status
        data[4] = self.send_goal
        data[5]=2

        # print(data)


        try:
            send_datas = data#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
            # ser.write(str(send_datas+'\r\n').encode("utf-8"))
            self.ser.write(send_datas)
            print('-' * 80)
            print("已发送数据:")
            print(send_datas)
            print('-' * 80)

        except Exception as exc:
            print("发送异常", exc)


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

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
