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
        

        self.cloud_point = None
        self.map = None
        
        # self.pc_visualizer = o3d.visualization.Visualizer()
        # self.pc_visualizer.create_window()

        self.i = 0

        self.start_pose = None
        self.position = None

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
        # elif angular_z < 0.0:
        #     angular_z = 1
        # elif angular_z == 0.0:
        #     angular_z = 2

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
