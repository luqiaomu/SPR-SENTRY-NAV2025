import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String, Int16, Int32
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import serial
import serial.tools.list_ports
import time
import math
import tf_transformations
from tf2_ros import TransformException, Buffer, TransformListener, TransformBroadcaster
import struct


class ControllerSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # 创建一个订阅者来接收 cmd_vel 消息    cmd_vel   rm_cmd_vel   cmd_vel_decomposed
        
        self.subscription = self.create_subscription(Twist, 'cmd_vel_decomposed', self.twist_callback, 10)
        self.events_publisher = self.create_publisher(String, 'events', 10)

        self.cloud_point = None
      
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
        self.timer_serial_read = self.create_timer(0.08, self.msg_read)

        # 创建参数
        self.declare_parameter('source_frame', 'base_link')
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # 创建 TF 缓存和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        self.status = 0
        
        self.current_cloud = o3d.geometry.PointCloud()

        self.trans_init = np.eye(4)

        self.init_done = False
        self.overlap_threshold = 80
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.reach_polygon = 0
        self.send_goal = 0

        # 创建一个发布者来发布关节状态数据
        self.joint_state_publisher = self.create_publisher(JointState, 'serial/gimbal_joint_state', 10)

        # 创建一个定时器来定期发布动态 TF
        self.timer_tf_broadcast = self.create_timer(0.08, self.publish_dynamic_tf)

    def reach_polygon_callback(self, msg):
        # 在这里处理接收到的消息
        self.get_logger().info(f'Received message on topic /reach_polygon: {msg.data}')
        self.reach_polygon = msg.data

    def status_callback(self, msg):
        self.status = msg.data

    def send_goal_callback(self, msg):
        self.send_goal = msg.data

    def msg_read(self):
        try:
            # 读取串口数据
            self.ser.flushInput()
            data = self.ser.read(7)
            if data[0] == 0xff and data[6] == 0xfe:
                # # 解析 yaw 数据
                # yaw_data = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
                # self.yaw = (float(yaw_data) / 100000000 )-10 # 转换为弧度


    # # 提取符号位（假设符号位是 data[2] 的最高位）
    #             sign_bit = (data[2] >> 7) & 1  # 1 表示负数，0 表示正数
                
    #             # 提取数值部分（低7位）
    #             yaw_data_unsigned = ((data[2] & 0x7F) << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
                
    #             # 根据符号位判断正负
    #             if sign_bit:
    #                 yaw_data_signed = -yaw_data_unsigned
    #             else:
    #                 yaw_data_signed = yaw_data_unsigned
                
    #             # 转换为弧度
    #             self.yaw = float(yaw_data_signed) / 100000000  # 转换为弧度
    
                yaw_data = struct.unpack('>i', data[2:6])[0]
                self.yaw = float(yaw_data) / 10000000.0




                # 发布关节状态数据
                self.publish_joint_state()
                print('gimbal: ', self.yaw)
                # print(f'send : {self.yaw}')
                msg = String()
                msg.data = str(data[1])
                self.events_publisher.publish(msg)
                print(f'send goal: {data[1]}')
        except Exception as exc:
            print("receive error", exc)

    def publish_joint_state(self):
    #     # 创建一个 JointState 消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['gimbal_joint']  # 云台关节名称
        # joint_state.position = [self.yaw]  # 云台关节的 yaw 角度
        joint_state.position = [self.yaw]  # 云台关节的 yaw 角度
        joint_state.velocity = []  # 速度（可选）
        joint_state.effort = [0.0]  # 力矩（可选）

        # 发布关节状态数据
        self.joint_state_publisher.publish(joint_state)

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        print(linear_x, linear_y)
        if linear_x == 0.0:
            linear_x = 40
        elif linear_x < 0.1 and linear_x > 0.0:
            linear_x = 41
        elif linear_x > -0.1 and linear_x < 0.0:
            linear_x = 39
        else:
            linear_x = int((linear_x + 4) * 10)
        
        if linear_y == 0.0:
            linear_y = 40
        elif linear_y < 0.1 and linear_y > 0.0:
            linear_y = 41
        elif linear_y > -0.1 and linear_y < 0.0:
            linear_y = 39
        else:
            linear_y = int((linear_y + 4) * 10)

        self.sendTwist(linear_x, linear_y)
        print(linear_x, linear_y)

    def open_port(self):
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
                self.port_clean()
            else:
                self.ser.open()
        except Exception as exc:
            print("串口打开异常", exc)

    def port_clean(self):
        self.ser.flushInput()#清理缓冲区数据
        
    def close_port(self):
        try:
            self.ser.close()
            if self.ser.isOpen():
                print("串口未关闭")
            else:
                print("串口已关闭")
        except Exception as exc:
            print("串口关闭异常", exc)

    def sendTwist(self, linear_x, linear_y):
        linear_x = int(linear_x)
        linear_y = int(linear_y)
       
        data =[0XFF,0,0,0,0,2,0,0XFE]

        
        data[1]=linear_x      #x
        data[2]=linear_y      #y
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

    def publish_dynamic_tf(self):
        # 创建一个 TransformStamped 消息
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'odom'
        transform_msg.child_frame_id = 'gimbal_link'
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = self.z
        transform_msg.transform.rotation = Quaternion(x=0.0, y=0.0, z=math.sin(self.yaw/2), w=math.cos(self.yaw/2))
        
        # 发布 TransformStamped 消息
        self.tf_broadcaster.sendTransform(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
