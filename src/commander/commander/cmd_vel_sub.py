import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import serial.tools.list_ports

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.open_port()
        # 创建一个订阅者来接收 cmd_vel 消息
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)
        
    
    # 串口打开函数
    def open_port(self):
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
        except Exception as exc:
            print("串口打开异常", exc)

    def twist_callback(self, msg):
        # 在这里处理接收到的 cmd_vel 消息
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        print(linear_x, angular_z)

        if linear_x > 0.0:
            linear_x = 3
        elif linear_x < 0.0:
            linear_x = 1
        elif linear_x == 0.0:
            linear_x = 2

        if angular_z > 0.0:
            angular_z = 3
        elif angular_z < 0.0:
            angular_z = 1
        elif angular_z == 0.0:
            angular_z = 2

        self.sendTwist(linear_x, angular_z)

        # 打印接收到的线速度和角速度
        # self.get_logger().info('Linear Velocity: %f, Angular Velocity: %f', linear_x, angular_z)
        print(linear_x, angular_z)

         #发送角度
    def sendTwist(self, linear_x, angular_z):#目前采用的弧度制
        linear_x = int(linear_x)
        angular_z = int(angular_z)
        #存放高八位和低八位数据的字符数组
        data =[0XFF,0,0,0,0,0,0,0XFE]

        #第一组数据的高八位和低八位
        data[1]=(linear_x)#取高八位,将高八位的数据对应的字符发出去
        data[2]=(angular_z)#取低八位
        
        # #第二组数据的高八位和低八位
        # data[3]=(angular_z>>8)
        # data[4]=(angular_z & 0x00ff)

        print(data)


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

class SerialPort:

    # 串口打开函数
    def open_port(self):
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
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


    #发送角度
    def sendTwist(self, linear_x, angular_z):#目前采用的弧度制
        linear_x = int(linear_x)
        angular_z = int(angular_z)
        #存放高八位和低八位数据的字符数组
        data =[0XFF,0,0,0,0,0XFE]

        #第一组数据的高八位和低八位
        data[1]=(linear_x)#取高八位,将高八位的数据对应的字符发出去
        data[2]=(angular_z)#取低八位
        
        # #第二组数据的高八位和低八位
        # data[3]=(angular_z>>8)
        # data[4]=(angular_z & 0x00ff)

        print(data)


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

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()

    port=SerialPort()
    # port.open_port()
    
    rclpy.spin(node)
    rclpy.shutdown()

    port.close_port()

if __name__ == '__main__':
    main()

