import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg

class IMUToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')
        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',  # 请用你的IMU话题名称替换此处
            self.imu_callback,
            10)
        self.subscription  # 防止未使用变量警告
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_time = None
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 0.0])

    def imu_callback(self, msg):
        current_time = msg.header.stamp
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) / 1e9

        # 这里假设IMU的线加速度以及角速度信息存储在msg中的对应字段中
        linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # 根据IMU数据计算位移增量
        self.position += dt * (linear_acceleration - np.array([0.0, 0.0, 9.81]))

        # 将角速度转换为四元数变化
        quaternion_change = np.array([0.0, angular_velocity[0] * dt / 2, angular_velocity[1] * dt / 2, angular_velocity[2] * dt / 2])
        self.orientation = self._quaternion_multiply(self.orientation, quaternion_change)
        self.orientation /= np.linalg.norm(self.orientation)

        # 发布里程计数据
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        self.odom_publisher.publish(odom_msg)

        # 发布tf变换
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = current_time
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.position[0]
        tf_msg.transform.translation.y = self.position[1]
        tf_msg.transform.translation.z = self.position[2]
        tf_msg.transform.rotation.x = self.orientation[0]
        tf_msg.transform.rotation.y = self.orientation[1]
        tf_msg.transform.rotation.z = self.orientation[2]
        tf_msg.transform.rotation.w = self.orientation[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        self.last_time = current_time

        self.get_logger().info('%s' % odom_msg.pose.pose.position)

    def _quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

def main(args=None):
    rclpy.init(args=args)
    imu_to_odom = IMUToOdom()
    rclpy.spin(imu_to_odom)
    imu_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
