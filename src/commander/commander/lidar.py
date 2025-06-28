import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs
import open3d as o3d

import numpy as np

class PcdPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pcd_pointcloud', 10)
        self.timer_ = self.create_timer(1.0, self.publish_pcd)
        self.pcd_file_path = "/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/write1.pcd" #/write.pcd
        self.pcd = o3d.io.read_point_cloud(self.pcd_file_path)
        self.downmap = self.pcd.uniform_down_sample(every_k_points=50)
    def publish_pcd(self):
        open3d_pc_msg = sensor_msgs.msg.PointCloud2()
        
        # # 填充消息头信息

        open3d_pc_msg.header.stamp = self.get_clock().now().to_msg()
        open3d_pc_msg.header.frame_id = "map"

        # 将 Vector3dVector 转换为 numpy 数组
        pc_points = np.asarray(self.downmap.points)
        
        # 定义字段 初始赋值
        open3d_pc_msg.height = 1
        open3d_pc_msg.width = len(pc_points)
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.is_bigendian = False
        open3d_pc_msg.point_step = 12
        open3d_pc_msg.row_step = open3d_pc_msg.point_step * len(pc_points)
        open3d_pc_msg.is_dense = False
        
        # 将点云数据转换为二进制格式并设置到消息中
        open3d_pc_msg.data = pc_points.astype(np.float32).tobytes()
        
        # # 发布点云消息
        self.publisher_.publish(open3d_pc_msg)
        self.get_logger().info("publish")

def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PcdPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()