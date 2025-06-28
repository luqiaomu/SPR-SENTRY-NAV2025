import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
def publish_2d_pose_estimate(node):
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # 创建PoseWithCovarianceStamped消息
    pose_estimate_msg = PoseWithCovarianceStamped()
    pose_estimate_msg.header.frame_id = 'map'
    pose_estimate_msg.pose.pose.position.x = 4.0  # 替换为机器人的X坐标
    pose_estimate_msg.pose.pose.position.y = 0.0  # 替换为机器人的Y坐标
    pose_estimate_msg.pose.pose.orientation.w = 0.0  # 替换为机器人的朝向

    # 发布消息
    for i in range(5):
        publisher.publish(pose_estimate_msg)
        node.get_logger().info('Published 2D Pose Estimate')
        time.sleep(0.2)

    

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pose_estimation_publisher')

    # 发布2D Pose Estimate
    publish_2d_pose_estimate(node)

    rclpy.spin(node)

    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
