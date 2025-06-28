import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def cancel_navigation_goal():
    rclpy.init()

    # 创建 ROS 节点
    node = rclpy.create_node('cancel_navigation_goal_node')

    # 创建 NavigateToPose action 客户端
    action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    # 等待 action 服务器就绪
    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Waiting for NavigateToPose action server...')

    # 创建取消请求
    cancel_goal_msg = NavigateToPose.Goal()

    # 设置取消请求的 header（通常需要设置一个有效的 header）
    cancel_goal_msg.pose.header = Header()

    # 发送取消请求
    node.get_logger().info('Cancelling navigation goal...')
    cancel_goal_future = action_client.send_goal_async(cancel_goal_msg)
    rclpy.spin_until_future_complete(node, cancel_goal_future)

    if cancel_goal_future.result() is not None:
        node.get_logger().info('Navigation goal cancelled successfully')
    else:
        node.get_logger().error('Failed to cancel navigation goal')

    rclpy.shutdown()

if __name__ == '__main__':
    cancel_navigation_goal()
