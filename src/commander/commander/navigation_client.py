import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient:
    def __init__(self):
        self.node = rclpy.create_node('navigation_client')
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for NavigateToPose action server...')

    def send_goal(self, goal_pose):
        print(type(goal_pose))
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.node.get_logger().info('Sending navigation goal...')
        self.action_client.wait_for_server()
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        if send_goal_future.result() is not None:
            self.node.get_logger().info('Goal sent successfully')
            goal_handle = send_goal_future.result()

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future)
            result = get_result_future.result()
            self.node.get_logger().info(f'Navigation result: {result.result}')
        else:
            self.node.get_logger().error('Failed to send navigation goal')

def create_pose_stamped(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = create_quaternion_from_yaw(yaw)
    return pose

def create_quaternion_from_yaw(yaw):
    # Create a quaternion from yaw angle (in radians)
    from geometry_msgs.msg import Quaternion
    import math

    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2)
    quaternion.w = math.cos(yaw / 2)
    return quaternion

def main():
    rclpy.init()
    navigation_client = NavigationClient()

    goal_pose = create_pose_stamped(0.5, 0.2, 6)

    navigation_client.send_goal(goal_pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
