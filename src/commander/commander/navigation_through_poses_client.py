import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

class NavigationThroughPosesClient:
    def __init__(self):
        self.node = rclpy.create_node('navigation_through_poses_client')
        self.action_client = ActionClient(self.node, NavigateThroughPoses, '/navigate_through_poses')

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for NavigateThroughPoses action server...')

    def send_goal(self, goal_poses):
        # print(type([]))
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = goal_poses

        self.node.get_logger().info('Sending navigation through poses goal...')
        self.action_client.wait_for_server()

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        if send_goal_future.result() is not None:
            self.node.get_logger().info('Goal sent successfully')
            goal_handle = send_goal_future.result()

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future)
            result = get_result_future.result()
            self.node.get_logger().info(f'Navigation through poses result: {result.result}')
        else:
            self.node.get_logger().error('Failed to send navigation through poses goal')

def main():
    rclpy.init()
    navigation_client = NavigationThroughPosesClient()

    # Create a list of PoseStamped messages representing the goal poses
    goal_poses = [
        create_pose_stamped(0.5, 0.2, 1.0),
        create_pose_stamped(0.5, 1.0, 1.0),
        # Add more poses as needed
    ]

    navigation_client.send_goal(goal_poses)

    rclpy.shutdown()

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

if __name__ == '__main__':
    main()
