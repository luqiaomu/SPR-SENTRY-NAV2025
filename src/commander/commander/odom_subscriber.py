import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

def odom_callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear_velocity = msg.twist.twist.linear
    angular_velocity = msg.twist.twist.angular

    print("Position: [{}, {}, {}]".format(position.x, position.y, position.z))
    print("Orientation: [{}, {}, {}]".format(orientation.x, orientation.y, orientation.z))
    print("Linear Velocity: [{}, {}, {}]".format(linear_velocity.x, linear_velocity.y, linear_velocity.z))
    print("Angular Velocity: [{}, {}, {}]".format(angular_velocity.x, angular_velocity.y, angular_velocity.z))

def main():
    rclpy.init()

    node = rclpy.create_node('odom_subscriber')
    odom_subscriber = node.create_subscription(
        Odometry,
        'odom',  # adjust the topic name based on your setup
        odom_callback,
        10  # adjust the queue size as needed
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
