import rclpy
from std_msgs.msg import Bool
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('bool_publisher')
    publisher = node.create_publisher(Bool, 'cancel_behavior', 10)

    
    bool_msg = Bool()
    bool_msg.data = True  # Set the Bool data to True or False as needed

    for i in range(15):
        node.get_logger().info('Publishing: {}'.format(bool_msg.data))
        publisher.publish(bool_msg)
        time.sleep(0.1)
   

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
