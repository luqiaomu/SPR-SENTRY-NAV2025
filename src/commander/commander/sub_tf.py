import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        # 创建订阅者
        self.subscription = self.create_subscription(
            TransformStamped,
            '/tf',
            self.tf_callback,
            10  # 10 is the queue size
        )
        self.subscription  # prevent unused variable warning

    def tf_callback(self, msg):
        # 回调函数，处理接收到的TF变换消息
        transform = msg
        self.get_logger().info(f"Received TF transform: {transform}")

def main():
    rclpy.init()
    node = TFListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
