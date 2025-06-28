import rclpy                        
from rclpy.node import Node                   
from std_msgs.msg import Float32   

class time_sub(Node):
    def __init__(self, name):         
        super().__init__(name)                              
        self.time_subscription = self.create_subscription(Float32, 'duration', self.time_callback, 10)

    def time_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):                                 
    rclpy.init(args=args)                            
    node = time_sub("time_sub")        
    rclpy.spin(node)                                 
    node.destroy_node()                              
    rclpy.shutdown()                                 
