import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'dynamic_pose', 10)
        self.timer = self.create_timer(0.1, self.Spin_Once)
        self.int_list = [0.5, 0.2, 1.0] 
        self.i = 0 

    def Spin_Once(self):
        if self.i == 50:
            self.publish_dynamic_pose([-2.0,0.2,1.0])
        elif self.i == 100:
            self.publish_dynamic_pose([-1.65,1.2,1.0])
        elif self.i == 150:
            self.publish_dynamic_pose([-0.65,1.85,1.0])
        elif self.i == 200:
            self.publish_dynamic_pose([0.5,1.6,1.0])
        elif self.i == 250:
            self.publish_dynamic_pose([-0.1,0.65,1.0])
        self.i += 1
        # self.publish_dynamic_pose([-0.1,0.65,1.0])

    def publish_dynamic_pose(self, pose_list):
        msg = Float32MultiArray()
        msg.data = pose_list
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
