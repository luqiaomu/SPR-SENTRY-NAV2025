import rclpy                              
from rclpy.node import Node            
import time
from std_msgs.msg import String

class TimePerception(Node):
    def __init__(self):      
        self.node = rclpy.create_node("event_perception")
        self.events_publisher = self.node.create_publisher(String, 'events', 10)
        self.start_time = time.time()    
        self.i = 0
        while rclpy.ok():
            self.spin_once()
            time.sleep(0.1)    

    def spin_once(self):
        if self.i < 50:
            self.event_publisher("3")
        if self.i > 600 and self.i < 700:
            self.event_publisher("1")
        self.i+=1


    def event_publisher(self, event):
        msg = String()
        msg.data = event
        self.node.get_logger().info(f"Publishing: {msg.data}")
        self.events_publisher.publish(msg)  


def main(args=None):             
    rclpy.init(args=args)
    node = TimePerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
