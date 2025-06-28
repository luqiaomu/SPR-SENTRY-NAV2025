import rclpy                              
from rclpy.node import Node            
import time
from std_msgs.msg import Float32
from std_msgs.msg import String
from interface.msg import EventTime

class TimePerception(Node):
    def __init__(self, name):      
        super().__init__(name)
        self.events_subscription = self.create_subscription(String, 'events', self.events_callback, 10)
        self.duration_publisher = self.create_publisher(Float32, 'duration', 10)
        self.specific_event_duration_publisher = self.create_publisher(EventTime, 'specific_event_duration', 10)
        self.start_time = time.time()
        self.current_timestamp = time.time()
        self.duration = 0.0
        self.specific_duration_dict = {'event2': []}
        self.timer_spin_once = self.create_timer(0.1, self.Spin_Once)
 

    def Spin_Once(self):
        self.current_timestamp = time.time()
        self.duration = self.current_timestamp - self.start_time
        self.time_publisher(self.duration)
        for key in self.specific_duration_dict.keys():
            if len(self.specific_duration_dict[key]) != 0:
                self.specific_duration_dict[key][1] = self.duration = self.current_timestamp - self.specific_duration_dict[key][0]
        self.specific_time_publisher(self.specific_duration_dict)


    def time_publisher(self, duration):
        msg = Float32()
        value = float(duration)
        msg.data = value
        # self.get_logger().info(f"Publishing: {msg.data}")
        self.duration_publisher.publish(msg)

    def specific_time_publisher(self, specific_duration_dict):
        for key in specific_duration_dict.keys():
            if len(specific_duration_dict[key]) != 0:
                msg = EventTime()
                msg.specific_event = key
                msg.time = specific_duration_dict[key][1]
                # self.get_logger().info(f"Publishing: {msg}")
                self.specific_event_duration_publisher.publish(msg)

    # Event perception callback function
    def events_callback(self, msg):
        # self.get_logger().info('Received event: "%s"' % msg.data)
        if msg.data in self.specific_duration_dict:
            if len(self.specific_duration_dict[msg.data]) == 0:
                self.specific_duration_dict[msg.data].append(self.current_timestamp)
                self.specific_duration_dict[msg.data].append(0.0)


def main(args=None):             
    rclpy.init(args=args)
    node = TimePerception("time_perception")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
