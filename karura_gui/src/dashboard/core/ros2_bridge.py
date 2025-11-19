import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ROS2Bridge(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(String, 'topic_out', 10)
        self.subscription = self.create_subscription(
            String,
            'topic_in',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        response_msg = String()
        response_msg.data = f'Echo: {msg.data}'
        self.publisher_.publish(response_msg)
        self.get_logger().info(f'Published message: {response_msg.data}')