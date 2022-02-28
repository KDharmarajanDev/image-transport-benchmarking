import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool
import csv

class CompressionRateTest(Node):

    def __init__(self, topic, data_type):
        super().__init__('bps_test')
        self.subscription = self.create_subscription(
            data_type,
            topic,
            self.listener_callback,
            10)
        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop',
            self.handle_stop,
        10)
        self.subscription  # prevent unused variable warning
        self.total_bytes = 0
        self.total_received_messages = 0
        self.start_time = 0
        self.end_time = 0

    def listener_callback(self, msg):
        self.total_bytes += len(msg.data)
        self.total_received_messages += 1
        if not self.start_time:
            self.start_time = time.time()
        self.end_time = time.time()

    def handle_stop(self, msg):
        self.get_logger().info('Saving BPS data')
        self.save_data()

    def save_data(self):
        with open('bps_robot.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Number of Local Images Received: ', self.total_received_messages])
            writer.writerow(['Avg bytes per message:', self.total_bytes / self.total_received_messages])
            writer.writerow(['Avg bytes per second:', self.total_bytes / (self.end_time - self.start_time)])
