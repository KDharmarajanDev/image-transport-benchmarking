from concurrent.futures import thread
import rclpy
import time
import csv
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool
import threading

class AcknowledgeTest(Node):

    def __init__(self, topic_name):
        super().__init__('acknowledge_node')

        self.source_topic_sub = self.create_subscription(
            Image,
            topic_name,
            self.on_source_receive,
            10)
        
        self.ack_topic_pub = self.create_publisher(
            Header,
            "/ack",
            10)
        
        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop',
            self.handle_stop,
        10)

        self.start_pub = self.create_publisher(
            Bool, 
            "/start",
            10
        )
        self.delayed_start = threading.Timer(20, self.start_pub.publish, [Bool()])
        self.delayed_start.start()
        self.cloud_received_images = 0
        self.last_received_time = None
        self.first_received_time = None

    def on_source_receive(self, image):
        self.get_logger().info("Received image!")
        curr_time = time.time()
        if not self.first_received_time:
            self.first_received_time = curr_time
        self.cloud_received_images += 1
        self.last_received_time = curr_time
        self.ack_topic_pub.publish(image.header)

    def handle_stop(self, msg):
        self.get_logger().info('Saving FPS data')
        self.save_data()

    def save_data(self):
        with open('fps.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Number of Frames Received: ', self.cloud_received_images])
            writer.writerow(['Avg. Frame Rate', self.cloud_received_images / (self.last_received_time - self.first_received_time)])