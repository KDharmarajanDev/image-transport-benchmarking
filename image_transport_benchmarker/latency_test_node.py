import rclpy
import time
import csv
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool

class LatencyTest(Node):

    def __init__(self, topic_name):
        super().__init__('latency_test')

        self.source_topic_sub = self.create_subscription(
            Image,
            topic_name,
            self.on_source_receive,
            10)
        
        self.ack_topic_sub = self.create_subscription(
            Header,
            "/ack",
            self.on_ack_receive,
            10)
        
        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop',
            self.handle_stop,
        10)

        self.source_timestamps = []
        self.latencies = []
        self.cloud_received_images = 0
        self.first_seq = 0
        self.total_latency = 0
        self.last_received_time = None
        self.first_received_time = None

    def on_source_receive(self, image):
        if not self.source_timestamps:
            self.first_seq = int(image.header.frame_id)
        self.source_timestamps.append(time.time())
        self.latencies.append(-1)

    def on_ack_receive(self, ack):
        received_seq = int(ack.frame_id)
        arr_index = received_seq - self.first_seq
        curr_time = time.time()
        if not self.first_received_time:
            self.first_received_time = curr_time
        self.last_received_time = curr_time
        self.latencies[arr_index] = curr_time - self.source_timestamps[arr_index]
        self.total_latency += self.latencies[arr_index]
        self.cloud_received_images += 1

    def handle_stop(self, msg):
        self.get_logger().info('Saving latency and FPS data')
        self.save_data()

    def save_data(self):
        with open('latency_and_fps.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Dropped Frames:', len(self.source_timestamps) - self.cloud_received_images])
            writer.writerow(['Number of Acks Received: ', self.cloud_received_images])
            writer.writerow(['Frame Rate from Ack', self.cloud_received_images / (self.last_received_time - self.first_received_time)])

            # Original latencies are in seconds, so they need to be converted to milliseconds
            writer.writerow(['Total Latency (ms):', round(self.total_latency * 1000)])
            writer.writerow(['Avg Latency (ms):', round(self.total_latency * 1000) / self.cloud_received_images])
            writer.writerow(['Seq #', 'Latency (ms)'])
            for i, latency in enumerate(self.latencies):
                writer.writerow([i+self.first_seq, -1 if latency == -1 else round(latency * 1000)])
