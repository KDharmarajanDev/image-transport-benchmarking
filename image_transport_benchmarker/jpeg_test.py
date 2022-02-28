from image_transport_benchmarker.compression_test import CompressionRateTest
from image_transport_benchmarker.latency_test_node import LatencyTest
from sensor_msgs.msg import CompressedImage

import rclpy

def main(args=None):
    rclpy.init(args=args)

    compression_rate_test = CompressionRateTest('/camera/image_raw/compressed', CompressedImage)
    latency_test = LatencyTest('/camera/image_raw')
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(compression_rate_test)
    executor.add_node(latency_test)
    executor.spin()
    compression_rate_test.destroy_node()    
    latency_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()