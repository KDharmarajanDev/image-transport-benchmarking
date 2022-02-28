from image_transport_benchmarker.compression_test import CompressionRateTest
from image_transport_benchmarker.acknowledger import AcknowledgeTest
from sensor_msgs.msg import Image

import rclpy

def main(args=None):
    rclpy.init(args=args)

    # compression_rate_test = CompressionRateTest('/camera/image_raw', Image)
    acknowledge_test = AcknowledgeTest('/camera/image_raw/cloud')
    
    executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(compression_rate_test)
    executor.add_node(acknowledge_test)
    executor.spin()
    # compression_rate_test.destroy_node()    
    acknowledge_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()