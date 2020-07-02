#!/usr/bin/python3
import sys

from fruit_detection.srv import Classification
from fruit_detection.srv import Detection
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(Detection, 'DetectionService')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Detection.Request()

    def send_request(self):
        self.req.file = str(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                result1 = str(response.classes)
                minimal_client.get_logger().info(
                    'Result %s = %s' %
                    (minimal_client.req.file, result1))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
