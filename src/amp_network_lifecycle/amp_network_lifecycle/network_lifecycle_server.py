from amp_msgs.srv import NetworkState
import rclpy
from rclpy.node import Node


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')
        self.server = self.create_service(NetworkState, 'network_state',
                                          self.network_state_callback)

    def network_state_callback(self, request, response):
        response.network_state = "Connected!"
        self.get_logger().info('Response: %s\n' % (response.network_state))

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
