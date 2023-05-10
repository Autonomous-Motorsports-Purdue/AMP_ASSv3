"""ROS2 service server for behavioral determination based on client call."""

from rcs_service.srv import TrackState
import rclpy
from rclpy.node import Node


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')
        self.server = self.create_service(TrackState, 'track_state',
                                          self.track_state_callback)

    def track_state_callback(self, request, response):
        response.state = request.state
        self.get_logger().info('Recieved: %s' % (request.state))
        self.get_logger().info('Response: %s\n' % (response.state))

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
