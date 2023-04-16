'''ROS2 service client for state based service calling'''

#usr/bin/env python3
from amp_msgs.srv import TrackState
import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(TrackState, 'track_state')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(
                'service not found and/or available, waiting again...')
        self.request = TrackState.Request()

    def send_request(self, state):
        self.request.state = state
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


class KartStateDetermination(Node):

    def __init__(self):
        super().__init__('kart_state_determination')
        self.state = "Red"  # Change to different default string?

        self.track_subscription = self.create_subscription(
            String, "track_state", self.callback, 10)
        self.kart_publisher = self.create_publisher(String, "kart_state", 10)

    def callback(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            service_client = ServiceClient()
            response = service_client.send_request(self.state)
            response_msg = String()
            response_msg.data = response.state
            self.kart_publisher.publish(response_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KartStateDetermination()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
