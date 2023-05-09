from amp_msgs.srv import NetworkState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(NetworkState, 'network_state')
        self.publisher_ = self.create_publisher(Bool, '/stop', 2)
        if not self.client.wait_for_service(timeout_sec=5.0):
            kill_msg = Bool()
            kill_msg.data = True
            self.publisher_.publish(kill_msg)
            self.get_logger().info("Kill msg sent")

        self.request = NetworkState.Request()

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main(args=None):

    rclpy.init(args=args)

    while rclpy.ok():
        client = ServiceClient()
        #print(client.client.wait_for_service())
        if client.client.wait_for_service():
            response = client.send_request()
            print(response.network_state)
        client.destroy_node()
        time.sleep(1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
