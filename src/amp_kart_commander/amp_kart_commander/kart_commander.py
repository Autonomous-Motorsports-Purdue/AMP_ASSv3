'''Server that receives updates to TrackState from RCS and sets the kart cmd_vel mux accordingly'''

#usr/bin/env python3
from amp_msgs.srv import TrackState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

# Create high-reliability QoS for mux toggling
qos_profile = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Last always has priority anyways
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    durability=DurabilityPolicy.
    TRANSIENT_LOCAL  # Re-send msg to late-joining subscribers
)


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')
        self.server = self.create_service(TrackState, 'track_state',
                                          self.track_state_callback)
        self.stop_publisher = self.create_publisher(Bool,
                                                    'stop',
                                                    qos_profile=qos_profile)
        self.joy_only_publisher = self.create_publisher(
            Bool, 'joy_only', qos_profile=qos_profile)

    def track_state_callback(self, request, response):
        response.state = request.state

        self.get_logger().info('Recieved: %s' % (request.state))
        self.get_logger().info('Response: %s\n' % (response.state))

        # TODO More complex states in TrackState.srv have transition rules not yet implemented

        # When "teleop_only" is false and "stop" is false, then both joy and nav2 messages
        # can coexist.
        # Note that while you can reach Stop from any other state, returning to
        # Autonomus from Stop is not allowed; switch to RC first.
        if response.state == response.ESTOP or response.state == response.SAFESTOP:
            self.stop_publisher.publish(True)
            self.get_logger().info('STOP: nav2 and teleop disabled')
        if response.state == response.RC or response.state == response.RACEDONE:
            self.joy_only_publisher.publish(True)
            self.get_logger().info('RC: nav2 disabled')
            self.stop_publisher.publish(False)
            self.get_logger().info('RC: STOP mode restrictions lifted')
        if response.state == response.AUTONOMOUS_SLOW or response.state == response.AUTONOMOUS_OVERTAKE:
            self.joy_only_publisher.publish(False)
            self.get_logger().info(
                'AUTO: allow both nav2 and teleop controls. Switching directly from'
                'ESTOP/SAFESTOP to this state is not allowed. Switch to RC first'
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
