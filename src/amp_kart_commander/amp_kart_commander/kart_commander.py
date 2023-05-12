'''Server that receives updates to TrackState from RCS and sets the kart cmd_vel mux accordingly'''

#usr/bin/env python3
from amp_msgs.srv import TrackState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

# Create high-reliability QoS for mux toggling
qos_profile = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Last always has priority anyways
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    durability=DurabilityPolicy.
    TRANSIENT_LOCAL  # Re-send msg to late-joining subscribers
)


class TrackStateReciever(Node):

    def __init__(self):
        super().__init__('track_state_reciever')
        self.server = self.create_service(TrackState, 'track_state',
                                          self.track_state_callback)
        self.mux_publisher = self.create_publisher(String,
                                                   'select_topic',
                                                   qos_profile=qos_profile)

    def track_state_callback(self, request, response):
        response.state = request.state

        self.get_logger().info('Recieved: %s Response: %s' %
                               (request.state, response.state))

        # self.get_logger().info('eeee' + str(dir(TrackState.Request)))

        # TODO More complex states in TrackState.srv have transition rules not yet implemented

        if response.state == TrackState.Request.EMERGENCY or response.state == TrackState.Request.SAFESTOP:
            self.mux_publisher.publish(String(data='__none'))
            self.get_logger().info('STOP: nav2 and teleop disabled')
        if response.state == TrackState.Request.RC or response.state == TrackState.Request.RACEDONE:
            self.mux_publisher.publish(String(data='joy_vel'))
            self.get_logger().info('RC: joy only')
        if response.state == TrackState.Request.AUTONOMOUS_SLOW or response.state == TrackState.Request.AUTONOMOUS_OVERTAKE:
            self.mux_publisher.publish(String(data='nav_vel'))
            self.get_logger().info('AUTO: nav2 only')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrackStateReciever()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
