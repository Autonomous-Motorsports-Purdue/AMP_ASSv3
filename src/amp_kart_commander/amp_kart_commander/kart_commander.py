'''Server that receives updates to TrackState from RCS and sets the kart cmd_vel mux accordingly'''

#usr/bin/env python3
from amp_msgs.srv import TrackState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

# Create high-reliability QoS with guaranteed delivery
qos_reliable = QoSProfile(
    history=HistoryPolicy.
    KEEP_LAST,  # TRANSIENT_LOCAL set, not point in history
    depth=1,  # See above
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    durability=DurabilityPolicy.
    TRANSIENT_LOCAL  # Re-send msg to late-joining subscribers
)


class TrackStateReciever(Node):

    def __init__(self):
        super().__init__('track_state_reciever')
        self.create_subscription(String, 'track_state',
                                 self.track_state_callback, qos_reliable)
        self.kart_state_pub = self.create_publisher(String, 'kart_state',
                                                    qos_reliable)
        self.mux_publisher = self.create_publisher(String, 'select_topic',
                                                   qos_reliable)

    def track_state_callback(self, new_state):
        new_state: str = new_state.data

        self.get_logger().info(f'Recieved Track State: {new_state}')

        # self.get_logger().info('eeee' + str(dir(TrackState.Request)))

        # TODO More complex states in TrackState.srv have transition rules not yet implemented

        if new_state == TrackState.Request.EMERGENCY or new_state == TrackState.Request.SAFESTOP:
            self.mux_publisher.publish(String(data='__none'))
            self.get_logger().info('STOP: nav2 and teleop disabled')
        if new_state == TrackState.Request.RC or new_state == TrackState.Request.RACEDONE:
            self.mux_publisher.publish(String(data='joy_vel'))
            self.get_logger().info('RC: joy only')
        if new_state == TrackState.Request.AUTONOMOUS_SLOW or new_state == TrackState.Request.AUTONOMOUS_OVERTAKE:
            self.mux_publisher.publish(String(data='nav_vel'))
            self.get_logger().info('AUTO: nav2 only')


def main(args=None):
    rclpy.init(args=args)
    node = TrackStateReciever()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
