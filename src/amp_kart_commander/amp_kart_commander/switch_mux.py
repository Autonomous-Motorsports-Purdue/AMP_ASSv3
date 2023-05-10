''' Mux the allows choosing a single topic to let through '''
#usr/bin/env python3
import rclpy
import functools

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
# Create lossy QoS that is compatible with any publisher
subscriber_any = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                            depth=5,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)

# Create high-reliability QoS for topic selection
reliable = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Last always has priority anyways
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    durability=DurabilityPolicy.TRANSIENT_LOCAL  # Re-send msg to late-joiners
)

EMPTY_TOPIC = "__none"


class SwitchMux(Node):

    def __init__(self):
        super().__init__('switch_mux', allow_undeclared_parameters=True)

        self.declare_parameter('initial_topic', String)
        self.declare_parameter('input_topics')

        # `initial_topic` is optionally defined at initialization
        try:
            self.current_topic = self.get_parameter('initial_topic').value
        except ParameterNotDeclaredException:
            self.current_topic = EMPTY_TOPIC

        # `input_topics` must be defined at initialization
        self.input_topics = self._parameters['input_topics'].value
        if self.input_topics is None:
            raise ParameterNotDeclaredException('input_topics')

        for topic_name in self.input_topics:
            # This specific name is disallowed to guarantee that it disables the mux
            if topic_name == EMPTY_TOPIC:
                continue
            # All input topics share the same base callback (input_topic_callback), but we
            # bind a third `topic_name` parameter that changes with the topic
            named_callback = functools.partial(self.input_topic_callback,
                                               topic_name)
            self.create_subscription(Twist, topic_name, named_callback,
                                     subscriber_any)

        # Topic name
        self.subscription = self.create_subscription(String, 'select_topic',
                                                     self.set_topic_callback,
                                                     reliable)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', reliable)

    def set_topic_callback(self, topic_name_msg):
        self.current_topic = str(topic_name_msg.data)

    def input_topic_callback(self, topic_name, twist_msg):
        ''' Upon receiving a message to any registered input of the mux'''
        self.get_logger().info(
            f"got from {topic_name} curr is {self.current_topic}")
        if topic_name == self.current_topic:
            self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwitchMux()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
