import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoNumSum

class Subscriber(Node):
    def __init__(self):
        super().__init__('jinhan_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.subscriber = self.create_subscription(
            TwoNumSum,
            'numbers',
            self.subscribe_topic_message,
            qos_profile
        )
        
    def subscribe_topic_message(self, msg):
        total = msg.num_a + msg.num_b
        self.get_logger().info('Received num_a message: {}'.format(msg.num_a))
        self.get_logger().info('Received num_b message: {}'.format(msg.num_b))
        self.get_logger().info('Received sum message: {}'.format(total))
        
def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
