import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoTextMessage

class Subscriber(Node):
    
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        qos_profile = QoSProfile(depth = 10)
        self.subscriber = self.create_subscription(
            TwoTextMessage,
            'my_custom_msg',
            self.subscribe_topic_message,
            qos_profile
        )
        
    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.text_a))
        self.get_logger().info('Received message: {0}'.format(msg.text_b))
        
def main(args=None):
    rclpy.init(args=None)
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
    