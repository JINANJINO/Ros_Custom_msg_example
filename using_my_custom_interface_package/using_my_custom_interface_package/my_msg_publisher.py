import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoTextMessage

class Publisher(Node):
    def __init__(self):
        super().__init__('custom_msg_publisher') # New node name
        qos_profile = QoSProfile(depth = 10)
        self.publisher = self.create_publisher(TwoTextMessage, 'my_custom_msg', qos_profile)
        self.timer = self.create_timer(1, self.publish_msg)
        self.count = 0
        
    def publish_msg(self):
        msg = TwoTextMessage()
        msg.stamp = self.get_clock().now().to_msg()
        msg.text_a = 'text_a: {0}'.format(self.count)
        msg.text_b = 'text_a: {0}'.format(self.count * 2)
        
        self.publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.text_a))
        self.get_logger().info('Published message: {0}'.format(msg.text_b))
        self.count += 1
        
# main function
def main(args = None):
    rclpy.init(args=args)
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        