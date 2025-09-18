import rclpy
import random
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoNumSum

class Publisher(Node):
    def __init__(self):
        super().__init__('jinhan_publisher')
        qos_profile = QoSProfile(depth=10)
        self.sum_publisher = self.create_publisher(TwoNumSum, 'numbers', qos_profile)
        self.timer = self.create_timer(1, self.publish_msg)
        self.count = 0
        
    def publish_msg(self):
        msg = TwoNumSum()
        msg.num_a = random.randint(0, 99)
        msg.num_b = random.randint(0, 99)
        
        self.sum_publisher.publish(msg)
        self.get_logger().info(f'Published num_a message: {msg.num_a}')
        self.get_logger().info(f'Published num_b message: {msg.num_b}')
        self.count += 1

def main(args=None):
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
