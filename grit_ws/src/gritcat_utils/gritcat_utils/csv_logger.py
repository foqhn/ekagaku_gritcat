#!/usr/bin/env python3
from sensor_msgs.msg import Imu, Image,MagneticField
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MultiSubscriberNode(Node):
    """
    複数のサブスクライバを持つノードの例
    """
    def __init__(self):
        super().__init__('multi_subscriber_node')
        # 2つの異なるトピックに対するサブスクライバ
        self.imu_subscription = self.create_subscription(
            Imu, '/bno055/imu', self.imu_callback, 10)
        self.mag_subscription = self.create_subscription(
            MagneticField, '/bno055/mag', self.mag_callback, 10)
        self.get_logger().info('複数サブスクライバノードが開始されました')

    def string_callback(self, msg):
        self.get_logger().info(f'string_topicから受信: {msg.data}')

    def int_callback(self, msg):
        self.get_logger().info(f'int_topicから受信: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()