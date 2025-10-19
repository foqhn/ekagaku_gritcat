"""ROS 2 subscriber node and shared state for FastAPI endpoints.

This module tries to import rclpy and cv_bridge if available. When not
running inside a ROS2 environment these imports will be skipped so the
web app can still start for development/testing.
"""
import threading
from typing import Optional

latest_imu_msg = None
latest_image_msg = None
imu_lock = threading.Lock()
image_lock = threading.Lock()


def has_cv_bridge() -> bool:
    try:
        import cv_bridge  # type: ignore
        return True
    except Exception:
        return False


def image_msg_to_jpeg_bytes(img_msg) -> Optional[bytes]:
    """Convert a ROS Image message to JPEG bytes. Requires cv_bridge and OpenCV.

    Returns None when conversion isn't possible.
    """
    try:
        from cv_bridge import CvBridge  # type: ignore
        import cv2  # type: ignore
    except Exception:
        return None

    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        ret, frame = cv2.imencode('.jpg', cv_image)
        if not ret:
            return None
        return frame.tobytes()
    except Exception:
        return None


try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import Imu, Image  # type: ignore
    from cv_bridge import CvBridge  # type: ignore


    class RosSubscriberNode(Node):
        def __init__(self):
            super().__init__('ros_fastapi_subscriber')
            self.imu_subscription = self.create_subscription(
                Imu, '/bno055/imu', self.imu_callback, 10)
            self.image_subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)
            self.bridge = CvBridge()
            self.get_logger().info('ROS Subscriber Node has been started.')

        def imu_callback(self, msg):
            global latest_imu_msg
            with imu_lock:
                latest_imu_msg = msg

        def image_callback(self, msg):
            global latest_image_msg
            with image_lock:
                latest_image_msg = msg
            self.get_logger().info('Image data received by ROS node')


    def imu_to_dict(imu_msg: Imu):
        if not imu_msg:
            return None
        return {
            'header': {
                'stamp': {'sec': imu_msg.header.stamp.sec, 'nanosec': imu_msg.header.stamp.nanosec},
                'frame_id': imu_msg.header.frame_id
            },
            'orientation': {
                'x': imu_msg.orientation.x, 'y': imu_msg.orientation.y,
                'z': imu_msg.orientation.z, 'w': imu_msg.orientation.w
            },
            'angular_velocity': {
                'x': imu_msg.angular_velocity.x, 'y': imu_msg.angular_velocity.y, 'z': imu_msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': imu_msg.linear_acceleration.x, 'y': imu_msg.linear_acceleration.y, 'z': imu_msg.linear_acceleration.z
            }
        }


    def run_ros_node():
        rclpy.init()
        ros_node = RosSubscriberNode()
        try:
            rclpy.spin(ros_node)
        except KeyboardInterrupt:
            pass
        finally:
            ros_node.destroy_node()
            rclpy.shutdown()

except Exception:
    # If ROS2 isn't available, provide no-op implementations so the web app can still run.
    def run_ros_node():
        print('ROS2 not available in this environment — run_ros_node is a no-op')


    def imu_to_dict(imu_msg):
        return None
