# ~/ros2_ws/src/gpsd_ros2_driver/gpsd_ros2_driver/gpsd_client_node.py

import rclpy
from rclpy.node import Node
# NavSatStatus を追加でインポートする
from sensor_msgs.msg import NavSatFix, NavSatStatus

import gpsd
import math

class GpsdClientNode(Node):
    """
    gpsdからGPSデータを取得し、NavSatFixメッセージとしてパブリッシュするノード。
    """
    def __init__(self):
        super().__init__('gpsd_client_node')

        # パラメータの宣言と取得
        self.declare_parameter('frame_id', 'gps_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # 1秒ごとにタイマーコールバックを実行
        self.timer = self.create_timer(1.0, self.timer_callback)

        # gpsdへの接続
        try:
            gpsd.connect()
            self.get_logger().info('gpsdへの接続に成功しました。')
        except Exception as e:
            self.get_logger().error(f'gpsdへの接続に失敗しました: {e}')
            # ノードの初期化に失敗したため、プログラムを終了させる
            raise e

    def timer_callback(self):
        """
        タイマーによって定期的に呼び出され、GPSデータの取得とパブリッシュを行う。
        """
        try:
            packet = gpsd.get_current()
            
            # 測位が成功しているか (modeが2=2Dまたは3=3D) を確認
            if packet.mode >= 2:
                self.publish_navsatfix(packet)
            else:
                self.get_logger().warn('測位待機中... (mode < 2)')

        except Exception as e:
            self.get_logger().error(f'GPSデータの取得中にエラーが発生しました: {e}')

    def publish_navsatfix(self, packet):
        """
        取得したgpsdパケットからNavSatFixメッセージを作成してパブリッシュする。
        """
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # --- Status ---
        # 【修正点】NavSatFix ではなく NavSatStatus の定数を使用する
        if packet.mode == 3:
            msg.status.status = NavSatStatus.STATUS_FIX
        else: # mode == 2
            # 2D測位は信頼性が低いため、多くのROSアプリケーションではNO_FIXとして扱う
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        
        # 【修正点】NavSatFix ではなく NavSatStatus の定数を使用する
        msg.status.service = NavSatStatus.SERVICE_GPS

        # --- Position ---
        msg.latitude = packet.lat
        msg.longitude = packet.lon
        if packet.mode == 3:
            msg.altitude = packet.alt
        else:
            msg.altitude = float('nan') # 3D測位でない場合はNaN

        # --- Covariance (精度情報) ---
        if hasattr(packet, 'error') and isinstance(packet.error, dict):
            epy = packet.error.get('y', 0.0)
            epx = packet.error.get('x', 0.0)
            epv = packet.error.get('v', 0.0)
            
            msg.position_covariance[0] = epy ** 2
            msg.position_covariance[4] = epx ** 2
            msg.position_covariance[8] = epv ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # メッセージをパブリッシュ
        self.publisher_.publish(msg)
        self.get_logger().info(f"GPSデータ配信: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Status={msg.status.status}")


def main(args=None):
    rclpy.init(args=args)
    try:
        gpsd_client_node = GpsdClientNode()
        rclpy.spin(gpsd_client_node)
    except Exception as e:
        print(f"ノードの実行中に致命的なエラーが発生しました: {e}")
    finally:
        if 'gpsd_client_node' in locals() and rclpy.ok():
            gpsd_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()