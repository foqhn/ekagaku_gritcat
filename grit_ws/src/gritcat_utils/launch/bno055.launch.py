"""
GritCat 用 BNO055 (IMU) 起動ファイル

外部パッケージ bno055 (https://github.com/flynneva/bno055) のノードを、
GritCat の設定 (このパッケージの config/bno055_params_i2c.yaml) と namespace 付きで起動する。
bno055 リポジトリ側のファイルは一切書き換えずに使う。

使い方:
  ros2 launch gritcat_utils bno055.launch.py namespace:=robot03
  → トピックは /robot03/bno055/imu, /robot03/bno055/mag など
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('gritcat_utils'), 'config', 'bno055_params_i2c.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ノードの namespace (GritCat ではロボットID)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='bno055 ノードのパラメータファイル'),
        Node(
            package='bno055',
            executable='bno055',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
