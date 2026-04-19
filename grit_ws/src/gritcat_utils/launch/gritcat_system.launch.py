import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのshareディレクトリパスを正しく取得
    included_launch_path = os.path.join(
        get_package_share_directory('bno055'),
        'launch',
        'bno055.launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_launch_path)
        ),

        Node(
            package='gritcat_utils',
            namespace='',
            executable='motor_ctl_node',
        ),

        Node(
            package='gpsd_driver',
            namespace='gpsd_client_node',
            executable='gpsd_client_node',
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'format': 'YUYV'
            }]
        )
    ])