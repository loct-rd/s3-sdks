from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('s3_ros2_node'),
    'config'
  )

  s3_config_yaml = os.path.join(config_dir, 's3_config.yaml')

  return LaunchDescription([
    Node(
      package='s3_ros2_node',
      executable='s3_ros2_node',
      name='s3_ros2_node',
      parameters=[
        s3_config_yaml
      ],
      output='screen'
    )
  ])