from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
  launch_pkg_prefix = get_package_share_directory('dag_case')
  param_file = os.path.join(launch_pkg_prefix, 'param/config.yaml')

  # Argument
  syn_param = DeclareLaunchArgument(
    'param_file',
    default_value = param_file,
    description='Path to config file for dag_case'
  )
  
  # Node
  dag_case_node = Node(
    package='dag_case',
    executable='dag_case',
    namespace='',
    parameters=[LaunchConfiguration('param_file')]
  )

  return LaunchDescription([
    syn_param,
    dag_case_node
  ])