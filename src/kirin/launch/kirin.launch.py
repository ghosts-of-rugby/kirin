from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  kirin_package_path = get_package_share_path('kirin')
  urdf_path = kirin_package_path / 'urdf/kirin.urdf.xacro'
  rviz_config_path = kirin_package_path / 'rviz/kirin.rviz'

  # robot_state_publisher
  robot_description = ParameterValue(Command(['xacro ', str(urdf_path)]), value_type=str)
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
  )
  
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{'source_list': ['manual_joint']}]
  )

  joy_node = Node(
    package='joy',
    executable='joy_node'
  )
  
  # arg にする
  red = 1
  base_position = [red * -0.9, 0.1, 0]
  base_orientation = [0.0, 0.0, 0.0, 1.0]
  base_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=list(map(str, base_position)) + list(map(str, base_orientation)) + ["base_link", "fix_base"]
  )

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', str(rviz_config_path)],
  )

  manual_controller_node = Node(
    package='kirin',
    executable='manual_controller_node',
    output='screen'
  )

  jagariko_marker_publiser = Node(
    package='kirin',
    executable='jagariko_marker_publisher'
  )

  return LaunchDescription([
    joy_node,
    manual_controller_node,
    joint_state_publisher_node,
    robot_state_publisher_node,
    base_publisher,
    rviz_node,
    jagariko_marker_publiser
  ])
