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
  
  manual_controller_node = Node(
    package='kirin',
    executable='manual_controller_node',
    output='screen'
  )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', str(rviz_config_path)],
  )

  return LaunchDescription([
    joy_node,
    manual_controller_node,
    joint_state_publisher_node,
    robot_state_publisher_node,
    rviz_node
  ])
