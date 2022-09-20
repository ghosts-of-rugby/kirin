import os, math

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  field_color = "red"

  field = LaunchConfiguration('field', default="red")
  use_hardware = LaunchConfiguration('use_hardware', default=False)

  # get urdf and rviz config path
  kirin_package_path = get_package_share_path('kirin')
  urdf_path = os.path.join(kirin_package_path, 'urdf/kirin.urdf.xacro')
  rviz_config_path = os.path.join(kirin_package_path, 'rviz/kirin.rviz')

  params_file = LaunchConfiguration(
    'params', default=[ThisLaunchFileDir(), '/params.yaml'])
  sim_file = LaunchConfiguration(
    'sim', default=[ThisLaunchFileDir(), '/sim.yaml'])

  topics_list = [
    '/joy',
    # '/all_state',
    '/rviz/date',
    '/rviz/phi',
    '/rviz/r',
    '/rviz/z',
    '/rviz/theta',
    '/rviz/next_target/pose',
    '/rviz/next_target/string',
    '/rviz/motor/right/reference',
    '/rviz/motor/left/reference',
    '/rviz/motor/theta/reference',
    '/rviz/motor/z/reference',
    '/rviz/motor/right/current',
    '/rviz/motor/left/current',
    '/rviz/motor/theta/current',
    '/rviz/motor/z/current',
  ]
  bag_play = ExecuteProcess(
    cmd=['ros2', 'bag', 'play', 'rosbag2_2022_09_18-15_49_34', '--topics' ]+topics_list,
    cwd=[os.path.expanduser('~'), '/ros2_ws/bag/'],
    output='log',
  )


  # robot_state_publisher
  # TODO evaluate from parameter
  robot_description = ParameterValue(Command(['xacro ', str(urdf_path), ' field:='+field_color]),
                                     value_type=str)
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[sim_file, {'robot_description': robot_description}]
  )
  
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[params_file, sim_file]
  )

  # joy_node = Node(
  #   package='joy',
  #   executable='joy_node',
  #   parameters=[params_file]
  # )
  
  # arg にする
  if (field_color == "red"):
    base_position = [-0.95, 0.0, 0]
    base_orientation = [0.0, 0.0, 0.0]
  else:
    base_position = [0.95, 0.0, 0]
    base_orientation = [math.pi, 0.0, 0.0]
  base_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=list(map(str, base_position)) + list(map(str, base_orientation)) + ["base_link", "fix_base"],
    parameters=[sim_file]
  )
  field_orientation = [0.0, 0.0, 0.0]
  field_frame_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="field_frame_publisher",
    arguments=list(map(str, [0.0, 0.0, 0.0])) + list(map(str, field_orientation)) + ["base_link", "field"],
    parameters=[sim_file]
  )

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', str(rviz_config_path)],
      parameters=[sim_file]
  )

  kirin_main_executor = Node(
    package='kirin', executable='kirin_main_executor', output='screen',
    parameters=[params_file, sim_file, {'use_hardware': use_hardware}, {'field': field}]
  )

  hand_tool_manager_node = Node(
    package='kirin', executable='hand_tool_manager_node', output='screen',
    parameters=[params_file, sim_file, {'use_hardware': use_hardware}, {'field': field}]
  )

  # rapid_hand_controller_node = Node(
  #   package='kirin', executable='rapid_hand_controller_node', output='screen',
  #   parameters=[params_file, {'use_hardware': use_hardware}]
  # )

  jagariko_marker_publiser = Node(
    package='kirin', executable='jagariko_marker_publisher', output='screen',
    parameters=[sim_file, {'field': field}]
  )

  # jsk_visualize_node = Node(
  #   package='kirin', executable='jsk_visualize_node',
  #   parameters=[sim_file]
  # )

  return LaunchDescription([
    bag_play,
    # joy_node,
    kirin_main_executor,
    hand_tool_manager_node,
    joint_state_publisher_node,
    robot_state_publisher_node,
    base_publisher,
    field_frame_publisher,
    rviz_node,
    jagariko_marker_publiser,
    # rapid_hand_controller_node,
    # jsk_visualize_node
  ])
