import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to this package.
  pkg_share = FindPackageShare(package='md_controller').find('md_controller')

  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/md.rviz')

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
  
  # Launch motor driver controller
  md_controller_cmd = Node(
    package='md_controller',
    executable='md_controller',
    parameters=[{
      "MDUI":184,
      "MDT":183,
      "Baudrate":57600,
      "ID":2,
      "GearRatio":25,
      "poles":8
    }],
    output='screen'
  )

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_rviz_cmd) 

  # Add any actions
  ld.add_action(md_controller_cmd)
  ld.add_action(start_rviz_cmd)

  return ld
