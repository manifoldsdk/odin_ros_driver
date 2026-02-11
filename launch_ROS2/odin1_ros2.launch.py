
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')
    
    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )
    
    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )
    
    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
       # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f) 
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # Cloud reprojection node
    reprojection_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f) 
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path 
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',  
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # Create RViz2 node - loads specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)  # Add RViz configuration argument
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(rviz_node)  # Add RViz node
    
    return ld
