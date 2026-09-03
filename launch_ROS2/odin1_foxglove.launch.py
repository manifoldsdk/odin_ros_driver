
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
    
    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    # Load parameters for pcd2depth node
    # Note: This logic follows the pattern in odin1_ros2.launch.py
    # Ideally this should use the launch argument, but we need to load the file to pass dict as params
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

    # Foxglove Bridge Node
    # This node allows Foxglove Studio to connect to the ROS 2 system
    # Requires: sudo apt install ros-<distro>-foxglove-bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(foxglove_bridge_node)
    
    return ld
