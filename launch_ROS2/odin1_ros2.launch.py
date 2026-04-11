
# USAGE:
#   Single device:  ros2 launch odin_ros_driver odin1_ros2.launch.py
#   Two devices:    ros2 launch odin_ros_driver odin1_ros2.launch.py num_devices:=2
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')
    config_dir   = os.path.join(package_dir, 'config')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'control_command.yaml'),
        description='Path to the control config YAML file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(config_dir, 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )
    num_devices_arg = DeclareLaunchArgument(
        'num_devices',
        default_value='1',
        description='Number of LiDAR devices to launch helper nodes for'
    )

    # Detect actual connected devices by scanning USB
    import subprocess
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
        # Count devices matching Rockchip VID:PID (2207:0019 or 2207:001a)
        connected_devices = 0
        for line in result.stdout.split('\n'):
            if '2207:0019' in line or '2207:001a' in line:
                connected_devices += 1
    except Exception:
        connected_devices = 1  # Default to 1 if detection fails

    # Read num_devices at generation time (static value needed to create node list)
    num_devices = int(os.environ.get('NUM_DEVICES', str(connected_devices)))
    # Also allow override via launch arg
    try:
        import sys
        for arg in sys.argv:
            if arg.startswith('num_devices:='):
                num_devices = int(arg.split(':=')[1])
    except Exception:
        pass
    # Use detected device count if not explicitly specified
    if num_devices == 1 and connected_devices > 1:
        num_devices = connected_devices
    
    print(f"[odin1_ros2.launch.py] Detected {connected_devices} connected device(s), launching {num_devices} device node(s)")

    # Load base config once
    base_config_path = os.path.join(config_dir, 'control_command.yaml')
    with open(base_config_path, 'r') as f:
        base_params = yaml.safe_load(f)

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}]
    )

    nodes = [config_file_arg, rviz_config_arg, num_devices_arg, host_sdk_node]

    for i in range(num_devices):
        ns = 'odin1_{}'.format(i)          # e.g. odin1_0, odin1_1
        # Use cam-index based calib path (calib file is generated at runtime by host_sdk_sample)
        calib_path = os.path.join(config_dir, 'calib_cam{}.yaml'.format(i))

        # --- pcd2depth node ---
        pcd2depth_params = dict(base_params)
        pcd2depth_params['calib_file_path'] = calib_path
        pcd2depth_params['cloud_raw_topic']  = '/{}/cloud_raw'.format(ns)
        pcd2depth_params['color_raw_topic']  = '/{}/image'.format(ns)
        pcd2depth_params['depth_image_topic'] = '/{}/depth_img_competetion'.format(ns)
        pcd2depth_params['depth_cloud_topic'] = '/{}/depth_img_competetion_cloud'.format(ns)
        nodes.append(Node(
            package='odin_ros_driver',
            executable='pcd2depth_ros2_node',
            name='pcd2depth_ros2_node_{}'.format(i),
            output='screen',
            parameters=[pcd2depth_params]
        ))

        # --- cloud reprojection node ---
        reproj_params = dict(base_params)
        reproj_params['calib_file_path']          = calib_path
        reproj_params['cloud_slam_topic']          = '/{}/cloud_slam'.format(ns)
        reproj_params['odometry_topic']            = '/{}/odometry'.format(ns)
        reproj_params['wiwc_topic']                = '/{}/wiwc'.format(ns)
        reproj_params['reprojected_image_topic']   = '/{}/reprojected_image'.format(ns)
        nodes.append(Node(
            package='odin_ros_driver',
            executable='cloud_reprojection_ros2_node',
            name='cloud_reprojection_ros2_node_{}'.format(i),
            output='screen',
            parameters=[reproj_params]
        ))

        # --- image overlay node ---
        overlay_params = dict(base_params)
        overlay_params['register_keys.overlay_reprojected_topic'] = '/{}/reprojected_image'.format(ns)
        overlay_params['register_keys.overlay_camera_topic']      = '/{}/image/undistorted'.format(ns)
        overlay_params['register_keys.overlay_output_topic']      = '/{}/overlay_image'.format(ns)
        nodes.append(Node(
            package='odin_ros_driver',
            executable='image_overlay_node',
            name='image_overlay_node_{}'.format(i),
            output='screen',
            parameters=[overlay_params]
        ))

    # RViz2
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    ))

    return LaunchDescription(nodes)
