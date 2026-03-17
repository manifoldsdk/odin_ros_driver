"""
Odin ROS2 驱动启动文件
适配系统：Ubuntu 22.04.5 + ROS2 Humble
硬件支持：x86_64 (AMD) 和 ARM/AArch64

启动的节点：
  1. host_sdk_sample         - 主驱动节点，负责设备连接和数据流管理
  2. pcd2depth_ros2_node     - 点云转深度图节点（深度补全演示）
  3. cloud_reprojection_ros2_node - 点云重投影节点（投影到相机图像演示）
  4. rviz2                   - 可视化界面节点

使用方法：
  ros2 launch odin_ros_driver odin1.launch.py
  # 自定义配置文件：
  ros2 launch odin_ros_driver odin1.launch.py config_file:=/path/to/control_command.yaml
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # 获取功能包安装路径
    # ============================================================
    package_dir = get_package_share_directory('odin_ros_driver')

    # ============================================================
    # Launch 参数声明
    # ============================================================

    # 控制参数配置文件路径（默认使用包内 config/control_command.yaml）
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='控制参数 YAML 配置文件路径'
    )

    # RViz2 配置文件路径（存储在 rviz/ 目录下）
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'rviz', 'odin_ros2.rviz'),
        description='RViz2 可视化配置文件路径'
    )

    # ============================================================
    # 节点 1：主驱动节点 host_sdk_sample
    # 功能：通过 USB 连接 Odin 设备，管理数据流，发布 ROS2 话题
    # 话题（发布）：
    #   /odin1/imu              - IMU 数据
    #   /odin1/image            - RGB 图像（BGR8）
    #   /odin1/image/compressed - 压缩 RGB 图像（JPEG）
    #   /odin1/cloud_raw        - 原始稀疏点云
    #   /odin1/cloud_slam       - SLAM 处理后的彩色点云
    #   /odin1/cloud_render     - 彩色渲染点云
    #   /odin1/odometry         - 里程计数据
    #   /odin1/odometry_highfreq - 高频里程计数据
    # ============================================================
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        # 如需调试日志，取消以下注释：
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    # ============================================================
    # 节点 2：点云转深度图节点 pcd2depth_ros2_node
    # 功能：订阅 cloud_raw 和 RGB 图像，生成密集深度图
    # 依赖：control_command.yaml 和 config/calib.yaml 标定文件
    # 话题（订阅）：/odin1/cloud_raw, /odin1/image
    # 话题（发布）：/odin1/depth_image, /odin1/depth_cloud
    # 注意：计算资源消耗较高，默认关闭（senddepth: 0）
    # ============================================================
    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    # 注入标定文件绝对路径
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path

    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # ============================================================
    # 节点 3：点云重投影节点 cloud_reprojection_ros2_node
    # 功能：将 SLAM 点云（cloud_slam）结合里程计位姿重投影到相机图像平面
    # 依赖：control_command.yaml 和 config/calib.yaml 标定文件
    # 话题（订阅）：/odin1/cloud_slam, /odin1/odometry
    # 话题（发布）：/odin1/reprojected_image
    # 注意：演示功能，默认关闭（sendreprojection: 0）
    # ============================================================
    reprojection_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f)
    # 注入标定文件绝对路径
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path

    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # ============================================================
    # 节点 4：RViz2 可视化节点
    # 使用 rviz/ 目录下的配置文件，展示点云、轨迹和图像
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # ============================================================
    # 构建并返回 LaunchDescription
    # ============================================================
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(rviz_node)

    return ld
