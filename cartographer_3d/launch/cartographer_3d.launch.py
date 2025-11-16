import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'cartographer_3d'
    pkg_share = get_package_share_directory(pkg_name)

    # Cartographer 配置文件目录（go2_3d.lua 在这里）
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'go2_3d.lua'

    # 你想存放 bag 和 pbstream 的目录（按你之前的约定）
    slam_local_dir = '/home/unitree/ros2_ws/LeggedRobot/src/Ros2SLAM/local'
    bag_output_prefix = os.path.join(slam_local_dir, 'go2_slam_bag')

    # 1) Cartographer 3D 主节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename',  configuration_basename,
        ],
        remappings=[
            ('points2', '/SMX/Go2Lidar'),
            ('imu',     '/SMX/Go2IMU'),
            ('odom',    '/SMX/Odom'),
        ]
    )

    # 2) 2D Occupancy Grid 节点（慢速更新 2D /map）
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ],
        arguments=[
            '-resolution', '0.05',                      # 2D 栅格地图分辨率 5cm
            '-publish_period_sec', '1.0',               # 每 1s 更新一次 /map
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename',  configuration_basename,
        ],
    )

    # 3) rosbag2 录制 Go2 的关键话题
    clean_old_bag = ExecuteProcess(
        cmd=['rm', '-rf', bag_output_prefix],
        output='screen'
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/tf',
            '/tf_static',
            '/SMX/Go2Lidar',
            '/SMX/Go2IMU',
            '/SMX/Odom',
            '-o', bag_output_prefix
        ],
        output='screen'
    )

    return LaunchDescription([
        cartographer_node,
        clean_old_bag,
        occupancy_grid_node,
        rosbag_record,
    ])
