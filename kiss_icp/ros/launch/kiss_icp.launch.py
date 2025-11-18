import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node

PACKAGE_NAME = "kiss_icp"

default_config_file = os.path.join(
    get_package_share_directory(PACKAGE_NAME), "config", "config.yaml"
)


def generate_launch_description():
    # 真机：不用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # LiDAR 点云 topic
    pointcloud_topic = LaunchConfiguration("topic", default="/SMX/Go2Lidar")
    visualize = LaunchConfiguration("visualize", default="false")

    # 直接把 LiDAR 自己当 base，避免 TF 不存在的问题
    base_frame = LaunchConfiguration("base_frame", default="utlidar_lidar")

    # 里程计坐标系单独起名，不要用传感器 frame 名
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame", default="odom_kiss_icp")

    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default="true")
    invert_odom_tf = LaunchConfiguration("invert_odom_tf", default="false")

    position_covariance = LaunchConfiguration("position_covariance", default=0.1)
    orientation_covariance = LaunchConfiguration("orientation_covariance", default=0.1)

    config_file = LaunchConfiguration("config_file", default=default_config_file)

    # ===== KISS-ICP node =====
    kiss_icp_node = Node(
        package=PACKAGE_NAME,
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": invert_odom_tf,
                "publish_debug_clouds": visualize,
                "use_sim_time": use_sim_time,
                "position_covariance": position_covariance,
                "orientation_covariance": orientation_covariance,
            },
            config_file,
        ],
    )

    return LaunchDescription(
        [
            kiss_icp_node,
        ]
    )
