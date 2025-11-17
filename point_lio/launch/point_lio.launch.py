from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 是否启动 RViz 的开关
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.'
    )

    # 使用我们为 Go2 + L1 准备的配置文件
    config_file = PathJoinSubstitution([
        FindPackageShare('point_lio'),
        'config', 'go2_l1.yaml'
    ])

    # Point-LIO 主要参数
    laser_mapping_params = [
        config_file,
        {
            # 你是有 IMU 的，所以这里建议直接用 IMU 作为输入模型
            'use_imu_as_input': True,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'cube_side_length': 100.0,
            'runtime_pos_log_enable': False,
        }
    ]

    # Point-LIO 主节点
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    # RViz（用自带的 loam_livox 配置）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        # rviz_node,
    ])
