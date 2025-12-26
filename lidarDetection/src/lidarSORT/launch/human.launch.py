import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    lidarSORT_package = FindPackageShare('lidarsort')
    rviz_config_file = PathJoinSubstitution(
        [lidarSORT_package, 'rviz', '']
    )
    
    human_detector_node = Node(
        package='lidarsort',
        executable='main.py',
        name='human_detector',
        output='screen',
        parameters=[
            {'buffer_size': 20},
            {'processing_rate': 10.0},
            {'detection_range': 10.0},
            {'voxel_size': 0.1},
            {'ground_removal_height': -0.5},
            
            {'human_height_min': 1.3},
            {'human_height_max': 2.2},
            {'human_width_max': 1.0},
            
            # DBSCAN聚类参数
            {'dbscan_eps': 0.3},
            {'dbscan_min_samples': 20},
            
            # 运动检测参数
            {'motion_threshold': 0.2},
            {'min_detection_points': 50},
            
            # 跟踪参数
            {'max_tracking_distance': 2.0},
            {'tracker_timeout': 2.0},
            
            # 坐标系ID
            {'world_frame': 'odom'},
            {'lidar_frame': 'lidar_body'}
        ]
    )
    
    # 创建静态TF发布器 (odom -> lidar_body)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_body']
    )
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(human_detector_node)
    ld.add_action(static_transform_publisher)
    ld.add_action(rviz_node)
    
    return ld