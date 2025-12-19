#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

# ROS2 采用.launch.py 文件来启动多个节点和配置参数,居多,不同于ROS1
def generate_launch_description():
    lidar_detection_pkg_share = get_package_share_directory('lidar_detection')

    config_file = os.path.join(
        lidar_detection_pkg_share,
        'config',
        'go2.yaml'
    )

    # static transform: lidar_body -> base_link
    static_tf_lidar_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_body_to_base_link_publisher',
        output='screen',
        arguments=[
            # x, y, z, roll, pitch, yaw, parent_frame, child_frame
            '-0.1', '0', '-0.2', '0', '0', '0', 'lidar_body', 'base_link'
        ]
    )
    
    # static transform: map -> odom
    static_tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map_publisher',
        output='screen',
        arguments=[
            # x, y, z, roll, pitch, yaw, parent_frame, child_frame
            '0', '0', '0', '0', '0', '0', 'map', 'odom'
        ]
    )

    obstacle_detector_node = Node(
        package='lidar_detection',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[config_file]
    )

    odom_trans_node = Node(
        package='lidar_detection',
        executable='odom_trans_node',
        name='odom_trans_node',
        output='screen',
        parameters=[config_file]
    )

    obstacle_to_baselink_node = Node(
        package='lidar_detection',
        executable='obstacle_to_baselink_node',
        name='obstacle_to_baselink_node',
        output='screen',
        parameters=[config_file]
    )

    interface_node = Node(
        package='lidar_detection',
        executable='interface_node',
        name='interface_node',
        output='screen',
        parameters=[config_file]
    )

    # rqt_reconfigure (可选 - 用于动态调参)
    # rqt_reconfigure_node = Node(
    #     package='rqt_reconfigure',
    #     executable='rqt_reconfigure',
    #     name='rqt_reconfigure',
    #     output='screen'
    # )

    # rviz2
    rviz_config_file = PathJoinSubstitution(
        [lidar_detection_pkg_share, 'rviz', 'detection.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    delayed_interface_node1 = TimerAction(
        period=2.5,
        actions=[
            odom_trans_node,
            obstacle_to_baselink_node
        ]
    )
    
    delayed_interface_node2 = TimerAction(
        period=3.5,
        actions=[
            interface_node,
            # rqt_reconfigure_node
        ]
    )
    delayed_obstacle_detector = TimerAction(
        period=1.5,
        actions=[obstacle_detector_node]
    )
    return LaunchDescription([
        static_tf_lidar_to_base,
        static_tf_odom_to_map,
        delayed_obstacle_detector,
        delayed_interface_node1,
        delayed_interface_node2,
        rviz_node,
    ])
