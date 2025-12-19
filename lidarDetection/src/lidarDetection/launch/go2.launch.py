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

    # static transform: base_link -> lidar_body
    static_tf_lidar_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_body_publisher',
        output='screen',
        arguments=[
            # x, y, z, roll, pitch, yaw, parent_frame, child_frame
            '-0.1', '0', '-0.2', '0', '0', '0', 'base_link', 'lidar_body'
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

    # obstacle_detector standalone node
    obstacle_detector_node = Node(
        package='lidar_detection',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[{
            'lidar_points_topic': '/livox/lidar', 
            'odom_topic': '/Odometry',
            'cloud_ground_topic': '/cloud_ground',
            'cloud_clusters_topic': '/cloud_clusters',
            'marker_bboxes_topic': '/marker_bboxes',
            'objects_topic': '/detected_objects',
            'bbox_target_frame': 'lidar_body',
            'min_x': -0.01, 'max_x': 0.01,
            'min_y': -0.01, 'max_y': 0.01,
            'min_z': -0.01, 'max_z': 0.01,
            'robot_radius': 0.2, # robot 半径
        }]
    )

    # odom_trans standalone node
    odom_trans_node = Node(
        package='lidar_detection',
        executable='odom_trans_node',
        name='odom_trans_node',
        output='screen',
        parameters=[{
            'input_odom_topic': '/Odometry',
            'output_odom_topic': '/odom_base_link',
            'source_frame': 'lidar_body',
            'target_frame': 'base_link',
        }]
    )

    # obstacle_to_baselink standalone node
    obstacle_to_baselink_node = Node(
        package='lidar_detection',
        executable='obstacle_to_baselink_node',
        name='obstacle_to_baselink_node',
        output='screen',
        parameters=[{
            'obstacle_lidar_topic': '/detected_objects',
            'obstacle_baselink_topic': '/obstacle_information_in_baselink',
            'source_frame': 'lidar_body',
            'target_frame': 'base_link',
        }]
    )

    # InterfaceNode
    interface_node = Node(
        package='lidar_detection',
        executable='interface_node',
        name='interface_node',
        output='screen',
        parameters=[{
            'input_topic_odom': '/odom_base_link',
            'output_topic_robot_state': '/robot_state',
            'input_relative_ball_topic': '/obstacle_information_in_baselink',
            'output_relative_ball_topic': '/ball_relative_topic',
            'input_map_ball_topic': '/obstacle_information_in_baselink',
            'output_map_ball_topic': '/ball_map_topic',
            'source_frame': 'base_link',
            'target_frame': 'base_link',
            'robot_radius': 0.5, # robot 半径
        }]
    )

    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )

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
        period=2.0,
        actions=[
            odom_trans_node,
            obstacle_to_baselink_node]
    )
    delayed_interface_node2 = TimerAction(
        period=2.0,
        actions=[
            interface_node,
            # rqt_reconfigure_node
            ]
    )
    return LaunchDescription([
        static_tf_lidar_to_base,
        static_tf_odom_to_map,
        obstacle_detector_node,
        delayed_interface_node1,
        delayed_interface_node2,
        rviz_node,
    ])
