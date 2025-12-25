import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# ROS2 采用.launch.py 文件来启动多个节点和配置参数,居多,不同于ROS1
def generate_launch_description():
    lidar_detection_pkg_share = get_package_share_directory('m_detector')

    config_file = os.path.join(
        lidar_detection_pkg_share,
        'config',
        'mid360',
        'mid360.yaml'
    )
    
    # 下面这几个DeclareLaunchArgument的参数在launch中进行修改
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='true',
        description='Whether to launch rviz'
    )
    
    time_file_arg = DeclareLaunchArgument(
        'time_file',
        default_value='',
        description='Time file path'
    )
    
    out_path_arg = DeclareLaunchArgument(
        'out_path',
        default_value=[os.path.join(lidar_detection_pkg_share, 'output', 'dynfilter_out.txt')],
        description='Output path'
    )
    
    out_origin_path_arg = DeclareLaunchArgument(
        'out_origin_path',
        default_value='test',
        description='Origin output path'
    )
    
    pose_log_arg = DeclareLaunchArgument(
        'pose_log',
        default_value='false',
        description='Whether to log pose'
    )
    
    pose_log_file_arg = DeclareLaunchArgument(
        'pose_log_file',
        default_value='',
        description='Pose log file path'
    )
    
    cluster_out_file_arg = DeclareLaunchArgument(
        'cluster_out_file',
        default_value='',
        description='Cluster output file path'
    )
    
    time_breakdown_file_arg = DeclareLaunchArgument(
        'time_breakdown_file',
        default_value='',
        description='Time breakdown file path'
    )
    
    # display node
    pred_file_arg = DeclareLaunchArgument(
        'pred_file',
        default_value='output',
        description='Prediction file path'
    )
    pc_file_arg = DeclareLaunchArgument(
        'pc_file',
        default_value='',
        description='Point cloud file path'
    )

    pc_topic_arg = DeclareLaunchArgument(
        'pc_topic',
        default_value='',
        description='Point cloud topic'
    )

    display_prediction_node = Node(
        package='m_detector',
        executable='display_prediction',
        name='display_prediction',
        output='screen',
        parameters=[
            {
                'pred_file': LaunchConfiguration('pred_file'),
                'pc_file': LaunchConfiguration('pc_file'),
                'pc_topic': LaunchConfiguration('pc_topic'),
            }
        ]
    )

    # Create dynfilter node with parameters
    dynfilter_node = Node(
        package='m_detector',
        executable='dynfilter',
        name='dynfilter',
        output='screen',
        parameters=[
            config_file,
            {
                'dyn_obj/out_file': LaunchConfiguration('out_path'),
                'dyn_obj/out_file_origin': LaunchConfiguration('out_origin_path'),
                'dyn_obj/time_file': LaunchConfiguration('time_file'),
                'dyn_obj/pose_log': LaunchConfiguration('pose_log'),
                'dyn_obj/pose_log_file': LaunchConfiguration('pose_log_file'),
                'dyn_obj/cluster_out_file': LaunchConfiguration('cluster_out_file'),
                'dyn_obj/time_breakdown_file': LaunchConfiguration('time_breakdown_file')
            }
        ]
    )
    
    # Create rviz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(lidar_detection_pkg_share, 'rviz', 'm_detector.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(rviz_arg)
    ld.add_action(time_file_arg)
    ld.add_action(out_path_arg)
    ld.add_action(out_origin_path_arg)
    ld.add_action(pose_log_arg)
    ld.add_action(pose_log_file_arg)
    ld.add_action(cluster_out_file_arg)
    ld.add_action(time_breakdown_file_arg)
    ld.add_action(pred_file_arg)
    ld.add_action(pc_file_arg)
    ld.add_action(pc_topic_arg)
    
    # Add nodes
    ld.add_action(display_prediction_node)
    ld.add_action(dynfilter_node)
    ld.add_action(rviz_node)
    
    return ld