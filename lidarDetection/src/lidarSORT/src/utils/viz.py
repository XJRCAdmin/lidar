import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

def create_pointcloud_msg(points, frame_id, timestamp):
    """Convert numpy array to PointCloud2 message for ROS 2"""
    if len(points) == 0:
        return None

    # Create header
    header = Header()
    header.stamp = timestamp # 确保传入的是 node.get_clock().now().to_msg()
    header.frame_id = frame_id

    # Create point cloud message
    # ROS 2 中 points 可以直接是 numpy array 或 list
    pc_msg = pc2.create_cloud_xyz32(header, points.tolist())
    return pc_msg


def create_colored_pointcloud_msg(clustered_points, frame_id, timestamp):
    """Create colored point cloud message for clustered points in ROS 2"""
    if len(clustered_points) == 0:
        return None

    color_palette = [
        [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0],
        [1.0, 0.5, 0.0], [0.5, 0.0, 1.0], [0.0, 0.5, 1.0], [1.0, 0.0, 0.5],
    ]

    all_cluster_points = []
    colors = []

    for i, (cluster_points, _) in enumerate(clustered_points):
        color = color_palette[i % len(color_palette)]
        all_cluster_points.extend(cluster_points.tolist())
        for _ in range(len(cluster_points)):
            colors.extend(color)

    if not all_cluster_points:
        return None

    points_array = np.array(all_cluster_points)
    colors_array = np.array(colors).reshape(-1, 3)

    # ROS 2 PointField 定义
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]

    header = Header()
    header.stamp = timestamp
    header.frame_id = frame_id

    cloud_data = []
    for i in range(len(points_array)):
        x, y, z = points_array[i]
        r, g, b = colors_array[i]

        # Pack RGB values (Standard PCL format: B-G-R-A)
        rgb_packed = struct.unpack("I", struct.pack("BBBB", int(b * 255), int(g * 255), int(r * 255), 255))[0]
        cloud_data.append([x, y, z, rgb_packed])

    # Create colored point cloud message
    clustered_pc_msg = pc2.create_cloud(header, fields, cloud_data)
    return clustered_pc_msg


def get_color_for_id(tracker_id):
    """Get consistent color for tracker ID (Remains same as ROS 1)"""
    color_palette = [
        [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0],
        [1.0, 0.5, 0.0], [0.5, 0.0, 1.0], [0.0, 0.5, 1.0],
        [1.0, 0.0, 0.5], [0.5, 1.0, 0.0], [1.0, 0.5, 0.5],
        [0.5, 0.5, 1.0], [0.8, 0.8, 0.0], [0.8, 0.0, 0.8],
    ]
    return color_palette[tracker_id % len(color_palette)]