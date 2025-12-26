import numpy as np
from sensor_msgs_py import point_cloud2 as pc2


def pointcloud2_to_array(cloud_msg):
    """Convert PointCloud2 message to numpy array"""
    points_list = []
    for point in pc2.read_points(cloud_msg, skip_nans=True):
        points_list.append([point[0], point[1], point[2]])
    return np.array(points_list, dtype=np.float32)