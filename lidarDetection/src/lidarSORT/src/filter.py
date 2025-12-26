import numpy as np
import open3d as o3d
import rclpy
from rclpy.logging import get_logger

def remove_ground_points(points, ground_threshold=0.0):
    """Remove ground points using height threshold"""
    return points[points[:, 2] > ground_threshold]


def height_filter(points, min_height, max_height):
    """Filter points by height range"""
    mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    return points[mask]


def statistical_outlier_filter(points, nb_neighbors=20, std_ratio=2.0, enable_filter=True, logger=None):
    """Apply statistical outlier filter to remove noise points"""
    if not enable_filter or len(points) < nb_neighbors:
        return points

    try:
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Apply statistical outlier removal
        filtered_pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

        # Convert back to numpy array
        filtered_points = np.asarray(filtered_pcd.points, dtype=np.float32)
        return filtered_points

    except Exception as e:
        log_msg = f"Statistical outlier filter failed: {str(e)}"
        if logger:
            logger.warn(log_msg)
        else:
            get_logger("point_cloud_utils").warn(log_msg)
        return points