#!/usr/bin/env python3

import math
import struct
import time
import os
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as MsgDuration

from config import Config
from filter import height_filter, remove_ground_points, statistical_outlier_filter
from sort import SORT, Detection3D
from utils.misc import pointcloud2_to_array
from utils.viz import create_colored_pointcloud_msg, create_pointcloud_msg, get_color_for_id
from sklearn.cluster import DBSCAN

if os.getenv('DEBUG_MODE') == '1':
    import debugpy
    debugpy.listen(5678)
    print("Waiting for debugger to attach on port 5678...")
    debugpy.wait_for_client()
    print("Debugger attached!")

class Lidar3DDetectionNode(Node):
    """3D LiDAR object detection and tracking node for ROS 2"""

    def __init__(self):
        super().__init__("lidar_3d_detection_node")

        # Load configuration
        self.config = Config()

        # Create subscriber (Using sensor_msgs QoS for LiDAR)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/livox/lidar",
            self.pointcloud_callback,
            qos_profile_sensor_data
        )

        # Create publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/detection_markers", 10)
        self.track_pub = self.create_publisher(MarkerArray, "/tracking_markers", 10)

        # Debug publishers
        self.debug_raw_pc_pub = self.create_publisher(PointCloud2, "/debug/raw_pointcloud", 1)
        self.debug_statistical_filtered_pc_pub = self.create_publisher(PointCloud2, "/debug/statistical_filtered_pointcloud", 1)
        self.debug_filtered_pc_pub = self.create_publisher(PointCloud2, "/debug/filtered_pointcloud", 1)
        self.debug_ground_removed_pc_pub = self.create_publisher(PointCloud2, "/debug/ground_removed_pointcloud", 1)
        self.debug_height_filtered_pc_pub = self.create_publisher(PointCloud2, "/debug/height_filtered_pointcloud", 1)
        self.debug_clustered_pc_pub = self.create_publisher(PointCloud2, "/debug/clustered_pointcloud", 1)

        # SORT tracker
        self.tracker = SORT(
            max_age=self.config.tracking.max_age,
            min_hits=self.config.tracking.min_hits,
            distance_threshold=self.config.tracking.distance_threshold,
            use_iou=self.config.tracking.use_iou,
            iou_threshold=self.config.tracking.iou_threshold,
            enable_motion_analysis=self.config.tracking.enable_motion_analysis,
            kalman_params=self.config.kalman_filter,
            track_params=self.config.track,
        )

        self.get_logger().info("Lidar 3D Detection Node Started")

    def pointcloud_callback(self, msg):
        """Point cloud callback function"""
        try:
            # Convert point cloud data
            points = pointcloud2_to_array(msg)
            if points is None or points.size == 0:
                return

            self.debug_raw_pc_pub.publish(msg)

            # Offset points
            points[:, 2] += self.config.filtering.height_offset

            # Height filtering
            height_filtered = height_filter(points, self.config.filtering.min_height, self.config.filtering.max_height)
            if height_filtered.size == 0:
                return
            
            height_filtered_pc_msg = create_pointcloud_msg(height_filtered, msg.header.frame_id, msg.header.stamp)
            if height_filtered_pc_msg:
                self.debug_height_filtered_pc_pub.publish(height_filtered_pc_msg)

            # Statistical outlier filter
            statistical_filtered = self.preprocessing_statistical_outlier_filter(height_filtered)
            if statistical_filtered.size == 0:
                return
            
            statistical_filtered_pc_msg = create_pointcloud_msg(statistical_filtered, msg.header.frame_id, msg.header.stamp)
            if statistical_filtered_pc_msg:
                self.debug_statistical_filtered_pc_pub.publish(statistical_filtered_pc_msg)

            # DBSCAN clustering
            detections, clustered_points = self.dbscan_clustering(statistical_filtered)
            if len(clustered_points) > 0:
                clustered_pc_msg = create_colored_pointcloud_msg(clustered_points, msg.header.frame_id, msg.header.stamp)
                if clustered_pc_msg:
                    self.debug_clustered_pc_pub.publish(clustered_pc_msg)

            # SORT tracking
            tracked_objects = self.tracker.update(detections)
            self.publish_detections(detections, msg.header)
            self.publish_tracking_results(tracked_objects, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud callback: {str(e)}")

    def preprocessing_statistical_outlier_filter(self, points):
        """Apply statistical outlier filter using Open3D"""
        if (not self.config.preprocessing_statistical_filter.enable_preprocessing_statistical_filter 
            or len(points) < self.config.preprocessing_statistical_filter.preprocessing_statistical_nb_neighbors):
            return points

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            filtered_pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=self.config.preprocessing_statistical_filter.preprocessing_statistical_nb_neighbors,
                std_ratio=self.config.preprocessing_statistical_filter.preprocessing_statistical_std_ratio,
            )

            filtered_points = np.asarray(filtered_pcd.points, dtype=np.float32)
            self.get_logger().debug(f"Filtered: {len(points)} -> {len(filtered_points)}")
            return filtered_points
        except Exception as e:
            self.get_logger().warn(f"Statistical filter failed: {str(e)}")
            return points

    def dbscan_clustering(self, points):
        """DBSCAN clustering logic"""
        if len(points) < self.config.clustering.dbscan_min_samples:
            return [], []

        db = DBSCAN(eps=self.config.clustering.dbscan_eps, min_samples=self.config.clustering.dbscan_min_samples)
        cluster_labels = db.fit_predict(points)

        detections = []
        clustered_points = []
        unique_labels = set(cluster_labels)

        for label in unique_labels:
            if label == -1: continue

            cluster_points = points[cluster_labels == label]
            
            # Sub-filter for clusters
            filtered_cluster_points = statistical_outlier_filter(
                cluster_points,
                self.config.statistical_filter.statistical_nb_neighbors,
                self.config.statistical_filter.statistical_std_ratio,
                self.config.statistical_filter.enable_statistical_filter,
            )

            if not (self.config.clustering.min_cluster_size <= len(filtered_cluster_points) <= self.config.clustering.max_cluster_size):
                continue

            min_coords = np.min(filtered_cluster_points, axis=0)
            max_coords = np.max(filtered_cluster_points, axis=0)
            center = np.mean(filtered_cluster_points, axis=0)

            bbox = [min_coords[0], min_coords[1], min_coords[2], max_coords[0], max_coords[1], max_coords[2]]
            detection = Detection3D(bbox, center.tolist())
            detections.append(detection)
            clustered_points.append((cluster_points, detection))

        return detections, clustered_points

    def publish_detections(self, detections, header):
        """Publish detection markers"""
        marker_array = MarkerArray()
        for i, det in enumerate(detections):
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(det.center[0])
            marker.pose.position.y = float(det.center[1])
            marker.pose.position.z = float(det.center[2])
            marker.scale.x = float(det.bbox[3] - det.bbox[0])
            marker.scale.y = float(det.bbox[4] - det.bbox[1])
            marker.scale.z = float(det.bbox[5] - det.bbox[2])
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.5
            marker.lifetime = Duration(seconds=1.0).to_msg()
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_tracking_results(self, tracked_objects, header):
        """Publish tracking markers with velocity arrows and IDs"""
        marker_array = MarkerArray()
        for obj in tracked_objects:
            if len(obj) < 8: continue
            
            is_moving = bool(obj[7])
            if self.tracker.enable_motion_analysis and not is_moving: continue

            track_id = int(obj[3])
            color = get_color_for_id(track_id)

            # Box Marker
            box = Marker()
            box.header = header
            box.ns = "tracking"
            box.id = track_id * 3
            box.type = Marker.CUBE
            box.pose.position.x, box.pose.position.y, box.pose.position.z = float(obj[0]), float(obj[1]), float(obj[2])
            box.scale.x, box.scale.y, box.scale.z = 0.5, 0.5, 1.5
            box.color.r, box.color.g, box.color.b, box.color.a = color[0], color[1], color[2], 0.6
            box.lifetime = Duration(seconds=1.0).to_msg()
            marker_array.markers.append(box)

            # Velocity Arrow
            vx, vy, vz = float(obj[4]), float(obj[5]), float(obj[6])
            v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
            if v_mag > 0.1:
                arrow = Marker()
                arrow.header = header
                arrow.ns = "tracking_velocity"
                arrow.id = track_id * 3 + 1
                arrow.type = Marker.ARROW
                arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = float(obj[0]), float(obj[1]), float(obj[2]) + 0.75
                
                yaw = math.atan2(vy, vx)
                pitch = math.atan2(vz, np.sqrt(vx**2 + vy**2))
                arrow.pose.orientation.y = math.sin(pitch / 2)
                arrow.pose.orientation.z = math.sin(yaw / 2) * math.cos(pitch / 2)
                arrow.pose.orientation.w = math.cos(yaw / 2) * math.cos(pitch / 2)
                
                arrow.scale.x = min(v_mag * 2, 2.0)
                arrow.scale.y, arrow.scale.z = 0.1, 0.1
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = color[0], color[1], color[2], 0.9
                arrow.lifetime = Duration(seconds=1.0).to_msg()
                marker_array.markers.append(arrow)

            # ID Text
            text = Marker()
            text.header = header
            text.ns = "tracking_text"
            text.id = track_id * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x, text.pose.position.y, text.pose.position.z = float(obj[0]), float(obj[1]), float(obj[2]) + 1.0
            text.scale.z = 0.3
            text.color.r, text.color.g, text.color.b, text.color.a = color[0], color[1], color[2], 1.0
            text.text = f"ID: {track_id}\nV: {v_mag:.1f}m/s"
            text.lifetime = Duration(seconds=1.0).to_msg()
            marker_array.markers.append(text)

        self.track_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()