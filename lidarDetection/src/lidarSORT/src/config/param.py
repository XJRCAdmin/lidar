from dataclasses import dataclass


@dataclass
class FilteringParams:
    """Parameters for point cloud filtering"""
    ground_threshold: float = 0.0
    min_height: float = 0.1
    max_height: float = 1.9
    height_offset: float = 0.3  # Offset to compensate robot height


@dataclass
class ClusteringParams:
    """Parameters for DBSCAN clustering"""
    dbscan_eps: float = 0.3
    dbscan_min_samples: int = 10
    min_cluster_size: int = 50
    max_cluster_size: int = 1000


@dataclass
class StatisticalFilterParams:
    """Parameters for statistical outlier filtering"""
    enable_statistical_filter: bool = False
    statistical_nb_neighbors: int = 20
    statistical_std_ratio: float = 2.0


@dataclass
class PreprocessingStatisticalFilterParams:
    """Parameters for preprocessing statistical outlier filtering"""
    enable_preprocessing_statistical_filter: bool = True
    preprocessing_statistical_nb_neighbors: int = 30
    preprocessing_statistical_std_ratio: float = 0.25


@dataclass
class KalmanFilterParams:
    """Parameters for Kalman filter"""
    dt: float = 0.1  # Time interval
    process_noise: float = 0.1  # Process noise
    observation_noise: float = 1.0  # Observation noise
    initial_covariance: float = 1000.0  # Initial covariance


@dataclass
class TrackParams:
    """Parameters for individual track management"""
    max_position_history: int = 20
    max_velocity_history: int = 10
    min_history_for_motion: int = 5
    min_velocity_history_for_analysis: int = 3
    dt: float = 0.1  # Time interval in seconds
    min_movement_threshold: float = 0.05  # m/s
    position_variance_threshold: float = 0.01  # Position variance threshold
    velocity_consistency_threshold: float = 0.3  # Velocity consistency threshold
    stationary_confirmation_frames: int = 5  # Frames to confirm stationary


@dataclass
class TrackingParams:
    """Parameters for SORT tracking"""
    max_age: int = 10
    min_hits: int = 2
    distance_threshold: float = 2.0
    use_iou: bool = False
    iou_threshold: float = 0.3
    enable_motion_analysis: bool = True


@dataclass
class Config:
    """Main configuration class containing all parameters"""
    filtering: FilteringParams = FilteringParams()
    clustering: ClusteringParams = ClusteringParams()
    statistical_filter: StatisticalFilterParams = StatisticalFilterParams()
    preprocessing_statistical_filter: PreprocessingStatisticalFilterParams = PreprocessingStatisticalFilterParams()
    kalman_filter: KalmanFilterParams = KalmanFilterParams()
    track: TrackParams = TrackParams()
    tracking: TrackingParams = TrackingParams()