import numpy as np
from scipy.optimize import linear_sum_assignment
# 注意：在 ROS 2 中通常不在逻辑类内部直接 import rclpy，而是通过参数传入时间

class Detection3D:
    """3D 检测结果类"""
    def __init__(self, bbox, center, id=-1, current_time=0.0):
        self.bbox = bbox  # [x_min, y_min, z_min, x_max, y_max, z_max]
        self.center = center  # [x, y, z]
        self.id = id
        # 使用 ROS 2 传入的时间戳
        self.timestamp = current_time
        self.birth_time = current_time
        self.last_seen = current_time
        self.confidence = 1.0
        self.is_alive = True

class KalmanFilter:
    """用于对象状态预测的卡尔曼滤波器"""
    def __init__(self, initial_state, params=None):
        from config import KalmanFilterParams
        if params is None:
            params = KalmanFilterParams()

        # 状态向量 [x, y, z, vx, vy, vz]
        self.state = np.array(initial_state, dtype=np.float32).reshape(-1, 1)
        self.P = np.eye(6) * params.initial_covariance 

        dt = params.dt
        # 状态转移矩阵
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)

        self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]], dtype=np.float32)
        self.Q = np.eye(6) * params.process_noise
        self.R = np.eye(3) * params.observation_noise

    def predict(self):
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.state[:3].flatten()

    def update(self, measurement):
        z = np.array(measurement, dtype=np.float32).reshape(-1, 1)
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

class Track:
    """跟踪目标类"""
    def __init__(self, detection, track_id, params=None, kalman_params=None, current_time=0.0):
        from config import TrackParams, KalmanFilterParams
        self.params = params if params is not None else TrackParams()
        self.id = track_id
        self.kalman = KalmanFilter(
            [detection.center[0], detection.center[1], detection.center[2], 0, 0, 0], 
            kalman_params
        )
        self.time_since_update = 0
        self.hit_streak = 1
        self.age = 1
        self.history = []
        
        # 使用 ROS 2 时间
        self.birth_time = current_time
        self.death_time = None
        self.is_alive = True
        self.confidence = detection.confidence
        
        self.position_history = [detection.center]
        self.velocity_history = []
        self.is_moving = False
        self.stationary_count = 0
        self.motion_consistency_score = 0.0

    def predict(self):
        return self.kalman.predict()

    def update(self, detection, current_time):
        self.kalman.update(detection.center)
        self.time_since_update = 0
        self.hit_streak += 1
        self.history.append(detection)
        self.confidence = detection.confidence
        detection.last_seen = current_time

        self.position_history.append(detection.center)
        if len(self.position_history) > self.params.max_position_history:
            self.position_history.pop(0)

        if len(self.position_history) >= 2:
            dt = self.params.dt
            velocity = (np.array(self.position_history[-1]) - np.array(self.position_history[-2])) / dt
            self.velocity_history.append(velocity)
            if len(self.velocity_history) > self.params.max_velocity_history:
                self.velocity_history.pop(0)

        self._analyze_motion_pattern()

    def _analyze_motion_pattern(self):
        """分析运动模式，确定物体是否在移动"""
        if len(self.position_history) < self.params.min_history_for_motion:
            self.is_moving = False
            return

        positions = np.array(self.position_history)
        position_var = np.sum(np.var(positions, axis=0))

        velocity_consistency = 0.0
        if len(self.velocity_history) >= self.params.min_velocity_history_for_analysis:
            velocities = np.array(self.velocity_history)
            mags = np.linalg.norm(velocities, axis=1)
            avg_mag = np.mean(mags)
            
            if avg_mag > self.params.min_movement_threshold:
                # 简化的方向一致性计算
                dirs = velocities / (mags[:, None] + 1e-6)
                consistency = np.mean([np.dot(dirs[i-1], dirs[i]) for i in range(1, len(dirs))])
                velocity_consistency = max(0, consistency)

        if position_var > self.params.position_variance_threshold and \
           velocity_consistency > self.params.velocity_consistency_threshold:
            self.is_moving = True
            self.stationary_count = 0
        else:
            self.stationary_count += 1
            if self.stationary_count >= self.params.stationary_confirmation_frames:
                self.is_moving = False

    def mark_death(self, current_time):
        self.death_time = current_time
        self.is_alive = False

    def get_lifetime(self, current_time):
        end_time = self.death_time if self.death_time else current_time
        return end_time - self.birth_time

class SORT:
    """SORT 跟踪算法 ROS 2 适配版"""
    def __init__(self, max_age=5, min_hits=3, **kwargs):
        from config import KalmanFilterParams, TrackParams
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = kwargs.get('distance_threshold', 2.0)
        self.use_iou = kwargs.get('use_iou', True)
        self.iou_threshold = kwargs.get('iou_threshold', 0.3)
        self.enable_motion_analysis = kwargs.get('enable_motion_analysis', True)
        
        self.kalman_params = kwargs.get('kalman_params', KalmanFilterParams())
        self.track_params = kwargs.get('track_params', TrackParams())
        
        self.tracks = []
        self.track_count = 0
        self.dead_tracks = []

    def update(self, detections, current_time):
        """
        更新跟踪器
        :param detections: Detection3D 列表
        :param current_time: 当前 ROS 2 节点时间（秒）
        """
        for track in self.tracks:
            track.predict()

        matched, unmatched_dets, unmatched_trks = self.associate_detections_to_trackers_hungarian(
            detections, self.tracks
        )

        # 更新匹配成功的轨迹
        for m in matched:
            self.tracks[m[1]].update(detections[m[0]], current_time)

        # 为未匹配的检测创建新轨迹
        for i in unmatched_dets:
            trk = Track(detections[i], self.track_count, self.track_params, self.kalman_params, current_time)
            self.track_count += 1
            self.tracks.append(trk)

        # 清理死亡轨迹
        for i in reversed(range(len(self.tracks))):
            trk = self.tracks[i]
            if trk.time_since_update > self.max_age:
                trk.mark_death(current_time)
                self.dead_tracks.append(trk)
                self.tracks.pop(i)
            else:
                trk.time_since_update += 1
                trk.age += 1

        # 返回当前有效轨迹
        ret = []
        for trk in self.tracks:
            if trk.time_since_update <= 1 and (trk.hit_streak >= self.min_hits or trk.age <= self.min_hits):
                pos = trk.kalman.state[:3].flatten()
                vel = trk.kalman.state[3:6].flatten()
                is_moving = trk.is_moving if self.enable_motion_analysis else True
                ret.append([pos[0], pos[1], pos[2], trk.id, vel[0], vel[1], vel[2], is_moving])

        return np.array(ret) if ret else np.empty((0, 8))

    def associate_detections_to_trackers_hungarian(self, detections, trackers):
        # 保持匈牙利算法逻辑不变，仅确保内部 bbox 计算正确
        if len(trackers) == 0:
            return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0,), dtype=int)

        dist_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
        for d, det in enumerate(detections):
            for t, trk in enumerate(trackers):
                if self.use_iou:
                    iou = self.calculate_iou_3d(det.bbox, self.get_predicted_bbox(trk))
                    dist_matrix[d, t] = 1.0 - iou
                else:
                    dist_matrix[d, t] = np.linalg.norm(np.array(det.center) - trk.kalman.state[:3].flatten())

        threshold = (1.0 - self.iou_threshold) if self.use_iou else self.distance_threshold
        row_ind, col_ind = linear_sum_assignment(dist_matrix)

        matched = []
        for r, c in zip(row_ind, col_ind):
            if dist_matrix[r, c] <= threshold:
                matched.append([r, c])

        matched = np.array(matched) if matched else np.empty((0, 2), dtype=int)
        unmatched_dets = [i for i in range(len(detections)) if not any(matched[:, 0] == i)]
        unmatched_trks = [i for i in range(len(trackers)) if not any(matched[:, 1] == i)]
        
        return matched, unmatched_dets, unmatched_trks

    def calculate_iou_3d(self, bbox1, bbox2):
        # 保持 3D IoU 计算逻辑
        x_min = max(bbox1[0], bbox2[0]); x_max = min(bbox1[3], bbox2[3])
        y_min = max(bbox1[1], bbox2[1]); y_max = min(bbox1[4], bbox2[4])
        z_min = max(bbox1[2], bbox2[2]); z_max = min(bbox1[5], bbox2[5])
        
        if x_min >= x_max or y_min >= y_max or z_min >= z_max: return 0.0
        
        inter = (x_max - x_min) * (y_max - y_min) * (z_max - z_min)
        vol1 = (bbox1[3]-bbox1[0]) * (bbox1[4]-bbox1[1]) * (bbox1[5]-bbox1[2])
        vol2 = (bbox2[3]-bbox2[0]) * (bbox2[4]-bbox2[1]) * (bbox2[5]-bbox2[2])
        return inter / (vol1 + vol2 - inter + 1e-6)

    def get_predicted_bbox(self, track):
        pos = track.kalman.state[:3].flatten()
        if track.history:
            b = track.history[-1].bbox
            w, h, d = b[3]-b[0], b[4]-b[1], b[5]-b[2]
        else: w = h = d = 0.5
        return [pos[0]-w/2, pos[1]-h/2, pos[2]-d/2, pos[0]+w/2, pos[1]+h/2, pos[2]+d/2]

    def associate_detections_to_trackers_hungarian(self, detections, trackers):
        """Associate detections to trackers using Hungarian algorithm"""
        if len(trackers) == 0:
            return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0,), dtype=int)
        if len(detections) == 0:
            return np.empty((0, 2), dtype=int), np.empty((0,), dtype=int), np.arange(len(trackers))

        # Compute distance matrix based on method selection
        if self.use_iou:
            # IoU-based distance matrix (1 - IoU as distance)
            distance_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
            for d, det in enumerate(detections):
                for t, trk in enumerate(trackers):
                    pred_bbox = self.get_predicted_bbox(trk)
                    iou = self.calculate_iou_3d(det.bbox, pred_bbox)
                    distance_matrix[d, t] = 1.0 - iou  # Convert IoU to distance

            # Use IoU threshold for filtering
            threshold = 1.0 - self.iou_threshold  # Convert IoU threshold to distance threshold
        else:
            # Euclidean distance matrix
            distance_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
            for d, det in enumerate(detections):
                for t, trk in enumerate(trackers):
                    trk_pos = trk.kalman.state[:3].flatten()
                    distance_matrix[d, t] = np.linalg.norm(np.array(det.center) - trk_pos)

            threshold = self.distance_threshold

        # Use Hungarian algorithm for optimal assignment
        if distance_matrix.size > 0:
            row_indices, col_indices = linear_sum_assignment(distance_matrix)

            # Filter out assignments with distance > threshold
            matched_indices = []
            for r, c in zip(row_indices, col_indices):
                if distance_matrix[r, c] <= threshold:
                    matched_indices.append([r, c])

            matched_indices = np.array(matched_indices) if matched_indices else np.empty((0, 2), dtype=int)

            # Find unmatched detections and trackers
            unmatched_detections = []
            for d in range(len(detections)):
                if not any(matched[0] == d for matched in matched_indices):
                    unmatched_detections.append(d)

            unmatched_trackers = []
            for t in range(len(trackers)):
                if not any(matched[1] == t for matched in matched_indices):
                    unmatched_trackers.append(t)
        else:
            matched_indices = np.empty((0, 2), dtype=int)
            unmatched_detections = list(range(len(detections)))
            unmatched_trackers = list(range(len(trackers)))

        return matched_indices, unmatched_detections, unmatched_trackers

    def get_track_statistics(self):
        """Get tracking statistics"""
        stats = {
            "active_tracks": len(self.tracks),
            "total_tracks_created": self.track_count,
            "dead_tracks": len(self.dead_tracks),
            "average_lifetime": 0.0,
        }

        if self.dead_tracks:
            lifetimes = [track.get_lifetime() for track in self.dead_tracks]
            stats["average_lifetime"] = np.mean(lifetimes)

        return stats
