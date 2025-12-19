#!/usr/bin/env python3
# filepath: /home/robocon/go2/lidarDetection/src/lidarDetection/scripts/reconfigure.py
"""
Obstacle Detector 参数动态调节工具 (PyQt5版本)
使用方法: ros2 run lidar_detection reconfigure.py
或者: python3 reconfigure.py
"""

import sys
import threading
from typing import Dict, Any, Tuple, Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox,
    QGroupBox, QScrollArea, QPushButton, QStatusBar, QMessageBox,
    QSlider, QFrame, QSplitter, QToolButton, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor

# 参数定义: (类型, 默认值, 最小值, 最大值, 描述)
PARAM_DEFINITIONS = {
    # 安装位姿参数
    "rotate_x": ("double", 0.0, -10.0, 10.0, "X轴平移 (米) - LiDAR相对于robot的前后位置"),
    "rotate_y": ("double", 0.0, -10.0, 10.0, "Y轴平移 (米) - LiDAR相对于robot的左右位置"),
    "rotate_z": ("double", 0.0, -10.0, 10.0, "Z轴平移 (米) - LiDAR相对于robot的高度"),
    "rotate_roll": ("double", 0.0, -3.14, 3.14, "横滚角 (弧度) - 绕X轴旋转"),
    "rotate_pitch": ("double", 0.0, -3.14, 3.14, "俯仰角 (弧度) - 绕Y轴旋转"),
    "rotate_yaw": ("double", 0.0, -3.14, 3.14, "偏航角 (弧度) - 绕Z轴旋转"),
    
    # 功能开关
    "use_pca_box": ("bool", True, None, None, "使用PCA边界框 (vs 轴对齐边界框)"),
    "use_tracking": ("bool", True, None, None, "启用目标跟踪功能"),
    
    # 点云下采样
    "voxel_grid_size": ("double", 0.2, 0.05, 1.0, "体素网格大小 (米) - 点云下采样分辨率"),
    
    # ROI区域设置
    "roi_max_x": ("double", 5.0, 0.0, 8.0, "point1前方 (米)"),
    "roi_max_y": ("double", 5.0, 0.0, 8.0, "point1右侧 (米)"),
    "roi_max_z": ("double", 2.0, 0.0, 3.0, "point1向上 (米)"),
    "roi_min_x": ("double", -4.0, -8.0, 0.0, "point2后方 (米) - 负值"),
    "roi_min_y": ("double", -4.0, -8.0, 0.0, "point2左侧 (米) - 负值"),
    "roi_min_z": ("double", -0.5, -1.0, 0.0, "point2向下 (米) - 负值"),
    
    # 地面分割
    "ground_segment": ("string", "RANSAC", None, None, "地面分割算法 (RANSAC)"),
    "ground_threshold": ("double", 0.3, 0.0, 1.0, "地面高度阈值 (米) - 地面平面的容差"),
    
    # 聚类参数
    "cluster_threshold": ("double", 0.6, 0.1, 3.0, "聚类距离阈值 (米) - 点间最大距离"),
    "cluster_max_size": ("int", 500, 10, 2000, "聚类最大点数 - 单个障碍物最大点数"),
    "cluster_min_size": ("int", 10, 3, 50, "聚类最小点数 - 过滤小噪声"),
    
    # 跟踪参数
    "displacement_threshold": ("double", 1.0, 0.1, 5.0, "位移阈值 (米) - 帧间最大移动距离"),
    "iou_threshold": ("double", 1.0, 0.1, 1.0, "IoU阈值 - 边界框重叠度匹配"),
    
    # 统计离群点滤波
    "enable_statistical_filter": ("bool", True, None, None, "启用统计离群点滤波"),
    "statistical_nb_neighbors": ("int", 20, 10, 100, "邻居点数量 - 统计分析的近邻点数"),
    "statistical_std_ratio": ("double", 1.0, 0.1, 4.0, "标准差倍数 - 离群点判定阈值 (越小越严格)"),
    
    # 边界框滤波
    "max_dimension_ratio": ("double", 5.0, 5.0, 10.0, "边界框最大长宽高比例（最长边/最短边）"),
    
    # 高度和范围滤波
    "enable_height_range_filter": ("bool", False, None, None, "启用高度和范围滤波"),
    "height_limit": ("double", 2.0, 0.5, 3.0, "最大高度限制 (米) - 过滤高空点云"),
    "range_length": ("double", 5.0, 0.5, 10.0, "前后范围 (米) - robot前后方向感兴趣距离"),
    "range_width": ("double", 5.0, 0.5, 10.0, "左右范围 (米) - robot左右方向感兴趣距离"),
}

# 参数分组
PARAM_GROUPS = {
    "安装位姿": ["rotate_x", "rotate_y", "rotate_z", "rotate_roll", "rotate_pitch", "rotate_yaw"],
    "功能开关": ["use_pca_box", "use_tracking"],
    "点云滤波": ["voxel_grid_size", "enable_statistical_filter", "statistical_nb_neighbors", "statistical_std_ratio"],
    "ROI区域": ["roi_max_x", "roi_max_y", "roi_max_z", "roi_min_x", "roi_min_y", "roi_min_z"],
    "地面分割": ["ground_segment", "ground_threshold"],
    "聚类参数": ["cluster_threshold", "cluster_max_size", "cluster_min_size"],
    "目标跟踪": ["displacement_threshold", "iou_threshold"],
    "高度范围滤波": ["enable_height_range_filter", "height_limit", "range_length", "range_width", "max_dimension_ratio"],
}


class RosSignals(QObject):
    """用于ROS回调和Qt GUI之间通信的信号"""
    param_updated = pyqtSignal(str, bool, str)  # param_name, success, message
    params_loaded = pyqtSignal(dict)  # all parameters


class ParamTunerNode(Node):
    def __init__(self, signals: RosSignals):
        super().__init__('param_tuner_qt')
        self.signals = signals
        self.target_node = '/obstacle_detector_node'
        
        self.set_param_client = self.create_client(
            SetParameters, f'{self.target_node}/set_parameters')
        self.get_param_client = self.create_client(
            GetParameters, f'{self.target_node}/get_parameters')
        
        self.get_logger().info(f'等待 {self.target_node} 节点...')
        
    def wait_for_service(self, timeout_sec=5.0) -> bool:
        return self.set_param_client.wait_for_service(timeout_sec)
    
    def set_parameter_async(self, name: str, value: Any):
        """异步设置参数"""
        if name not in PARAM_DEFINITIONS:
            self.signals.param_updated.emit(name, False, f'未知参数: {name}')
            return
        
        param_def = PARAM_DEFINITIONS[name]
        param_type, default, min_val, max_val, desc = param_def
        
        # 范围检查
        if param_type in ("double", "int") and min_val is not None and max_val is not None:
            if value < min_val or value > max_val:
                self.signals.param_updated.emit(name, False, f'值 {value} 超出范围 [{min_val}, {max_val}]')
                return
        
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        
        if param_type == "double":
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
        elif param_type == "int":
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        elif param_type == "bool":
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = bool(value)
        elif param_type == "string":
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = str(value)
        
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.set_param_client.call_async(request)
        future.add_done_callback(lambda f: self._on_set_param_done(f, name, value))
        
    def _on_set_param_done(self, future, name: str, value: Any):
        """设置参数完成回调"""
        try:
            result = future.result()
            if result and result.results and result.results[0].successful:
                self.signals.param_updated.emit(name, True, f'{name} = {value}')
            else:
                reason = result.results[0].reason if result and result.results else '未知错误'
                self.signals.param_updated.emit(name, False, reason)
        except Exception as e:
            self.signals.param_updated.emit(name, False, str(e))
    
    def get_all_parameters_async(self):
        """异步获取所有参数"""
        request = GetParameters.Request()
        request.names = list(PARAM_DEFINITIONS.keys())
        
        future = self.get_param_client.call_async(request)
        future.add_done_callback(self._on_get_params_done)
        
    def _on_get_params_done(self, future):
        """获取参数完成回调"""
        result = {}
        try:
            response = future.result()
            if response and response.values:
                names = list(PARAM_DEFINITIONS.keys())
                for i, value in enumerate(response.values):
                    if i < len(names):
                        name = names[i]
                        if value.type == ParameterType.PARAMETER_DOUBLE:
                            result[name] = value.double_value
                        elif value.type == ParameterType.PARAMETER_INTEGER:
                            result[name] = value.integer_value
                        elif value.type == ParameterType.PARAMETER_BOOL:
                            result[name] = value.bool_value
                        elif value.type == ParameterType.PARAMETER_STRING:
                            result[name] = value.string_value
                        else:
                            result[name] = PARAM_DEFINITIONS[name][1]
        except Exception as e:
            self.get_logger().error(f'获取参数失败: {e}')
        
        # 填充缺失的参数
        for name in PARAM_DEFINITIONS:
            if name not in result:
                result[name] = PARAM_DEFINITIONS[name][1]
        
        self.signals.params_loaded.emit(result)


class CollapsibleGroupBox(QWidget):
    """可折叠的分组框"""
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.is_collapsed = False
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 标题栏
        self.header = QFrame()
        self.header.setStyleSheet("""
            QFrame {
                background-color: #3d3d3d;
                border-radius: 4px;
                padding: 5px;
            }
        """)
        header_layout = QHBoxLayout(self.header)
        header_layout.setContentsMargins(10, 5, 10, 5)
        
        self.toggle_btn = QToolButton()
        self.toggle_btn.setArrowType(Qt.DownArrow)
        self.toggle_btn.setStyleSheet("QToolButton { border: none; }")
        self.toggle_btn.clicked.connect(self.toggle_content)
        
        self.title_label = QLabel(title)
        self.title_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.title_label.setStyleSheet("color: #ffffff;")
        
        header_layout.addWidget(self.toggle_btn)
        header_layout.addWidget(self.title_label)
        header_layout.addStretch()
        
        # 内容区域
        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(10, 5, 10, 10)
        self.content_layout.setSpacing(8)
        
        layout.addWidget(self.header)
        layout.addWidget(self.content)
        
        # 使标题栏可点击
        self.header.mousePressEvent = lambda e: self.toggle_content()
        
    def toggle_content(self):
        self.is_collapsed = not self.is_collapsed
        self.content.setVisible(not self.is_collapsed)
        self.toggle_btn.setArrowType(Qt.RightArrow if self.is_collapsed else Qt.DownArrow)
        
    def add_widget(self, widget: QWidget):
        self.content_layout.addWidget(widget)


class ParameterWidget(QWidget):
    """单个参数的控件"""
    value_changed = pyqtSignal(str, object)  # param_name, value
    
    def __init__(self, name: str, param_type: str, default: Any, 
                 min_val: Optional[float], max_val: Optional[float], 
                 description: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.param_type = param_type
        self._updating = False
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        
        # 参数名标签
        name_label = QLabel(name)
        name_label.setMinimumWidth(180)
        name_label.setToolTip(description)
        name_label.setStyleSheet("color: #cccccc;")
        layout.addWidget(name_label)
        
        if param_type == "bool":
            self.widget = QCheckBox()
            self.widget.setChecked(default)
            self.widget.stateChanged.connect(self._on_bool_changed)
            layout.addWidget(self.widget)
            layout.addStretch()
            
        elif param_type == "string":
            self.widget = QComboBox()
            if name == "ground_segment":
                self.widget.addItems(["RANSAC"])
            self.widget.setCurrentText(str(default))
            self.widget.currentTextChanged.connect(self._on_string_changed)
            self.widget.setMinimumWidth(120)
            layout.addWidget(self.widget)
            layout.addStretch()
            
        elif param_type == "double":
            # 滑块
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setMinimum(int(min_val * 100))
            self.slider.setMaximum(int(max_val * 100))
            self.slider.setValue(int(default * 100))
            self.slider.setMinimumWidth(200)
            self.slider.valueChanged.connect(self._on_slider_changed)
            
            # 数值输入
            self.widget = QDoubleSpinBox()
            self.widget.setRange(min_val, max_val)
            self.widget.setSingleStep(0.01)
            self.widget.setDecimals(3)
            self.widget.setValue(default)
            self.widget.setMinimumWidth(80)
            self.widget.valueChanged.connect(self._on_double_changed)
            
            # 范围标签
            range_label = QLabel(f"[{min_val}, {max_val}]")
            range_label.setStyleSheet("color: #888888; font-size: 10px;")
            
            layout.addWidget(self.slider)
            layout.addWidget(self.widget)
            layout.addWidget(range_label)
            
        elif param_type == "int":
            # 滑块
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setMinimum(int(min_val))
            self.slider.setMaximum(int(max_val))
            self.slider.setValue(int(default))
            self.slider.setMinimumWidth(200)
            self.slider.valueChanged.connect(self._on_int_slider_changed)
            
            # 数值输入
            self.widget = QSpinBox()
            self.widget.setRange(int(min_val), int(max_val))
            self.widget.setValue(int(default))
            self.widget.setMinimumWidth(80)
            self.widget.valueChanged.connect(self._on_int_changed)
            
            # 范围标签
            range_label = QLabel(f"[{int(min_val)}, {int(max_val)}]")
            range_label.setStyleSheet("color: #888888; font-size: 10px;")
            
            layout.addWidget(self.slider)
            layout.addWidget(self.widget)
            layout.addWidget(range_label)
    
    def _on_bool_changed(self, state):
        if not self._updating:
            self.value_changed.emit(self.name, state == Qt.Checked)
    
    def _on_string_changed(self, text):
        if not self._updating:
            self.value_changed.emit(self.name, text)
    
    def _on_slider_changed(self, value):
        if not self._updating:
            self._updating = True
            real_value = value / 100.0
            self.widget.setValue(real_value)
            self._updating = False
            self.value_changed.emit(self.name, real_value)
    
    def _on_double_changed(self, value):
        if not self._updating:
            self._updating = True
            self.slider.setValue(int(value * 100))
            self._updating = False
            self.value_changed.emit(self.name, value)
    
    def _on_int_slider_changed(self, value):
        if not self._updating:
            self._updating = True
            self.widget.setValue(value)
            self._updating = False
            self.value_changed.emit(self.name, value)
    
    def _on_int_changed(self, value):
        if not self._updating:
            self._updating = True
            self.slider.setValue(value)
            self._updating = False
            self.value_changed.emit(self.name, value)
    
    def set_value(self, value):
        """设置当前值（不触发信号）"""
        self._updating = True
        if self.param_type == "bool":
            self.widget.setChecked(bool(value))
        elif self.param_type == "string":
            self.widget.setCurrentText(str(value))
        elif self.param_type == "double":
            self.widget.setValue(float(value))
            self.slider.setValue(int(float(value) * 100))
        elif self.param_type == "int":
            self.widget.setValue(int(value))
            self.slider.setValue(int(value))
        self._updating = False
    
    def get_value(self):
        """获取当前值"""
        if self.param_type == "bool":
            return self.widget.isChecked()
        elif self.param_type == "string":
            return self.widget.currentText()
        elif self.param_type == "double":
            return self.widget.value()
        elif self.param_type == "int":
            return self.widget.value()


class MainWindow(QMainWindow):
    """主窗口"""
    def __init__(self, ros_node: ParamTunerNode, signals: RosSignals):
        super().__init__()
        self.ros_node = ros_node
        self.signals = signals
        self.param_widgets: Dict[str, ParameterWidget] = {}
        
        self.setWindowTitle("Obstacle Detector 参数调节面板")
        self.setMinimumSize(800, 600)
        self.resize(900, 750)
        
        # 设置深色主题
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QGroupBox {
                border: 1px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                color: #ffffff;
            }
            QPushButton {
                background-color: #0d6efd;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #0b5ed7;
            }
            QPushButton:pressed {
                background-color: #0a58ca;
            }
            QPushButton#resetBtn {
                background-color: #6c757d;
            }
            QPushButton#resetBtn:hover {
                background-color: #5c636a;
            }
            QDoubleSpinBox, QSpinBox, QComboBox {
                background-color: #3d3d3d;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 4px;
                color: #ffffff;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #555555;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #0d6efd;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
            QSlider::sub-page:horizontal {
                background: #0d6efd;
                border-radius: 3px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
            QCheckBox::indicator:unchecked {
                background-color: #3d3d3d;
                border: 2px solid #555555;
                border-radius: 3px;
            }
            QCheckBox::indicator:checked {
                background-color: #0d6efd;
                border: 2px solid #0d6efd;
                border-radius: 3px;
            }
            QScrollArea {
                border: none;
            }
            QStatusBar {
                background-color: #1e1e1e;
                color: #aaaaaa;
            }
        """)
        
        self._setup_ui()
        self._connect_signals()
        
        # 初始加载参数
        QTimer.singleShot(500, self._refresh_params)
        
    def _setup_ui(self):
        """设置UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 滚动区域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(10)
        
        # 创建参数分组
        for group_name, param_names in PARAM_GROUPS.items():
            group = CollapsibleGroupBox(group_name)
            
            for param_name in param_names:
                if param_name not in PARAM_DEFINITIONS:
                    continue
                    
                param_type, default, min_val, max_val, desc = PARAM_DEFINITIONS[param_name]
                widget = ParameterWidget(param_name, param_type, default, min_val, max_val, desc)
                widget.value_changed.connect(self._on_param_changed)
                group.add_widget(widget)
                self.param_widgets[param_name] = widget
            
            scroll_layout.addWidget(group)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        
        # 按钮栏
        button_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("🔄 刷新参数")
        refresh_btn.clicked.connect(self._refresh_params)
        
        reset_btn = QPushButton("↩️ 恢复默认值")
        reset_btn.setObjectName("resetBtn")
        reset_btn.clicked.connect(self._reset_to_defaults)
        
        apply_btn = QPushButton("✅ 应用全部")
        apply_btn.clicked.connect(self._apply_all)
        
        button_layout.addWidget(refresh_btn)
        button_layout.addWidget(reset_btn)
        button_layout.addStretch()
        button_layout.addWidget(apply_btn)
        
        main_layout.addLayout(button_layout)
        
        # 状态栏
        self.statusBar().showMessage("就绪 - 已连接到 obstacle_detector")
        
    def _connect_signals(self):
        """连接ROS信号"""
        self.signals.param_updated.connect(self._on_param_update_result)
        self.signals.params_loaded.connect(self._on_params_loaded)
        
    def _on_param_changed(self, name: str, value: Any):
        """参数值变化时发送到ROS"""
        self.ros_node.set_parameter_async(name, value)
        
    def _on_param_update_result(self, name: str, success: bool, message: str):
        """参数更新结果回调"""
        if success:
            self.statusBar().showMessage(f"✓ {message}", 3000)
        else:
            self.statusBar().showMessage(f"✗ {name}: {message}", 5000)
            
    def _on_params_loaded(self, params: dict):
        """参数加载完成回调"""
        for name, value in params.items():
            if name in self.param_widgets:
                self.param_widgets[name].set_value(value)
        self.statusBar().showMessage(f"已加载 {len(params)} 个参数", 3000)
        
    def _refresh_params(self):
        """刷新所有参数"""
        self.statusBar().showMessage("正在刷新参数...")
        self.ros_node.get_all_parameters_async()
        
    def _reset_to_defaults(self):
        """恢复默认值"""
        reply = QMessageBox.question(
            self, "确认", "确定要恢复所有参数到默认值吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            for name, (param_type, default, _, _, _) in PARAM_DEFINITIONS.items():
                if name in self.param_widgets:
                    self.param_widgets[name].set_value(default)
                    self.ros_node.set_parameter_async(name, default)
            self.statusBar().showMessage("已恢复所有参数到默认值", 3000)
            
    def _apply_all(self):
        """应用所有当前值"""
        for name, widget in self.param_widgets.items():
            value = widget.get_value()
            self.ros_node.set_parameter_async(name, value)
        self.statusBar().showMessage(f"正在应用 {len(self.param_widgets)} 个参数...", 3000)


def ros_spin_thread(node: Node):
    """ROS spin线程"""
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    # 初始化ROS
    rclpy.init()
    
    # 创建信号对象
    signals = RosSignals()
    
    # 创建ROS节点
    ros_node = ParamTunerNode(signals)
    
    # 等待服务
    if not ros_node.wait_for_service(timeout_sec=10.0):
        print("错误: obstacle_detector 节点未运行")
        app = QApplication(sys.argv)
        QMessageBox.critical(
            None, "连接失败",
            "obstacle_detector 节点未运行\n请先启动 obstacle_detector 节点"
        )
        ros_node.destroy_node()
        rclpy.shutdown()
        return
    
    ros_node.get_logger().info('已连接到 obstacle_detector 节点')
    
    # 启动ROS spin线程
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 创建主窗口
    window = MainWindow(ros_node, signals)
    window.show()
    
    # 运行Qt事件循环
    try:
        ret = app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(ret)


if __name__ == '__main__':
    main()