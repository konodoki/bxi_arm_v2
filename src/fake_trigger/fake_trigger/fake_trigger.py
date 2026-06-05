#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, 
    QHBoxLayout, QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt, QTimer
import sys
import threading

class SliderPublisherNode(Node):
    def __init__(self):
        super().__init__('slider_publisher_node')
        
        # 创建发布者，发布到 /pico/*_trigger 话题
        self.r_publisher_ = self.create_publisher(Float32, '/pico/right_trigger', 10)
        self.l_publisher_ = self.create_publisher(Float32, '/pico/left_trigger', 10)
        
        # 设置定时器，定期发布当前值（即使滑块没有变化）
        self.timer = self.create_timer(0.01, self.publish_current_value)  # 10Hz
        
        # 当前值，初始为0
        self.current_value = 0.0
        
        self.get_logger().info('Slider Publisher Node 已启动')

    def publish_current_value(self):
        """发布当前滑块的值"""
        msg = Float32()
        msg.data = float(self.current_value)
        self.r_publisher_.publish(msg)
        self.l_publisher_.publish(msg)
        
        # 可选：在终端显示当前值
        # self.get_logger().info(f'发布值: {self.current_value:.2f}')

    def set_value(self, value):
        """设置当前值"""
        self.current_value = value

class SliderWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        self.init_ui()
        self.setWindowTitle('ROS2 Trigger 控制器')
        self.resize(400, 200)
        
    def init_ui(self):
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        layout = QVBoxLayout(central_widget)
        
        # 标题标签
        title_label = QLabel('左右扳机控制器 (0.0 - 1.0)')
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
        layout.addWidget(title_label)
        
        # 当前值显示
        self.value_label = QLabel('当前值: 0.00')
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 14px; margin: 10px;")
        layout.addWidget(self.value_label)
        
        # 滑块
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)  # 使用0-100的范围，方便转换为0.0-1.0
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_changed)
        layout.addWidget(self.slider)
        
        # 滑块刻度标签
        scale_layout = QHBoxLayout()
        scale_layout.addWidget(QLabel('0.0'))
        scale_layout.addStretch()
        scale_layout.addWidget(QLabel('0.5'))
        scale_layout.addStretch()
        scale_layout.addWidget(QLabel('1.0'))
        layout.addLayout(scale_layout)
        
        # 预设值按钮布局
        preset_layout = QHBoxLayout()
        
        # 预设值按钮
        preset_values = [0.0, 0.25, 0.5, 0.75, 1.0]
        for value in preset_values:
            btn = QPushButton(f'{value}')
            btn.clicked.connect(lambda checked, v=value: self.set_preset_value(v))
            preset_layout.addWidget(btn)
        
        layout.addLayout(preset_layout)
        
        # 状态标签
        self.status_label = QLabel('已连接 ROS2')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: green; margin: 10px;")
        layout.addWidget(self.status_label)
        
    def on_slider_changed(self, value):
        """滑块值改变时的回调函数"""
        # 将0-100转换为0.0-1.0
        normalized_value = value / 100.0
        
        # 更新显示
        self.value_label.setText(f'当前值: {normalized_value:.2f}')
        
        # 更新ROS节点中的值
        self.ros_node.set_value(normalized_value)
        
    def set_preset_value(self, value):
        """设置预设值"""
        # 将0.0-1.0转换为0-100
        slider_value = int(value * 100)
        self.slider.setValue(slider_value)

def ros_spin(ros_node):
    """在独立线程中运行ROS2的spin"""
    rclpy.spin(ros_node)

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    try:
        # 创建ROS节点
        ros_node = SliderPublisherNode()
        
        # 在独立线程中运行ROS2的spin
        ros_thread = threading.Thread(target=ros_spin, args=(ros_node,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 创建Qt应用和窗口
        app = QApplication(sys.argv)
        window = SliderWindow(ros_node)
        window.show()
        
        # 启动Qt主循环
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭ROS2节点
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()