from udp_rx import UDP_RX
from calibration import Calibration
from opengl_widget import OpenGLWidget
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel,
                             QPushButton, QHBoxLayout, QLineEdit, QSpinBox,
                             QGridLayout, QSizePolicy, QFrame, QTextEdit)
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, QByteArray, QBuffer, QTimer
import time
from PyQt5.QtCore import pyqtSignal, QObject
import numpy as np
import cv2 as cv
import socket


# log显示模块
class Logger(QWidget):
    def __init__(self):
        super().__init__()
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setLineWrapMode(QTextEdit.WidgetWidth)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.log_output)
        self.setLayout(self.layout)

    def append_log(self, message):
        self.log_output.append(message)
        self.log_output.ensureCursorVisible()


# 顶层监视模块
# 视觉定位坐标输出
# GUI可视化显示
class Monitor(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        # 是否正在三角化
        self.is_triangulating = False
        # UDP CAM监视模块
        self.udp1_rx = UDP_RX("udp1", 1)
        self.udp2_rx = UDP_RX("udp2", 2)
        self.udp1_rx.update_signal.connect(self.cam1_update_callback)
        # Calibration模块
        self.calibration = Calibration(2)
        self.calibration.log_signal.connect(self.log_callback)  # logger output
        # OPENGL 显示模块
        self.opengl_widget = OpenGLWidget()
        # Logger 模块
        self.logger = Logger()
        # 主显示窗口
        self.main_hbox_layout = QHBoxLayout()
        self.image_grid_layout = QGridLayout()
        self.info_vbox_layout = QVBoxLayout()

        self.image_grid_layout.addWidget(self.udp1_rx, 0, 0)
        self.image_grid_layout.addWidget(self.udp2_rx, 1, 0)

        # Main Info
        # Calibration
        self.calibration_frame = QFrame()
        self.calibration_grid_layout = QGridLayout()

        self.auto_calibration_button = QPushButton("Start Auto Calibration")  # 开始自动捕获有效点
        self.capture_sample_button = QPushButton("Capture Sample")  # 手动捕获有效点
        self.start_calculation_button = QPushButton("Start Calculation")  # 开始计算标定结果
        self.print_all_points_button = QPushButton("Print Points")  # 打印所有相机所有采集点
        self.clear_all_points_button = QPushButton("Clear Points")  # 清除所有采集的点
        self.triangulate_one_point_button = QPushButton("Triangulate")  # 三角化单个点
        self.triangulating_button = QPushButton("Start Triangulating")  # 持续三角化开始/停止按钮
        self.valid_sample_count_label = QLabel("Valid Samples:")
        self.valid_sample_count_value_label = QLabel("0")
        self.calibration_state_label = QLabel("State:")
        self.calibration_state_value_label = QLabel()

        self.capture_sample_button.clicked.connect(self.upload_points)
        self.print_all_points_button.clicked.connect(self.print_all_points)
        self.clear_all_points_button.clicked.connect(self.clear_all_points)
        self.start_calculation_button.clicked.connect(self.start_calculation)
        self.triangulate_one_point_button.clicked.connect(self.triangulate_one_point)
        self.triangulating_button.clicked.connect(self.triangulation_button_callback)

        self.calibration_grid_layout.addWidget(self.auto_calibration_button, 0, 0)
        self.calibration_grid_layout.addWidget(self.capture_sample_button, 0, 1)
        self.calibration_grid_layout.addWidget(self.start_calculation_button, 1, 0)
        self.calibration_grid_layout.addWidget(self.print_all_points_button, 1, 1)
        self.calibration_grid_layout.addWidget(self.clear_all_points_button, 2, 0)
        self.calibration_grid_layout.addWidget(self.valid_sample_count_label, 3, 0)
        self.calibration_grid_layout.addWidget(self.valid_sample_count_value_label, 3, 1)
        self.calibration_grid_layout.addWidget(self.calibration_state_label, 4, 0)
        self.calibration_grid_layout.addWidget(self.calibration_state_value_label, 4, 1)
        self.calibration_grid_layout.addWidget(self.triangulate_one_point_button, 5, 0)
        self.calibration_grid_layout.addWidget(self.triangulating_button, 5, 1)

        self.calibration_frame.setLayout(self.calibration_grid_layout)
        self.calibration_frame.setObjectName("calibration_frame")
        self.calibration_frame.setStyleSheet("""
                    QFrame#calibration_frame{
                        border: 1px solid black;
                        border-radius: 15px;
                        padding: 5px;
                        font: 20px "Consolas";
                    }
                    QFrame#calibration_frame * {
                        font: 20px "Consolas";
                    }
                """)
        self.info_vbox_layout.addWidget(self.calibration_frame)
        # Logger
        self.logger_frame = QFrame()
        self.logger_hlayout = QHBoxLayout()
        self.logger_hlayout.addWidget(self.logger)
        self.logger_frame.setObjectName("logger_frame")
        self.logger_frame.setStyleSheet("""
                    QFrame#logger_frame{
                        border: 1px solid black;
                        border-radius: 15px;
                        padding: 5px;
                        font: 20px "Consolas";
                    }
                    QFrame#logger_frame * {
                        font: 20px "Consolas";
                    }
                """)
        self.logger_frame.setLayout(self.logger_hlayout)
        self.info_vbox_layout.addWidget(self.logger_frame)

        # OPENGL Widget
        self.opengl_frame = QFrame()
        self.opengl_hlayout = QHBoxLayout()
        self.opengl_hlayout.addWidget(self.opengl_widget)
        self.opengl_frame.setLayout(self.opengl_hlayout)
        self.opengl_frame.setObjectName("opengl_frame")
        self.opengl_frame.setStyleSheet("""
                border: 2px solid black;
                border-radius: 15px;
                font: 20px "Consolas";
        """)
        self.info_vbox_layout.addWidget(self.opengl_frame)

        self.info_vbox_layout.setStretch(0, 1)
        self.info_vbox_layout.setStretch(1, 1)
        self.info_vbox_layout.setStretch(2, 2)
        self.logger.append_log("System Begin!")

        self.main_hbox_layout.addLayout(self.image_grid_layout)
        self.main_hbox_layout.addLayout(self.info_vbox_layout)
        self.main_hbox_layout.setStretch(0, 3)
        self.main_hbox_layout.setStretch(1, 4)

        self.setLayout(self.main_hbox_layout)

    # 更新相机位姿到opengl显示模块
    def update_cam_poses(self):
        self.logger.append_log("Update Cam Poses to OpenGL Widget!")
        # 获取相机位姿
        R1 = self.calibration.cam1_R
        t1 = self.calibration.cam1_t
        R2 = self.calibration.cam2_R
        t2 = self.calibration.cam2_t
        self.opengl_widget.update_cam_poses(R1, t1, R2, t2)

    # log信号回调函数 用于其他模块输出log信息
    def log_callback(self, log_str):
        self.logger.append_log(log_str)

    # 向Calibration模块上传有效点 用于求解相机相对位姿
    def upload_points(self):
        # 检测点是否有效
        point1 = self.udp1_rx.get_current_valid_point()
        point2 = self.udp2_rx.get_current_valid_point()
        points = [point1, point2]
        if point1 and point2:  # 均有效
            self.calibration.add_valid_points(points)
            self.valid_sample_count_value_label.setText(str(self.get_valid_points_num()))
            print("upload success!")
        else:
            print("upload failed!")


    # CAM1触发三角化信号回调函数  认为此时CAM1 CAM2近似同步
    def cam1_update_callback(self):
        if self.is_triangulating:  # 正在三角化
            # 触发单次三角化函数
            self.triangulate_one_point()

    # 开始/停止持续三角化 按钮回调函数
    def triangulation_button_callback(self):
        if self.is_triangulating:
            self.is_triangulating = False
            self.logger.append_log("MAIN: Stop Triangulate!")
            self.triangulating_button.setText("Start Triangulating")
        else:
            self.is_triangulating = True
            self.logger.append_log("MAIN: Start Triangulate!")
            self.triangulating_button.setText("Stop Triangulating")

    # 获取当前有效点数量
    def get_valid_points_num(self):
        return self.calibration.valid_points_num

    # 清除所有采集的有效点
    def clear_all_points(self):
        self.calibration.clear_all_points()
        self.valid_sample_count_value_label.setText(str(self.get_valid_points_num))

    # 打印所有采集的有效点
    def print_all_points(self):
        self.calibration.print_all_points()

    # 开始校准计算
    def start_calculation(self):
        self.calibration.start_calculation()
        self.update_cam_poses()

    # 单对点三角化
    def triangulate_one_point(self):
        # 检测点是否有效
        point1 = self.udp1_rx.get_current_valid_point()
        point2 = self.udp2_rx.get_current_valid_point()
        if point1 and point2:
            _x, _y, _z = self.calibration.triangulate(point1, point2)
            self.opengl_widget.set_display_point(_x, _y, _z)
            # print("triangulate success!")
        else:
            print("triangulate failed!")


if __name__ == "__main__":
    app = QApplication([])
    main_widget = QWidget()
    main_monitor = Monitor()
    main_widget.setLayout(main_monitor.main_hbox_layout)
    main_widget.show()
    app.exec_()