from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel,
                             QPushButton, QHBoxLayout, QLineEdit, QSpinBox,
                             QGridLayout, QSizePolicy, QFrame)
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, QByteArray, QBuffer
from PyQt5.QtCore import pyqtSignal, QObject
import numpy as np
import cv2 as cv
import socket
import select
import re

# UDP单包最大数量(bytes)
UDP_BUFFER_SIZE = 60000

# 滑动均值滤波器 用于FPS平滑
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data_window = []

    def apply(self, new_data_point):
        self.data_window.append(new_data_point)
        if len(self.data_window) > self.window_size:
            self.data_window.pop(0)
        return sum(self.data_window) / len(self.data_window)


class ReceiveThread(QThread):
    udp_state_signal = pyqtSignal(bool)  # 报告是否超时
    image_update_signal = pyqtSignal(bool, bytes)
    fps_update_signal = pyqtSignal(float)

    def __init__(self, udp_socket):
        super().__init__()
        # Timer
        self.sys_tick_freq = cv.getTickFrequency()
        self.last_tick = cv.getTickCount()
        self.curr_tick = cv.getTickCount()
        self.avr_fps = 0.00  # 平均帧率
        # Threading
        self.running = False  # 线程是否正在运行
        # Socket
        self.udp_socket = udp_socket
        self.socket_rx_addr = None  # 接收到的信息来源地址
        # Image Data
        self.raw_udp_data = None  # 原始UDP接收数据
        # self.cv_image = None  # 解码后OpenCV图像
        self.success_image_count = 0  # 总解码成功图像数量
        self.success_time = 0.00  # 接收连续计时
        self.fps_filter = MovingAverageFilter(window_size=100)

    # 返回图像信息是否有效
    def is_data_valid(self):
        if self.running and self.success_image_count > 0:
            return True
        else:
            return False

    # 获取距离上一次获取图像的时间间隔 用于计算FPS
    def get_dt(self):
        self.curr_tick = cv.getTickCount()
        dt = (self.curr_tick - self.last_tick) / self.sys_tick_freq
        self.last_tick = self.curr_tick
        return dt

    # 线程运行函数
    def run(self):
        print("UDP RX Thread RUNNING")
        self.udp_socket.setblocking(False)  # 使用非阻塞接收 设置超时时间 用于检测超时
        while self.running:
            if self.udp_socket:
                ready = select.select([self.udp_socket], [], [], 1.0)
                if ready[0]:
                    self.raw_udp_data, self.socket_rx_addr = self.udp_socket.recvfrom(UDP_BUFFER_SIZE)
                    if len(self.raw_udp_data) > 0:
                        if len(self.raw_udp_data) == UDP_BUFFER_SIZE:
                            print("the Image Data is too Large!")
                            continue
                        if self.raw_udp_data[0] == 0xff and self.raw_udp_data[1] == 0xd8 and self.raw_udp_data[-2] == 0xff and self.raw_udp_data[-1] == 0xd9:
                            self.image_update_signal.emit(self.running, self.raw_udp_data)
                            self.success_image_count += 1
                            self.avr_fps = self.fps_filter.apply(1.0 / self.get_dt())
                            self.fps_update_signal.emit(self.avr_fps)
                            self.udp_state_signal.emit(True)
                        else:
                            print("UDP Receive Lost! Data is Broken!")
                            continue
                    else:
                        print("No Image Received!")
                        continue
                else:
                    # UDP超时处理
                    # print("UDP Timeout!")
                    self.success_image_count = 0
                    self.success_time = 0
                    self.udp_state_signal.emit(False)
                    continue
            else:
                print("Socket is None!")
                continue

    # 线程停止函数
    def stop(self):
        print("Thread Stop!")
        self.running = False


class ImageLabel(QLabel):
    def __init__(self):
        super().__init__()
        self.label_size = self.size()
        self.setScaledContents(False)  # 禁用直接缩放，使用resizeEvent处理

    def resizeEvent(self, event):
        self.label_size = self.size()

    def get_current_max_length(self):
        self.label_size = self.size()
        if self.label_size.width() > self.label_size.height():
            return self.label_size.width()
        else:
            return self.label_size.height()


# UDP相机接收窗口
class UDP_RX(QWidget):
    update_signal = pyqtSignal()
    image_save_signal = pyqtSignal()
    udp_start_listening_signal = pyqtSignal()

    def __init__(self, name, index, parent=None):
        super().__init__(parent)
        self.detect_points = 0
        self.udp_timeout = True  # 默认超时
        self.current_points = None  # 最新帧检测到的点集
        self.listening_socket = None
        self.udp_is_listening = False
        self.cv_image = None
        # Camera Info
        self.index = index
        # self.setWindowTitle(name)
        # self.setGeometry(100, 100, 400, 300)
        # Layout
        self.main_hbox_layout = QHBoxLayout(self)  # 最外层显示栏
        self.info_vbox_layout = QVBoxLayout(self)  # 右侧信息显示栏
        self.info_widget = QWidget(self)
        # UDP Info
        self.udp_info_frame = QFrame()
        self.udp_info_frame.setObjectName("udp_info_frame")
        self.udp_info_glayout = QGridLayout()

        self.udp_listening_ipaddr_label = QLabel("Listening IP:")
        self.udp_listening_ipaddr_lineedit = QLineEdit()
        #     Set the default listening ip address
        hostname = socket.gethostname()
        ip_addr = socket.gethostbyname(hostname)
        self.udp_listening_ipaddr_lineedit.setText(ip_addr)
        self.udp_listening_ipaddr_lineedit.textChanged.connect(self.validate_ip)

        self.udp_listening_port_label = QLabel("Listening Port:")
        self.udp_listening_port_spinbox = QSpinBox(self)
        self.udp_listening_port_spinbox.setRange(1024, 65535)
        self.udp_listening_port_spinbox.setValue(6666)
        self.udp_receive_from_label = QLabel("Receive from:")
        self.udp_receive_addr_label = QLabel("   .   .   .   ")
        self.udp_listening_button = QPushButton()
        self.udp_listening_button.setEnabled(False)
        self.validate_ip()
        self.udp_listening_button.setText("Start Listening")

        self.udp_info_glayout.addWidget(self.udp_listening_ipaddr_label, 0, 0)
        self.udp_info_glayout.addWidget(self.udp_listening_ipaddr_lineedit, 0, 1)
        self.udp_info_glayout.addWidget(self.udp_listening_port_label, 1, 0)
        self.udp_info_glayout.addWidget(self.udp_listening_port_spinbox, 1, 1)
        self.udp_info_glayout.addWidget(self.udp_receive_from_label, 2, 0)
        self.udp_info_glayout.addWidget(self.udp_receive_addr_label, 2, 1)
        self.udp_info_glayout.addWidget(self.udp_listening_button)

        self.udp_info_frame.setLayout(self.udp_info_glayout)
        self.udp_info_frame.setStyleSheet("""
            QFrame#udp_info_frame{
                border: 1px solid black;
                border-radius: 15px;
                padding: 5px;
                font: 20px "Consolas";
            }
            QFrame#udp_info_frame * {
                font: 20px "Consolas";
            }
        """)
        self.info_vbox_layout.addWidget(self.udp_info_frame)

        # Image info
        self.image_info_frame = QFrame()
        self.image_info_frame.setObjectName("image_info_frame")
        self.image_info_grid_layout = QGridLayout()

        self.fps_label = QLabel("Average FPS:")
        self.fps_value_label = QLabel()
        self.points_label = QLabel("Points:")
        self.points_value_label = QLabel()
        self.state_label = QLabel("Detect State:")
        self.state_value_label = QLabel()

        self.image_info_grid_layout.addWidget(self.fps_label, 0, 0)
        self.image_info_grid_layout.addWidget(self.fps_value_label, 0, 1)
        self.image_info_grid_layout.addWidget(self.points_label, 1, 0)
        self.image_info_grid_layout.addWidget(self.points_value_label, 1, 1)
        self.image_info_grid_layout.addWidget(self.state_label, 2, 0)
        self.image_info_grid_layout.addWidget(self.state_value_label, 2, 1)

        self.image_info_frame.setLayout(self.image_info_grid_layout)
        self.info_vbox_layout.addWidget(self.image_info_frame)
        self.image_info_frame.setStyleSheet("""
            QFrame#image_info_frame{
                border: 1px solid black;
                border-radius: 15px;
                padding: 5px;
                font: 20px "Consolas";
            }
            QFrame#image_info_frame * {
                font: 20px "Consolas";
            }
        """)
        self.info_widget.setLayout(self.info_vbox_layout)

        # ImageLabel
        # self.image_label = QLabel(self)
        # self.image_label.setScaledContents(True)
        self.image_label = ImageLabel()
        # self.image_label.setPixmap(self.pixmap)
        self.show_no_video()

        self.main_hbox_layout.addWidget(self.image_label)
        self.main_hbox_layout.addWidget(self.info_widget)
        self.main_hbox_layout.setStretch(0, 2)
        self.main_hbox_layout.setStretch(1, 1)
        # Thread
        self.rx_thread = ReceiveThread(None)
        # Signal Connect
        self.rx_thread.image_update_signal.connect(self.image_update)
        self.rx_thread.fps_update_signal.connect(self.fps_update)
        self.rx_thread.udp_state_signal.connect(self.is_udp_timeout)
        self.udp_start_listening_signal.connect(self.udp_start_listening)
        self.udp_listening_button.clicked.connect(self.udp_start_listening)

    # UDP超时或者收到新图像时执行此回调函数
    def is_udp_timeout(self, udp_state):
        if udp_state:
            self.udp_timeout = False
        else:
            self.udp_timeout = True
        self.update_detect_state()

    # 返回图像是否有效
    def is_image_valid(self):
        return self.rx_thread.is_data_valid()

    # 获取当前检测点集
    def get_current_points(self):
        return self.current_points

    # 获取当前单个有效点(only one)
    def get_current_valid_point(self):
        if not self.udp_timeout:
            if self.detect_points == 1:
                return self.current_points[0]
            else:
                return None
        return None

    # 更新检测状态显示
    def update_detect_state(self):
        if not self.udp_timeout:
            if self.detect_points == 1:
                self.state_value_label.setText("Success!")
            elif self.detect_points > 1:
                self.state_value_label.setText("Too much points!")
            elif self.detect_points == 0:
                self.state_value_label.setText("No point found.")
        else:
            self.state_value_label.setText("Image Invalid")

    # 获取host有效可监听IP地址
    def validate_ip(self):
        ip = self.udp_listening_ipaddr_lineedit.text()
        regex = r"^\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b$"
        if re.match(regex, ip):
            if all(0 <= int(part) <= 255 for part in ip.split('.')):
                self.udp_listening_button.setEnabled(True)  # 合法
            else:
                self.udp_listening_button.setEnabled(False)  # 不合法
        else:
            self.udp_listening_button.setEnabled(False)  # 不合法

    # 显示当前检测到的所有点
    def label_show_points(self, points):
        show_str = ""
        if points:
            for point in points:
                x = point[0]
                y = point[1]
                show_str += f"({x:.2f}, {y:.2f})\n"
        self.points_value_label.setText(show_str)

    # 图像检测点函数
    def find_dot_from_image(self, img):
        # img = cv.GaussianBlur(img,(5,5),0)
        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255 * 0.9, 255, cv.THRESH_BINARY)[1]
        contours, _ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        img = cv.drawContours(img, contours, -1, (0, 255, 0), 1)

        image_points = []
        for contour in contours:
            moments = cv.moments(contour)
            if moments["m00"] != 0:
                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]
                image_points.append([center_x, center_y])
                center_x = int(center_x)
                center_y = int(center_y)
                cv.putText(img, f'({center_x}, {center_y})', (center_x, center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3,
                           (100, 255, 100), 1)
                cv.circle(img, (center_x, center_y), 1, (100, 255, 100), -1)

        num_points = len(image_points)
        if num_points == 0:
            image_points = None

        return img, image_points, num_points

    # 更新FPS显示回调函数
    def fps_update(self, fps):
        formatted_str = "{:.2f}".format(fps)
        self.fps_value_label.setText(formatted_str)

    # UDP开始监听回调函数
    def udp_start_listening(self):
        if self.udp_is_listening:  # 停止监听
            self.udp_is_listening = False
            self.rx_thread.running = False
            self.rx_thread.stop()
            self.rx_thread.wait()
            print("CLose the Socket!")
            self.rx_thread.udp_socket.close()
            self.udp_listening_port_spinbox.setEnabled(True)
            self.udp_listening_ipaddr_lineedit.setEnabled(True)
            self.udp_listening_button.setText("Start Listening")
            self.show_no_video()

        else:
            self.udp_is_listening = True
            # Socket Bind
            # Socket
            self.listening_socket = (self.udp_listening_ipaddr_lineedit.text(), self.udp_listening_port_spinbox.value())
            self.rx_thread.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.rx_thread.udp_socket.bind(self.listening_socket)
            print(f"Listening on {self.listening_socket}")
            self.rx_thread.udp_socket = self.rx_thread.udp_socket  # 更新线程Socket
            self.rx_thread.running = True
            self.rx_thread.start()

            self.udp_listening_port_spinbox.setEnabled(False)
            self.udp_listening_ipaddr_lineedit.setEnabled(False)
            self.udp_listening_button.setText("Stop Listening")

    # 新图像获取回调函数
    def image_update(self, running, image_data):
        # 不知为什 必须添加对运行的状态检测 否则在停止时会有多余刷新
        if running:
            pixmap = QPixmap()
            byte_array = QByteArray(image_data)
            buffer = QBuffer(byte_array)
            buffer.open(QBuffer.ReadOnly)
            pixmap.loadFromData(buffer.data(), "JPEG")
            self.image_label.setPixmap(pixmap)
            if pixmap.width() != 640 or pixmap.height() != 480:
                print("The Image Size is Error!")
                return

            np_data = np.frombuffer(image_data, dtype=np.uint8)
            self.cv_image = cv.imdecode(np_data, cv.IMREAD_COLOR)
            if self.cv_image is not None:
                # 图像解码成功
                _img, _points, _num_points = self.find_dot_from_image(self.cv_image)
                self.current_points = _points
                self.label_show_points(_points)
                self.detect_points = _num_points
                self.update_detect_state()
                self.update_signal.emit()  # 发送图像更新信号
                # cv.imshow("decode img", self.cv_image)
                # cv.waitKey(1)
            else:
                # raise ValueError("解码失败，数据不是有效的JPEG图像。")
                print("opencv decode failed")

        else:
            return

    # 显示"No Video"图像
    def show_no_video(self):
        pixmap = QPixmap()
        if pixmap.load("no_video.jpg"):
            # size = self.image_label.size()
            # if size.width() > self.height():
            #     self.image_label.setPixmap(pixmap.scaledToWidth(size.width()))
            # else:
            #     self.image_label.setPixmap(pixmap.scaledToWidth(size.height()))
            self.image_label.setPixmap(pixmap)
            # print("Show No Video")
        else:
            print("Failed to load <No Video> image")

    # 窗口关闭事件回调函数
    def closeEvent(self, event):
        self.rx_thread.stop()
        self.rx_thread.wait()
        super().closeEvent(event)


# main test
if __name__ == "__main__":
    app = QApplication([])
    window1 = UDP_RX("main", 1)
    window1.show()
    app.exec_()