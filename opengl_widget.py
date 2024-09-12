import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np




class OpenGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        glutInit()  # 初始化 GLUT
        super(OpenGLWidget, self).__init__(parent)
        # 当前绘制移动点
        self.current_point = np.array([0, 0, 0])
        # 当前点是否有效
        self.current_point_is_valid = False

        self.last_pos = None
        self.x_rotation = 0  # 初始x旋转角度
        self.y_rotation = 0  # 初始y旋转角度
        self.zoom = -5.0  # 初始缩放
        self.mouse_button = None
        self.x_translation = 0
        self.z_translation = 0
        self.cam1_R = np.eye(3)
        self.cam1_t = np.array([0, 0, 0])
        self.cam2_R = np.eye(3)
        self.cam2_t = np.array([0, 0, 0])

        self.cam_fx = 204.5
        self.cam_fy = 204.5
        self.cam_cx = 320
        self.cam_cy = 240
        self.cam_matrix = np.array([[self.cam_fx, 0, self.cam_cx],
                               [0, self.cam_fy, self.cam_cy],
                               [0, 0, 1]], dtype=np.float64)

    def pixel2cam(self,px, py):
        return (px - self.cam_cx) / self.cam_fx, (py - self.cam_cy) / self.cam_fy

    # 设置绘制点坐标
    def set_display_point(self, x, y, z):
        self.current_point_is_valid = True
        self.current_point = np.array([x, y, z])
        self.update()

    # 从欧拉角构建旋转矩阵
    def euler_to_rotation_matrix(self, yaw, pitch, roll):
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        R = Rz @ Ry @ Rx
        return R

    # 旋转矩阵转换为Angle-Axis格式
    def rotation_matrix_to_gl_rotate(self, R):
        angle = np.arccos((np.trace(R) - 1) / 2)
        if np.sin(angle) != 0:
            x = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
            y = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
            z = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))
        else:
            x, y, z = 1.0, 0.0, 0.0
        angle = np.degrees(angle)
        return angle, x, y, z

    def euler2angle_axis(self, roll, pitch, yaw):  # 注意单位为弧度!!!
        rotation_matrix = self.euler_to_rotation_matrix(yaw, pitch, roll)
        angle, x, y, z = self.rotation_matrix_to_gl_rotate(rotation_matrix)
        return angle, x, y, z  # 返回角度为角度制!!!

    # 更新相机世界位姿
    # 分别为旋转矩阵和平移向量
    def update_cam_poses(self, cam1_R, cam1_t, cam2_R, cam2_t):
        self.cam1_R = cam1_R
        self.cam1_t = cam1_t
        self.cam2_R = cam2_R
        self.cam2_t = cam2_t
        self.update()

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glEnable(GL_DEPTH_TEST)
        # glEnable(GL_LIGHTING)
        # glEnable(GL_LIGHT0)
        glDisable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    # 绘制XY平面
    def draw_xy_plane(self, sx, sy, ex, ey, z, xnum, ynum):
        x_dist = (ex-sx)/xnum
        y_dist = (ey-sy)/ynum
        for i in range(xnum+1):
            glVertex3f(sx + x_dist * i, sy, z)
            glVertex3f(sx + x_dist * i, ey, z)
        for i in range(ynum+1):
            glVertex3f(sx, sy + y_dist * i, z)
            glVertex3f(ex, sy + y_dist * i, z)

    # 绘制XZ平面
    def draw_xz_plane(self, sx, sz, ex, ez, y, xnum, znum):
        glLineWidth(2.0)
        glClearColor(0.95, 0.95, 0.95, 1.0)
        glColor3f(0, 0, 0)
        x_dist = (ex - sx) / xnum
        z_dist = (ez - sz) / znum
        glBegin(GL_LINES)
        for i in range(xnum+1):
            glVertex3f(sx + x_dist * i, y, sz)
            glVertex3f(sx + x_dist * i, y, ez)
        for i in range(znum+1):
            glVertex3f(sx, y, sz + z_dist * i)
            glVertex3f(ex, y, sz + z_dist * i)
        glEnd()

    # 绘制三维坐标轴 RGB XYZ
    def draw_xyz_axis(self, x, y, z, diameter):
        glLineWidth(diameter)
        glBegin(GL_LINES)
        glColor3f(1.0, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(x, 0, 0)
        glColor3f(0, 1.0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, y, 0)
        glColor3f(0, 0, 1.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, z)
        glEnd()

    # 根据相机内参绘制理论可见范围(线框)
    def draw_camera_view(self, z_length):
        p1 = self.pixel2cam(0, 0)
        p2 = self.pixel2cam(639, 0)
        p3 = self.pixel2cam(639, 479)
        p4 = self.pixel2cam(0, 479)
        t1 = np.array([p1[0], p1[1], 1]) * z_length
        t2 = np.array([p2[0], p2[1], 1]) * z_length
        t3 = np.array([p3[0], p3[1], 1]) * z_length
        t4 = np.array([p4[0], p4[1], 1]) * z_length
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(0.7, 0.7, 0.7)
        glVertex3f(0, 0, 0)
        glVertex3f(t1[0], t1[1], t1[2])
        glVertex3f(0, 0, 0)
        glVertex3f(t2[0], t2[1], t2[2])
        glVertex3f(0, 0, 0)
        glVertex3f(t3[0], t3[1], t3[2])
        glVertex3f(0, 0, 0)
        glVertex3f(t4[0], t4[1], t4[2])
        # 连接顶点
        glVertex3f(t1[0], t1[1], t1[2])
        glVertex3f(t2[0], t2[1], t2[2])
        glVertex3f(t2[0], t2[1], t2[2])
        glVertex3f(t3[0], t3[1], t3[2])
        glVertex3f(t3[0], t3[1], t3[2])
        glVertex3f(t4[0], t4[1], t4[2])
        glVertex3f(t4[0], t4[1], t4[2])
        glVertex3f(t1[0], t1[1], t1[2])
        glEnd()

    # 绘制所有相机位姿 paintGL中直接调用
    def draw_cam_poses(self):
        # ---Cam1---
        glPushMatrix()
        # 绘制相机坐标轴
        glTranslatef(self.cam1_t[0], self.cam1_t[1], self.cam1_t[2])
        angle, x, y, z = self.rotation_matrix_to_gl_rotate(self.cam1_R)
        glRotatef(angle, x, y, z)
        self.draw_xyz_axis(0.5, 0.5, 0.5, 6)
        # 绘制相机可视框
        self.draw_camera_view(2)
        glPopMatrix()

        # ---Cam2---
        glPushMatrix()
        # 绘制相机坐标轴
        glTranslatef(self.cam2_t[0], self.cam2_t[1], self.cam2_t[2])
        angle, x, y, z = self.rotation_matrix_to_gl_rotate(self.cam2_R)
        glRotatef(angle, x, y, z)
        self.draw_xyz_axis(0.5, 0.5, 0.5, 6)
        # 绘制相机可视框
        self.draw_camera_view(2)
        glPopMatrix()

    # 显示单个点
    def draw_point(self):
        if self.current_point_is_valid:
            glColor3f(1.0, 0, 1.0)
            quad = gluNewQuadric()
            glPushMatrix()
            glTranslatef(self.current_point[0], self.current_point[1], self.current_point[2])
            gluSphere(quad, 0.05, 50, 50)
            glPopMatrix()
            gluDeleteQuadric(quad)

    # 绘制更新函数
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()  # 复位单帧变换矩阵
        # 鼠标移动控制
        glTranslatef(self.z_translation*0.01, -self.x_translation*0.01, self.zoom)
        glRotatef(self.x_rotation*0.5, 1.0, 0.0, 0.0)  # 绕x轴旋转
        glRotatef(self.y_rotation*0.5, 0.0, 1.0, 0.0)  # 绕y轴旋转
        # 绘制坐标平面
        self.draw_xz_plane(-5, -5, 5, 5, 0, 10, 10)
        # 绘制相机Pose
        self.draw_cam_poses()
        # 绘制三角化点
        self.draw_point()

    # 鼠标事件处理回调函数
    def mousePressEvent(self, event):
        self.last_pos = event.pos()
        self.mouse_button = event.button()

    # 鼠标移动事件处理函数
    def mouseMoveEvent(self, event):
        if self.last_pos is not None:
            dx = event.x() - self.last_pos.x()
            dy = event.y() - self.last_pos.y()
            if self.mouse_button == Qt.LeftButton:
                self.x_rotation += dy
                self.y_rotation += dx
            elif self.mouse_button == Qt.RightButton:
                self.x_translation += dy
                self.z_translation += dx
            self.update()
        self.last_pos = event.pos()

    # 鼠标滚轮事件处理回调函数
    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120  # 每个步长的单位为120
        self.zoom += delta * 0.5  # 缩放因子
        self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setGeometry(100, 100, 1000, 1000)
        self.glWidget = OpenGLWidget(self)
        self.setCentralWidget(self.glWidget)


# Main Test
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
