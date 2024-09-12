import cv2 as cv
import numpy as np
from collections import deque
import os
from PyQt5.QtCore import pyqtSignal, QObject

# cam_num: 使用的相机数量
class Calibration(QObject):
    log_signal = pyqtSignal(str)  # logger

    def __init__(self, cam_num):
        super(QObject, self).__init__()
        self.cam_num = cam_num
        # 相机外参
        self.cam1_R = np.eye(3)
        self.cam1_t = np.array([0, 0, 0])
        self.cam2_R = np.eye(3)
        self.cam2_t = np.array([0, 0, 0])
        self.cam1_proj = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        self.cam2_proj = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

        # deque
        self.cam_points = [deque() for _ in range(self.cam_num)]
        # numpy array
        self.cam_array = [np.array(None) for _ in range(self.cam_num)]
        self.valid_points_num = 0  # 有效采集点数量
        self.calibration_ok = False  # 是否已经成功校准
        # cam intrinsic
        self.cam1_fx = 204.64863681
        self.cam1_fy = 204.47041377
        self.cam1_cx = 308.78000754
        self.cam1_cy = 258.21809417
        self.cam1_k1 = 0.24406997
        self.cam1_k2 = -0.22412072
        self.cam1_p1 = -0.00079476
        self.cam1_p2 = -0.00035923
        self.cam1_k3 = 0.05262498

        self.cam2_fx = 204.42765186
        self.cam2_fy = 204.43521494
        self.cam2_cx = 310.99781296
        self.cam2_cy = 257.91267286
        self.cam2_k1 = 0.2305133
        self.cam2_k2 = -0.20287915
        self.cam2_p1 = -0.00140612
        self.cam2_p2 = 0.0033575
        self.cam2_k3 = 0.04448097

        # Standard Camera Intrinsic
        self.cam_fx = 204.5
        self.cam_fy = 204.5
        self.cam_cx = 320
        self.cam_cy = 240
        self.cam_matrix = np.array([[self.cam_fx, 0, self.cam_cx],
                               [0, self.cam_fy, self.cam_cy],
                               [0, 0, 1]], dtype=np.float64)

        self.cam1_matrix = np.array([[self.cam1_fx, 0, self.cam1_cx],
                                  [0, self.cam1_fy, self.cam1_cy],
                                  [0, 0, 1]], dtype=np.float64)
        self.cam1_dist = np.array([self.cam1_k1, self.cam1_k2, self.cam1_p1, self.cam1_p2, self.cam1_k3], dtype=np.float64)

        self.cam2_matrix = np.array([[self.cam2_fx, 0, self.cam2_cx],
                                  [0, self.cam2_fy, self.cam2_cy],
                                  [0, 0, 1]], dtype=np.float64)
        self.cam2_dist = np.array([self.cam2_k1, self.cam2_k2, self.cam2_p1, self.cam2_p2, self.cam2_k3], dtype=np.float64)


    def log(self, log_str):
        self.log_signal.emit(log_str)

    # points: 单个相机点
    # num:有效相机数量(使用中)
    def add_valid_points(self, points):
        # 检测点有效性
        if len(points) != self.cam_num:
            return
        else:
            self.valid_points_num += 1
            for index in range(self.cam_num):
                self.cam_points[index].append(points[index])
            # self.cam_points[0].append(points[0])
            # self.cam_points[1].append(points[1])

    # 清除所有采集的点
    def clear_all_points(self):
        self.valid_points_num = 0
        for index in range(self.cam_num):
            self.cam_points[index].clear()

    def print_all_points(self):
        # print(self.cam_points[0])
        # print(self.cam_points[1])
        for cam_index in range(self.cam_num):
            point_num = len(self.cam_points[cam_index])
            # print(f"Cam{cam_index} has {point_num} points:")
            self.log(f"Cam{cam_index} has {point_num} points:")
            for point_index, point in enumerate(self.cam_points[cam_index]):
                self.log(f"Point{point_index}: x:{point[0]} y:{point[1]}")
                # print(f"Point{point_index}: x:{point[0]} y:{point[1]}")

    def cam1_pixel2cam(self, px, py):
        # return (px-cam1_cx)/cam1_fx, (py-cam1_cy)/cam1_fy
        undistorted_points = cv.undistortPoints(np.array([px, py], np.float64), self.cam1_matrix, self.cam1_dist, P=self.cam1_matrix)
        upx = undistorted_points[0][0][0]
        upy = undistorted_points[0][0][1]
        return (upx - self.cam1_cx) / self.cam1_fx, (upy - self.cam1_cy) / self.cam1_fy

    def cam2_pixel2cam(self, px, py):
        # return (px-cam2_cx)/cam2_fx, (py-cam2_cy)/cam2_fy
        undistorted_points = cv.undistortPoints(np.array([px, py], np.float64), self.cam2_matrix, self.cam2_dist, P=self.cam2_matrix)
        upx = undistorted_points[0][0][0]
        upy = undistorted_points[0][0][1]
        return (upx - self.cam2_cx) / self.cam2_fx, (upy - self.cam2_cy) / self.cam2_fy

    # 归一化平面(Z=1)转换为像素坐标
    # 使用理想相机模型 fx=1 fy=1 cx=320 cy=240
    def cam2pixel(self, sx, sy):
        return (sx * self.cam_fx + self.cam_cx), (sy * self.cam_fy + self.cam_cy)

    # 开始进行计算求解相机相对位姿 即相机外参标定
    def start_calculation(self):
        self.log("Calibration: Begin Calculate!")
        cam1_list = []
        cam2_list = []
        while self.cam_points[0] and self.cam_points[1]:
            point1 = self.cam_points[0].popleft()
            point2 = self.cam_points[1].popleft()
            # 在此处对像素坐标进行处理
            _point1 = self.cam1_pixel2cam(point1[0], point1[1])
            _point2 = self.cam2_pixel2cam(point2[0], point2[1])
            cam1_list.append(self.cam2pixel(_point1[0], _point1[1]))
            cam2_list.append(self.cam2pixel(_point2[0], _point2[1]))
        cam1_array = np.array(cam1_list)
        cam2_array = np.array(cam2_list)
        self.log("Calibration: Find Essential Matrix!")
        try:
            E, mask = cv.findEssentialMat(
                points1=cam1_array,
                points2=cam2_array,
                cameraMatrix=self.cam_matrix,
                method=cv.RANSAC,
                threshold=2.0
            )
            # print("the essential mask:")
            # print(mask)
            print("essential useful points rate:")
            success = np.sum(mask) / mask.size
            print("useful points:")
            print(np.sum(mask))
            print(success)
            print("the essential matrix:")
            print(E)
            ret, R, t, mask, tri_points = cv.recoverPose(
                E=E,
                points1=cam1_array,
                points2=cam2_array,
                cameraMatrix=self.cam_matrix,
                distanceThresh=5  # 5刚刚好
            )
            # print(tri_points)
            # print("the point mask:")
            # print(mask)
            print("recoverPose useful points rate:")
            success = np.sum(mask) / mask.size / 255
            print("recoverPose useful points:")
            print(np.sum(mask) / 255)
            print(success)
            print("the rotation matrix:")
            print(R)
            print("the t vector")
            print(t)
            # 更新相机位姿
            self.log("Calibration: Update Cam Poses!")
            self.cam1_R = np.eye(3)
            self.cam1_t = np.array([0, 0, 0])
            self.cam2_R = R
            self.cam2_t = t
            R1 = self.cam1_R
            t1 = self.cam1_t
            R2 = self.cam2_R
            t2 = self.cam2_t
            self.cam1_proj = np.hstack((R1, t1.reshape(-1, 1)))
            self.cam2_proj = np.hstack((R2, t2.reshape(-1, 1)))
            print(self.cam1_proj)
            print(self.cam2_proj)
            self.calibration_ok = True

        except cv.error as e:
            self.log(f"OpenCV Error:{str(e)}")
        except Exception as e:
            self.log(f"Error:{str(e)}")

    def triangulate(self, point1, point2):  # 三角化函数
        if self.calibration_ok:
            print("Begin Triangulate!")
            # 将点转换为归一化相机坐标
            _point1 = self.cam1_pixel2cam(point1[0], point1[1])
            _point2 = self.cam2_pixel2cam(point2[0], point2[1])
            _point1 = np.array(_point1)
            _point2 = np.array(_point2)
            print("Point1:")
            print(_point1)
            print("Point2:")
            print(_point2)
            try:
                tri_points = cv.triangulatePoints(projMatr1=self.cam1_proj, projMatr2=self.cam2_proj,
                                                  projPoints1=_point1.T,
                                                  projPoints2=_point2.T)
                if tri_points[3][0] != 0:
                    _x = tri_points[0][0] / tri_points[3][0]
                    _y = tri_points[1][0] / tri_points[3][0]
                    _z = tri_points[2][0] / tri_points[3][0]
                    self.log(f"X:{str(_x)} Y:{str(_y)} Z:{str(_z)}")
                    return _x, _y, _z
            except cv.error as e:
                self.log(f"OpenCV Error:{str(e)}")
            except Exception as e:
                self.log(f"Error:{str(e)}")


