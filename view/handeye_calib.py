#!/usr/bin/env python
# coding: utf-8

"""
@author: George Zeng
@contact: george.zeng@syensqo.com
@version: 1.0.0
@file: handeye_calib.py
@time: 2024/3/11 12:59
"""

import time
import numpy as np
import cv2
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QThreadPool
from PyQt5.QtWidgets import QApplication, QWidget
from qfluentwidgets import FluentIcon as FIF
from qfluentwidgets import (Flyout, FlyoutAnimationType, InfoBarIcon, StateToolTip)

from view.handeye_calib_ui import Ui_HandeyeCalib
from universal_robot.ur_api import URDashboard, URRTInterface
from rs_camera.rs_application import RSApplication
from util.coordinate_trans import rv2rpy, rm2rpy
from handeye.handeye_calib import HandeyeCalibration
from handeye.handeye_robot import generate_poses_around_state, validate_pose, is_pose_repeated, IK_Solver


class HandEyeCalib(Ui_HandeyeCalib, QWidget):
    def __init__(self, env):
        super().__init__()
        self.setupUi(self)

        self.thread_pool = QThreadPool.globalInstance()
        self.ur_dash = None
        self.ur_cmd = None
        self.camera: CameraThread = QThread()

        self.robot_timer = QTimer()
        self.camera_on = False

        self.M_target_to_camera = np.zeros((4, 4))
        self.hand = []
        self.eye = []

        self.calibration_sample = int(env["Calibration_sample"])
        self.translation_delta = float(env["Translation_delta"])
        self.angle_delta = float(env["Angle_delta"])
        self.chessboard_x_number = int(env["Chessboard_x_number"])
        self.chessboard_y_number = int(env["Chessboard_y_number"])
        self.chessboard_cell_size = float(env["Chessboard_cell_size"])
        self.aruco_tag_size = float(env["Aruco_tag_size"])
        self.robot_ip.setText(str(env["Robot_IP"]))

        self.state = None

        self._init_window()
        self._bind_slots()

    def start_calibration(self):
        if not self.ur_cmd or not self.ur_dash:
            self._show_flyout(InfoBarIcon.ERROR, "Run Error", "Robot is not connected !", self.start)
        elif not self.camera_on:
            self._show_flyout(InfoBarIcon.ERROR, "Run Error", "Camera is not connected !", self.start)
        else:
            self.state = StateToolTip("Calibration", "Calibration is in progress...", self)
            self.state.show()

            # start calibration thread
            self.thread_pool.start(self._calibration_thread)

    def _calibration_thread(self):
        # set DH parameters
        IK_Solver.set_dh_param(self.robot_type.currentText())

        start_pose = self._convert_robot_pose(self.ur_cmd.get_tcp_pose())

        calib_cnt = 0
        executed_pose_list = []
        while calib_cnt < self.calibration_sample:
            new_pose = generate_poses_around_state(start_pose, self.translation_delta, self.angle_delta)
            # computer IK solutions and select the least joint movement pose in joint space
            validated_pose = validate_pose(new_pose, self.ur_cmd.get_joint_positions())
            # check if the pose is repeated
            repeat = is_pose_repeated(validated_pose, executed_pose_list)
            if repeat:
                continue
            self.ur_cmd.movej(validated_pose, 0.4, 0.1)
            self.ur_cmd.wait_arrive_joint(validated_pose)
            executed_pose_list.append(validated_pose)
            time.sleep(0.5)
            # return to previous pose if no target detected
            if not self.M_target_to_camera[0, 0]:
                calib_cnt -= 1
                self.ur_cmd.movej(validated_pose, 0.4, 0.1)
                self.ur_cmd.wait_arrive_joint(validated_pose)
                executed_pose_list.pop()
                continue

            hand_pose = self._convert_robot_pose(self.ur_cmd.get_tcp_pose())
            self.hand.append(hand_pose)
            eye_pose = self._convert_camera_pose(self.M_target_to_camera)
            self.eye.append(eye_pose)
            calib_cnt += 1

        # calculate hand-eye calibration result
        hand_eye_calib = HandeyeCalibration(self.hand, self.eye, method=self.mount_type.currentText())
        res = hand_eye_calib.calibrate()
        np.savetxt("calibration_result.csv", res, delimiter=",")

        self.state.setContent("Calibration is completed !")
        self.state.setState(True)
        self.state = None

    @staticmethod
    def _convert_robot_pose(pose):
        # convert translation part from millimeters to meters
        pose[0:3] = [i / 1000 for i in pose[0:3]]
        pose[3:6] = rv2rpy(pose[3], pose[4], pose[5])
        return pose

    @staticmethod
    def _convert_camera_pose(matrix):
        # covert camera pose from homogeneous matrix to [x, y, z, roll, pitch, yaw]
        eye_translation = [matrix[i, 3] for i in range(3)]
        eye_rotation = [rm2rpy(matrix[:3, :3])[i] for i in range(3)]
        return eye_translation + eye_rotation

    def handle_robot_connection(self):
        if self.ur_dash and self.ur_cmd:
            self.ur_dash.close()
            self.ur_cmd.close()
            self.robot_timer.stop()
            self.ur_dash = None
            self.ur_cmd = None
        else:
            try:
                self.ur_dash = URDashboard(ip=self.robot_ip.text(), port=29999)
                self.ur_cmd = URRTInterface(ip=self.robot_ip.text(), port=30003)
                self.robot_timer.start()
                self._show_flyout(InfoBarIcon.SUCCESS, "Success", "Robot connected successfully !", self.connect_robot)
            except Exception as e:
                self.connect_robot.toggle()
                self._show_flyout(InfoBarIcon.ERROR, "Connection Error", str(e), self.connect_robot)

    def _update_robot_status(self):
        tcp_pose = self.ur_cmd.get_tcp_pose()
        self.robot_x.setText(f"X: {tcp_pose[0]:.2f}")
        self.robot_y.setText(f"Y: {tcp_pose[1]:.2f}")
        self.robot_z.setText(f"Z: {tcp_pose[2]:.2f}")
        rpy = rv2rpy(tcp_pose[3], tcp_pose[4], tcp_pose[5])
        self.robot_rx.setText(f"Rx: {rpy[0]:.3f}")
        self.robot_ry.setText(f"Ry: {rpy[1]:.3f}")
        self.robot_rz.setText(f"Rz: {rpy[2]:.3f}")

    def handle_camera_connection(self):
        if not self.camera_on:
            try:
                self.camera = CameraThread(self.chessboard_x_number,
                                           self.chessboard_y_number,
                                           self.chessboard_cell_size,
                                           self.aruco_tag_size)
                self.camera.camera_image.connect(self._update_camera_image)
                self.camera.M_target_to_camera.connect(self._update_target_pose)
                self.camera.camera_trigger = True
                self.camera.start()
                self.camera_on = True
                self._show_flyout(InfoBarIcon.SUCCESS, "Success", "Camera connected successfully !",
                                  self.connect_camera)
            except Exception as e:
                self.connect_camera.toggle()
                self._show_flyout(InfoBarIcon.ERROR, "Connection Error", str(e), self.connect_camera)
        else:
            if self.camera.isRunning():
                self.camera.camera_trigger = False
                self.camera.quit()
                self.camera_on = False

    def _update_camera_image(self, img):
        image = QImage(img, 640, 480, QImage.Format_BGR888)
        self.PixmapLabel.setPixmap(QPixmap.fromImage(image))

    def _update_target_pose(self, m_target_to_camera):
        self.M_target_to_camera = m_target_to_camera
        self.target_x.setText(f"X: {m_target_to_camera[0, 3]:.3f}")
        self.target_y.setText(f"Y: {m_target_to_camera[1, 3]:.3f}")
        self.target_z.setText(f"Z: {m_target_to_camera[2, 3]:.3f}")
        rpy = rm2rpy(m_target_to_camera[:3, :3])
        self.target_rx.setText(f"Rx: {rpy[0]:.3f}")
        self.target_ry.setText(f"Ry: {rpy[1]:.3f}")
        self.target_rz.setText(f"Rz: {rpy[2]:.3f}")

    def _update_target_type(self):
        if self.target_type.currentText() == "Chessboard":
            self.camera.calibration_method = "Chessboard"
        elif self.target_type.currentText() == "Aruco":
            self.camera.calibration_method = "Aruco"

    def closeEvent(self, event):
        self.robot_timer.stop()
        if self.ur_dash:
            self.ur_dash.close()
        if self.ur_cmd:
            self.ur_cmd.close()
        if self.camera.isRunning():
            self.camera.camera_trigger = False
            self.camera.quit()

    def _bind_slots(self):
        self.connect_robot.clicked.connect(self.handle_robot_connection)
        self.connect_camera.clicked.connect(self.handle_camera_connection)
        self.start.clicked.connect(self.start_calibration)
        self.exit.clicked.connect(self.close)
        self.minimize.clicked.connect(self.showMinimized)
        self.target_type.currentIndexChanged.connect(self._update_target_type)
        self.robot_timer.timeout.connect(self._update_robot_status)

    def _init_window(self):
        self.setWindowTitle("Hand-Eye Calibration")
        self.setWindowFlag(Qt.FramelessWindowHint)

        self._center_window()
        self._add_combobox_items()
        self._add_icons()

    def _add_combobox_items(self):
        self.target_type.addItems(["Chessboard", "Aruco"])
        self.mount_type.addItems(["Eye-in-Hand", "Eye-to-Hand"])
        self.calibration_algorithm.addItems(["OpenCV/Tsai"])
        self.robot_type.addItems(["UR3e", "UR5e", "UR10e"])

    def _add_icons(self):
        self.exit.setIcon(FIF.CLOSE)
        self.minimize.setIcon(FIF.REMOVE)
        self.connect_robot.setIcon(FIF.ROBOT)
        self.connect_camera.setIcon(FIF.VIEW)

    def _start_state_tooltip(self):
        if not self.state:
            self.state = StateToolTip("Calibration", "Calibration started", self)
            self.state.show()

    def _stop_state_tooltip(self):
        if self.state:
            self.state.setContent("Calibration finished")
            self.state.setState(True)
            self.state = None

    def _show_flyout(self, icon, title, content, button):
        Flyout.create(
            icon=icon,
            title=title,
            content=content,
            target=button,
            parent=self,
            aniType=FlyoutAnimationType.PULL_UP
        )

    def _center_window(self):
        desktop = QApplication.desktop().availableGeometry()
        w, h = desktop.width(), desktop.height()
        self.move(w // 2 - self.width() // 2, h // 2 - self.height() // 2)


class CameraThread(QThread):
    camera_trigger = pyqtSignal(bool)
    camera_image = pyqtSignal(np.ndarray)
    M_target_to_camera = pyqtSignal(np.ndarray)
    calibration_method = pyqtSignal(str)
    process_flag = pyqtSignal()

    def __init__(self, chess_x, chess_y, square_size, aruco_size):
        super().__init__()

        self.rs_camera = RSApplication()

        self.calibration_method = "Chessboard"
        self.chess_x = chess_x
        self.chess_y = chess_y
        self.square_size = square_size
        self.aruco_size = aruco_size

    def run(self):
        self.rs_camera.start()
        self.process_flag.emit()
        while True:
            if self.camera_trigger:
                _, img = self.rs_camera.get_frame()
                M_t2c = np.zeros((4, 4))
                M_t2c[3, 3] = 1
                try:
                    if self.calibration_method == "Chessboard":
                        M_t2c, img = self.rs_camera.detect_chessboard(img, self.chess_x, self.chess_y, self.square_size)
                    elif self.calibration_method == "Aruco":
                        M_t2c, img = self.rs_camera.detect_aruco(img, self.aruco_size)
                except Exception as e:
                    cv2.putText(img, str(e), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                    self.M_target_to_camera.emit(M_t2c)
                else:
                    self.M_target_to_camera.emit(M_t2c)
                finally:
                    self.camera_image.emit(img)
            else:
                self.rs_camera.close()
                self.process_flag.emit()
                break
