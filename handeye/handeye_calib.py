#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   handeye_calib.py
@Time    :   2024/03/08 10:00:06
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""

import cv2
import numpy as np
import transforms3d as tfs


class HandeyeCalibration:
    def __init__(self, hand, eye, method="Eye-in-Hand"):
        self.hand = hand
        self.eye = eye
        self.method = method
    
    def calibrate(self):
        if self.method == "Eye-in-Hand":
            R_gripper_to_base, T_gripper_to_base = self._gripper_to_base()
            R_target_to_camera, T_target_to_camera = self._target_to_camera()
            R_camera_to_gripper, T_camera_to_gripper = cv2.calibrateHandEye(R_gripper_to_base, T_gripper_to_base, R_target_to_camera, T_target_to_camera, cv2.CALIB_HAND_EYE_TSAI)
            return tfs.affines.compose(np.squeeze(T_camera_to_gripper), R_camera_to_gripper, [1, 1, 1])
        elif self.method == "Eye-to-Hand":
            R_base_to_giripper, T_base_to_giripper = self._base_to_gripper()
            R_target_to_camera, T_target_to_camera = self._target_to_camera()
            R_camera_to_base, T_camera_to_base = cv2.calibrateHandEye(R_base_to_giripper, T_base_to_giripper, R_target_to_camera, T_target_to_camera, cv2.CALIB_HAND_EYE_TSAI)
            return tfs.affines.compose(np.squeeze(T_camera_to_base), R_camera_to_base, [1, 1, 1])
        
    def _target_to_camera(self):
        R_target_to_camera = []
        T_target_to_camera = []
        for i in self.eye:
            M_target_to_camera = tfs.euler.euler2mat(i[3], i[4], i[5])
            M_target_to_camera = tfs.affines.compose(i[0:3], M_target_to_camera, [1, 1, 1])
            R_target_to_camera.append(M_target_to_camera[0:3, 0:3])
            T_target_to_camera.append(M_target_to_camera[0:3, 3])
        return R_target_to_camera, T_target_to_camera
        
    def _gripper_to_base(self):
        R_gripper_to_base = []
        T_gripper_to_base = []
        for i in self.hand:
            M_gripper_to_base = tfs.euler.euler2mat(i[3], i[4], i[5])
            M_gripper_to_base = tfs.affines.compose(i[0:3], M_gripper_to_base, [1, 1, 1])
            R_gripper_to_base.append(M_gripper_to_base[0:3, 0:3])
            T_gripper_to_base.append(M_gripper_to_base[0:3, 3])
        return R_gripper_to_base, T_gripper_to_base

    def _base_to_gripper(self):
        R_base_to_gripper = []
        T_base_to_gripper = []
        for i in self.hand:
            M_base_to_gripper = tfs.euler.euler2mat(i[3], i[4], i[5])
            M_base_to_gripper = tfs.affines.compose(i[0:3], M_base_to_gripper, [1, 1, 1])
            R_base_to_gripper.append(M_base_to_gripper[0:3, 0:3])
            T_base_to_gripper.append(M_base_to_gripper[0:3, 3])
        return R_base_to_gripper, T_base_to_gripper
