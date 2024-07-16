#!/usr/bin/env python
# coding: utf-8

"""
@author: George Zeng
@contact: george.zeng@syensqo.com
@version: 1.0.0
@file: rs_application.py
@time: 2024/5/27 13:32
"""

import cv2
import numpy as np

from rs_camera.rs_api import RSCamera


class RSApplication(RSCamera):
    """
    Algorithm implementation using the RealSense camera
    """

    def detect_chessboard(self, color_image, chessboard_x_num, chessboard_y_num, chess_cell_size, display=False):
        """
        Solve the PnP problem to get the transformation matrix from the camera to the chessboard
        """
        camera_matrix, distortion = self.get_intrinsics()
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (chessboard_x_num, chessboard_y_num), None)

        if ret:
            cv2.drawChessboardCorners(color_image, (chessboard_x_num, chessboard_y_num), corners, ret)
            if display:
                cv2.imshow("Chessboard", color_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            corner_points = np.zeros((2, corners.shape[0]), dtype=np.float64)
            for i in range(corners.shape[0]):
                corner_points[:, i] = corners[i, 0, :]
            object_points = np.zeros((3, chessboard_x_num * chessboard_y_num), dtype=np.float64)
            count = 0
            for i in range(chessboard_y_num):
                for j in range(chessboard_x_num):
                    object_points[:2, count] = np.array(
                        [(chessboard_x_num - j - 1) * chess_cell_size,
                         (chessboard_y_num - i - 1) * chess_cell_size])
                    count += 1
            _, rvec, tvec = cv2.solvePnP(object_points.T, corner_points.T, camera_matrix, distCoeffs=distortion)
            M_chessboard_to_camera = np.column_stack(((cv2.Rodrigues(rvec))[0], tvec))
            M_chessboard_to_camera = np.row_stack((M_chessboard_to_camera, np.array([0, 0, 0, 1])))
            return M_chessboard_to_camera, color_image
        else:
            raise Exception("Chessboard not detected !")

    def detect_aruco(self, color_image, aruco_size, aruco_dict=cv2.aruco.DICT_ARUCO_ORIGINAL, display=False):
        """
        Detect the aruco marker and get the transformation matrix from the camera to the aruco marker
        """
        camera_matrix, distortion = self.get_intrinsics()
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(color_image, aruco_dict, parameters=parameters)
        if isinstance(ids, np.ndarray) and len(ids) > 1:
            raise Exception("More than one aruco marker detected !")
        else:
            if ids:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix,
                                                                    distCoeffs=distortion)
                cv2.aruco.drawAxis(color_image, camera_matrix, distortion, rvec[0], tvec[0],
                                   0.03)
                if display:
                    cv2.imshow("Aruco", color_image)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                M_aruco_to_camera = np.column_stack(((cv2.Rodrigues(rvec[0].T))[0], tvec[0].T))
                M_aruco_to_camera = np.row_stack((M_aruco_to_camera, np.array([0, 0, 0, 1])))
                M_aruco_to_camera[0, 3] *= 1000
                M_aruco_to_camera[1, 3] *= 1000
                M_aruco_to_camera[2, 3] *= 1000
                return M_aruco_to_camera, color_image
            else:
                raise Exception("Aruco marker not detected !")
