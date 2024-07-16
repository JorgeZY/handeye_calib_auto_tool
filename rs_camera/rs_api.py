#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   rs_api.py
@Time    :   2024/03/10 14:39:52
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""

import pyrealsense2 as rs
import numpy as np

from util.log import LogUtil


class RSCamera:
    def __init__(self):
        ctx = rs.context()
        if len(ctx.query_devices()) == 0:
            raise Exception("No device is connected. Please connect a RealSense device !")

        self.pipeline = rs.pipeline(ctx)
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        if not self.config.can_resolve(self.pipeline):
            raise Exception("Can't resolve the pipeline configuration !")

        # init logger
        log = LogUtil("API")
        log.logger_init()
        self.logger = log.logger

    def start(self):
        self.pipeline.start(self.config)

    def get_frame(self):
        """
        Get the depth and color image from the camera
        """
        frames = self.pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return None, color_image

    def get_intrinsics(self):
        """
        Get the camera intrinsics
        """
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        distortion = np.array(intr.coeffs)
        return camera_matrix, distortion
    
    def close(self):
        self.pipeline.stop()


