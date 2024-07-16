#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   ur_msg.py
@Time    :   2024/03/08 14:10:31
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""

from dataclasses import dataclass


URMsg = {
    "Message Size": "i",
    "Time": "d",
    "q target": "6d",
    "qd target": "6d",
    "qdd target": "6d",
    "I target": "6d",
    "M target": "6d",
    "q actual": "6d",
    "qd actual": "6d",
    "I actual": "6d",
    "I control": "6d",
    "Tool vector actual": "6d",
    "TCP speed actual": "6d",
    "TCP force": "6d",
    "Tool vector target": "6d",
    "TCP speed target": "6d",
    "Digital input bits": "d",
    "Motor temperatures": "6d",
    "Controller Timer": "d",
    "Test value": "d",
    "Robot Mode": "d",
    "Joint Modes": "6d",
    "Safety Mode": "d",
    "empty1": "6d",
    "Tool Accelerometer values": "3d",
    "empty2": "6d",
    "Speed scaling": "d",
    "Linear momentum norm": "d",
    "SoftwareOnly": "d",
    "softwareOnly2": "d",
    "V main": "d",
    "V robot": "d",
    "I robot": "d",
    "V actual": "6d",
    "Digital outputs": "d",
    "Program state": "d",
    "Elbow position": "3d",
    "Elbow velocity": "3d",
    "Safety Status": "d",
    "empty3": "d",
    "empty4": "d",
    "empty5": "d",
    "Payload Mass": "d",
    "Payload CoG": "3d",
    "Payload Inertia": "6d",
}


@dataclass
class URRobotMode:
    ROBOT_MODE_NO_CONTROLLER: int = -1
    ROBOT_MODE_DISCONNECTED: int = 0
    ROBOT_MODE_CONFIRM_SAFETY: int = 1
    ROBOT_MODE_BOOTING: int = 2
    ROBOT_MODE_POWER_OFF: int = 3
    ROBOT_MODE_POWER_ON: int = 4
    ROBOT_MODE_IDLE: int = 5
    ROBOT_MODE_BACKDRIVE: int = 6
    ROBOT_MODE_RUNNING: int = 7
    ROBOT_MODE_UPDATING_FIRMWARE: int = 8
