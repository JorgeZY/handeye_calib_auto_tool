#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   ur_api.py
@Time    :   2024/03/08 12:32:27
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""

import socket
import struct
import copy
from typing import List
import numpy as np

from universal_robot.ur_msg import URMsg
from util.coordinate_trans import rpy2rv
from util.log import LogUtil


class URClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_robot = 0

        if self.port == 29999 or self.port == 30003:
            try:
                self.socket_robot = socket.socket()
                self.socket_robot.settimeout(1)
                self.socket_robot.connect((self.ip, self.port))
                self.socket_robot.recv(1024) if self.port == 29999 else None
            except socket.error:
                raise Exception(f"Unable to set socket connection use port {self.port} !")
            else:
                # init logger
                log = LogUtil("API")
                log.logger_init()
                self.logger = log.logger

    def send_data(self, string):
        self.logger.info(f"Send data: {string}")
        self.socket_robot.send(str.encode(string, 'utf-8'))

    def wait_reply(self):
        """
        Read the return value
        """
        data = self.socket_robot.recv(1024)
        data_str = str(data, "utf-8")
        self.logger.info(f"Receive data: {data_str}")
        return data_str

    def close(self):
        """
        Close the port
        """
        if self.socket_robot != 0:
            self.socket_robot.close()

    def __del__(self):
        self.close()


class URDashboard(URClient):
    """
    Define UR Dashboard Server Commands
    
    Official Documentation:
    https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
    """

    def PowerOn(self):
        """
        Power on the robot
        """
        self.send_data("power on\n")
        return self.wait_reply()

    def PowerOff(self):
        """
        Power off the robot
        """
        self.send_data("power off\n")
        return self.wait_reply()

    def BrakeRelease(self):
        """
        Release the brake
        """
        self.send_data("brake release\n")
        return self.wait_reply()

    def UnlockProtectiveStop(self):
        """
        Unlock the protective stop
        """
        self.send_data("unlock protective stop\n")
        return self.wait_reply()

    def CloseSafetyPopup(self):
        """
        Close the safety popup
        """
        self.send_data("close safety popup\n")
        return self.wait_reply()

    def LoadProgram(self, program_name):
        """
        Load the program
        """
        self.send_data(f"load {program_name}.urp\n")
        return self.wait_reply()

    def Play(self):
        """
        Play the program
        """
        self.send_data("play\n")
        return self.wait_reply()

    def Stop(self):
        """
        Stop the program
        """
        self.send_data("stop\n")
        return self.wait_reply()

    def Pause(self):
        """
        Pause the program
        """
        self.send_data("pause\n")
        return self.wait_reply()

    def Quit(self):
        """
        Closes the connection to the robot
        """
        self.send_data("quit\n")
        return self.wait_reply()

    def Shutdown(self):
        """
        Shutdown and turn off the robot and controller
        """
        self.send_data("shutdown\n")
        return self.wait_reply()

    def Running(self):
        """
        Check if the program is running
        """
        self.send_data("running\n")
        return self.wait_reply()

    def RobotMode(self):
        """
        Get the robot mode
        """
        self.send_data("robotmode\n")
        return self.wait_reply()

    def GetLoadedProgram(self):
        """
        Get the loaded program
        """
        self.send_data("get loaded program\n")
        return self.wait_reply()

    def Popup(self, message):
        """
        Show a popup message on the robot's touch screen
        """
        self.send_data(f"popup {message}\n")
        return self.wait_reply()

    def ClosePopup(self):
        """
        Close the popup message on the robot's touch screen
        """
        self.send_data("close popup\n")
        return self.wait_reply()

    def IsProgramSaved(self):
        """
        Check if the active program is saved
        """
        self.send_data("is program saved\n")
        return self.wait_reply()

    def ProgramState(self):
        """
        Get the program state
        """
        self.send_data("program state\n")
        return self.wait_reply()

    def PolyscopeVersion(self):
        """
        Get the Polyscope version
        """
        self.send_data("PolyscopeVersion\n")
        return self.wait_reply()

    def Version(self):
        """
        Get the version number of the UR software
        """
        self.send_data("version\n")
        return self.wait_reply()

    def SafetyStatus(self):
        """
        Get the safety status
        """
        self.send_data("safetystatus\n")
        return self.wait_reply()

    def LoadInstallation(self, installation_name):
        """
        Load the installation
        """
        self.send_data(f"load installation {installation_name}.installation\n")
        return self.wait_reply()


class URRTInterface(URClient):
    """
    Define UR Real-time Interface Commands based on URScript
    
    Official Documentation:
    https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/
    """

    def movej(self, q: List, a, v, t=0, r=0):
        """
        Move to position in joint space
        q: joint position in radians [q1, q2, q3, q4, q5, q6]
        a: joint acceleration of leading axis [rad/s^2]
        v: joint speed of leading axis [rad/s]
        t: time [S]
        r: blend radius [m]
        """
        return self.send_data(f"movej({q}, a={a}, v={v}, t={t}, r={r})\n")

    def movel(self, p: List, a, v, t=0, r=0):
        """
        Move to position in cartesian space
        p: target pose [x, y, z, rv_x, rv_y, rv_z]
        a: acceleration [m/s^2]
        v: speed [m/s]
        t: time [S]
        r: blend radius [m]
        """
        p[:3] = [i/1000 for i in p[:3]]
        rv = rpy2rv(np.array(p[3:]))
        p[3:] = [i for i in rv]
        return self.send_data(f"movel(p{p}, a={a}, v={v}, t={t}, r={r})\n")

    def servoj(self, q: List, t, lookahead_time, gain, a=0, v=0):
        """
        Online realtime control of joint positions.
        q: joint position in radians [q1, q2, q3, q4, q5, q6]
        #//a: not used in current version
        #//v: jnot used in current version
        t: time [S], where the command is controlling the robot
        lookahead_time: time [S], range[0.03,0.2] smoothens the trajectory with this lookahead time
        gain: proportional gain for following target position, range[100,2000]
        """
        return self.send_data(f"servoj({q}, a={a}, v={v}, t={t}, lookahead_time={lookahead_time}, gain={gain})\n")

    def speedj(self, qd: List, a, t=0):
        """
        Accelerate linearly in joint space and continue with constant joint speed
        qd: joint speed in radians [qd1, qd2, qd3, qd4, qd5, qd6]
        a: joint acceleration of leading axis [rad/s^2]
        t: time [S] before the function returns (optional)
        """
        return self.send_data(f"speedj({qd}, a={a}, t={t})\n")

    def stopj(self, a):
        """
        Decelerate joint speeds to zero
        a: joint acceleration of leading axis [rad/s^2]
        """
        return self.send_data(f"stopj({a})\n")

    def set_tcp(self, p: List):
        """
        Set the TCP offset
        p: a pose describing the transformation [x, y, z, rv_x, rv_y, rv_z]
        """
        p[:3] = [i/1000 for i in p[:3]]
        return self.send_data(f"set_tcp(p{p})\n")

    def get_joint_positions(self) -> List:
        """
        Get the joint positions of the robot in radians 
        [q1, q2, q3, q4, q5, q6]
        """
        return list(self._unpack_message()["q actual"])

    def get_tcp_pose(self) -> List:
        """
        Get the pose of the robot 
        [x, y, z, rv_x, rv_y, rv_z] 
        """
        tcp_pose = list(self._unpack_message()["Tool vector actual"])
        tcp_pose[:3] = [round(i * 1000, 2) for i in tcp_pose[:3]]
        return tcp_pose

    def get_robot_mode(self):
        """
        Get the robot mode
        """
        robot_mode = int(self._unpack_message()["Robot Mode"][0])
        return robot_mode

    def wait_arrive_joint(self, q: List):
        """
        Wait for the robot to arrive at the target joint position
        """
        while True:
            is_arrive = True

            current_q = self.get_joint_positions()
            for index in range(6):
                if abs(current_q[index] - q[index]) > 0.02:
                    is_arrive = False

            if is_arrive:
                return

    def wait_arrive_cartesian(self, pose: List):
        """
        Wait for the robot to arrive at the target pose
        """
        while True:
            is_arrive = True

            current_pose = self.get_tcp_pose()
            rot = rpy2rv(np.array(pose[3:]))
            for index in range(3):
                if abs(current_pose[index] - pose[index]) > 0.5 or abs(current_pose[index+3] - rot[index]) > 0.02:
                    is_arrive = False

            if is_arrive:
                return

    def _unpack_message(self):
        """
        Unpack the message from the robot
        """
        has_read = 0
        data = bytes()
        while has_read < 1220:
            temp = self.socket_robot.recv(1220 - has_read)
            if len(temp) > 0:
                has_read += len(temp)
                data += temp
        msg = copy.deepcopy(URMsg)
        names = []
        msg_size = range(len(msg))
        for key, _ in zip(msg, msg_size):
            fmtsize = struct.calcsize(msg[key])
            block, data = data[0:fmtsize], data[fmtsize:]
            fmt = "!" + msg[key]
            names.append(struct.unpack(fmt, block))
            msg[key] = struct.unpack(fmt, block)
        return msg
