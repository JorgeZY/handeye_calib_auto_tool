#!/usr/bin/env python
# coding: utf-8

import copy
import numpy as np
from transformations import quaternion_matrix, translation_matrix
from util.coordinate_trans import rv2rm
from universal_robot.ur_ik import IK


def generate_homogenous_matrix(pose):
    """
    :param pose: list includes [x,y,z,rx,ry,rz] or [x,y,z,rx,ry,rz,rw]
    :return: desire pose in homogenous matrix
    """
    if len(pose) < 7:
        rxyz = rv2rm(pose[3], pose[4], pose[5])
        xyz = np.array([pose[0], pose[1], pose[2]])
        hm = np.array([0, 0, 0, 1])
        t_pose = np.insert(rxyz, 3, values=xyz, axis=1)
        desire_pose = np.row_stack((t_pose, hm))
        return desire_pose
    else:
        q_matrix = quaternion_matrix(pose[3:])
        t_matrix = translation_matrix(pose[:3])
        rt_matrix = np.dot(t_matrix, q_matrix)
        return rt_matrix


def joints_filter(current_joint_values, ik_solutions):
    """
    :param current_joint_values: list includes [r1, r2, r3, r4, r5, r6]
    :param ik_solutions: inverse kinematics solutions (6X8 matrix)
    :return: new ik solutions considering turn limit and accessibility
    """
    new_ik_solutions = copy.deepcopy(ik_solutions)

    # limit elbow joint from -pi to pi due to collision
    for i in range(0, 8):
        if new_ik_solutions[2, i] > np.pi:
            new_ik_solutions[2, i] -= 2 * np.pi
        elif new_ik_solutions[2, i] < -np.pi:
            new_ik_solutions[2, i] += 2 * np.pi

    # limit joints turning angles
    for i in range(0, 6):
        for j in range(0, 8):
            turning_angles = abs(new_ik_solutions[i, j] - current_joint_values[i])
            if turning_angles > np.pi:
                if new_ik_solutions[i, j] > 0 and (new_ik_solutions[i, j] - 2 * np.pi) > -2 * np.pi:
                    new_ik_solutions[i, j] -= 2 * np.pi
                elif new_ik_solutions[i, j] < 0 and (new_ik_solutions[i, j] + 2 * np.pi) < 2 * np.pi:
                    new_ik_solutions[i, j] += 2 * np.pi
                else:
                    pass

    return new_ik_solutions


class IkFilter:
    def __init__(self, current_pose, current_joint_values, target_pose, target_joint_values):
        ik_solver = IK()
        self.current_rt_matrix = generate_homogenous_matrix(current_pose)
        self.current_ik_solutions = joints_filter(current_joint_values,
                                                  ik_solver.invKine_Analytic(self.current_rt_matrix))
        self.target_rt_matrix = generate_homogenous_matrix(target_pose)
        self.target_ik_solutions = joints_filter(target_joint_values,
                                                 ik_solver.invKine_Analytic(self.current_rt_matrix))

    @staticmethod
    def find_solution_type(ik_solutions, target_ik):
        ik_type = []
        # define shoulder position type
        avg = ik_solutions[0].sum() / ik_solutions.shape[1]
        if target_ik[0] < avg:
            ik_type.append("R")
        else:
            ik_type.append("L")

        # define elbow position type
        if target_ik[2] >= 0:
            ik_type.append("U")
        elif target_ik[2] < 0:
            ik_type.append("D")

        # define wrist position type
        if -np.pi < target_ik[4] < 0:
            ik_type.append("D")
        elif np.pi < target_ik[4] < 2 * np.pi:
            ik_type.append("U")
        if 0 < target_ik[4] < np.pi:
            ik_type.append("U")
        elif -2 * np.pi < target_ik[4] < -np.pi:
            ik_type.append("U")

        # define elbow position
        # if ik_type[0] == "L":
        #     if target_ik[1] > -np.pi/2 and target_ik[2] < 0:
        #         ik_type.append("D")
        #     elif target_ik[1] < -np.pi/2 and target_ik[2] < 0:
        #         ik_type.append("D")
        #     elif target_ik[1] > -np.pi/2 and target_ik[2] > 0:
        #         ik_type.append("U")
        #     elif target_ik[1] < -np.pi/2 and target_ik[2] > 0:
        #         ik_type.append("U")
        # elif ik_type[0] == "R":
        #     if target_ik[1] > -np.pi/2 and target_ik[2] < 0:
        #         ik_type.append("U")
        #     elif target_ik[1] < -np.pi/2 and target_ik[2] < 0:
        #         ik_type.append("U")
        #     elif target_ik[1] < -np.pi/2 and target_ik[2] > 0:
        #         ik_type.append("D")
        #     elif target_ik[1] > -np.pi/2 and target_ik[2] > 0:
        #         ik_type.append("D")

        # define wrist position
        # if np.pi > target_ik[3] > 0:
        #     ik_type.append("D")
        # elif -np.pi > target_ik[3] > -2*np.pi:
        #     ik_type.append("D")
        # else:
        #     ik_type.append("U")

        return "".join(ik_type)

    # 8 ik solutions
    # shoulder-Right/Left
    # elbow-Up/Down
    # wrist-Up/Down
    def solution_filter(self, ik_solutions, select_type):
        """
        :param ik_solutions: inverse kinematics solutions (6x8 matrix)
        :param select_type: define poses according to ik solutions, examples: RUD-shoulderRight/elbowUp/wristDown
        :return: list: target joint values
        """
        for row in range(0, 6):
            if ik_solutions[row].any() == 0:
                raise Exception("No available IK solutions")

        target_joints = []
        for i in range(0, 8):
            for j in range(0, 6):
                poses = ik_solutions[j, i]
                target_joints.append(poses)
        for k in range(0, 48, 6):
            ik = target_joints[k:k + 6]
            ik_type = self.find_solution_type(ik_solutions, ik)
            if ik_type == select_type:
                return ik
