#!/usr/bin/env python
# coding: utf-8

"""
@author: George Zeng
@contact: george.zeng@syensqo.com
@version: 1.0.0
@file: handeye_robot.py
@time: 2024/6/7 11:38
"""

import numpy as np
import math
import random
from copy import deepcopy
from transformations.transformations import quaternion_multiply, euler_from_quaternion, quaternion_from_euler
from itertools import chain

from universal_robot.ur_ik import IK
from universal_robot.ur_ik_filter import generate_homogenous_matrix, joints_filter
from util.coordinate_trans import rpy2rv


IK_Solver = IK()


def generate_poses_around_state(start_pose, translation_delta, angle_delta):
    """
    start_pose:[x,y,z,rx,ry,rz]
    angle_delta: maximum rotation angle
    translation_delta: maximum translation distance
    """
    basis = np.eye(3)
    angle_rad = math.radians(angle_delta)

    # Rotate ±angle_delta along three axes(2x3=6 different rotation)
    pos_deltas = [quaternion_from_euler(*rot_axis * angle_rad) for rot_axis in basis]
    neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_rad)) for rot_axis in basis]

    quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave

    final_rots = []
    for qd in quaternion_deltas:
        final_rots.append(list(qd))

    # Rotate ±angle_delta/2 along three axes(2x3=6 different rotation, 12 in total)
    pos_deltas = [quaternion_from_euler(*rot_axis * angle_rad / 2) for rot_axis in basis]
    neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_rad / 2)) for rot_axis in basis]

    quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave
    for qd in quaternion_deltas:
        final_rots.append(list(qd))

    # Change the amount of translation randomly, get 12 different final_poses
    final_poses = []
    for rot in final_rots:
        fp = deepcopy(start_pose)
        x, y, z = [random.choice([True, False]) for _ in range(3)]
        if x:
            fp[0] += translation_delta / 2
        else:
            fp[0] -= translation_delta / 2
        if y:
            fp[1] += translation_delta
        else:
            fp[1] -= translation_delta
        if z:
            fp[2] += translation_delta / 3
        else:
            fp[2] -= translation_delta / 3
        # Starting pose Euler angle->quaternion
        ori = quaternion_from_euler(fp[3], fp[4], fp[5], axes='sxyz')
        # The rotation of final_pose is equal to the combination of current_pose_rotation and subsequent rotation
        combined_rot = quaternion_multiply([ori[0], ori[1], ori[2], ori[3]], rot)
        fp[3] = combined_rot[0]
        fp[4] = combined_rot[1]
        fp[5] = combined_rot[2]
        fp.append(combined_rot[3])
        fp[3:] = euler_from_quaternion(fp[3:])
        rpy = np.array(fp[3:])
        rv = rpy2rv(rpy)
        fp[3] = rv[0]
        fp[4] = rv[1]
        fp[5] = rv[2]
        final_poses.append(fp)
    generated_pose = final_poses[random.randint(0, 11)]
    return generated_pose


def validate_pose(generated_pose, current_joint_values):
    M_new_pose = generate_homogenous_matrix(generated_pose)
    # compute the inverse kinematics solutions
    ik_solutions = IK_Solver.invKine_Geometric(M_new_pose)
    valid_poses = joints_filter(current_joint_values, ik_solutions)
    # find the least joint movement solution
    joint_sum_movement = []
    for i in range(8):
        turning_angle = 0
        for j in range(6):
            turning_angle += abs(valid_poses.tolist()[j][i] - current_joint_values[j])
        joint_sum_movement.append(turning_angle)
    min_index = joint_sum_movement.index(min(joint_sum_movement))
    validated_pose = []
    for i in range(6):
        validated_pose.append(valid_poses[i, min_index])
    return validated_pose


def is_pose_repeated(generated_pose, pose_list):
    if pose_list:
        for pose in pose_list:
            if np.allclose(generated_pose, pose, atol=1e-2):
                return True
    return False


