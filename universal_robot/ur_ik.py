#!/usr/bin/env python
# coding: utf-8

import numpy as np
from numpy import linalg

import cmath
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

mat = np.matrix


# ****** Coefficients ******
# Universal Robot DH parameters
class UR3e:
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    alpha = [pi / 2, 0, 0, pi / 2, -pi / 2, 0]


class UR5e:
    a = [0, -0.425, -0.3922, 0, 0, 0]
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    alpha = [pi / 2, 0, 0, pi / 2, -pi / 2, 0]


class UR10e:
    a = [0, -0.6127, -0.57155, 0, 0, 0]
    d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
    alpha = [pi / 2, 0, 0, pi / 2, -pi / 2, 0]


class IK:
    def __init__(self):
        self.d1 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0
        self.a2 = 0.0
        self.a3 = 0.0

        self.d = mat([])
        self.a = mat([])
        self.alph = mat([])

    def set_dh_param(self, robot_type):
        if robot_type == "UR3e":
            self._set_params(UR3e)
        elif robot_type == "UR5e":
            self._set_params(UR5e)
        elif robot_type == "UR10e":
            self._set_params(UR10e)
        else:
            raise ValueError("Invalid Robot Type!")

    def _set_params(self, dh_param):
        self.d = mat(dh_param.d)
        self.a = mat(dh_param.a)
        self.alph = mat(dh_param.alpha)
        self.d1 = dh_param.d[0]
        self.d4 = dh_param.d[3]
        self.d5 = dh_param.d[4]
        self.d6 = dh_param.d[5]
        self.a2 = dh_param.a[1]
        self.a3 = dh_param.a[2]

    # FORWARD KINEMATICS
    # (n-1)T(n) matrix

    def AH(self, n, th, c):
        T_a = mat(np.identity(4), copy=False)
        T_a[0, 3] = self.a[0, n - 1]
        T_d = mat(np.identity(4), copy=False)
        T_d[2, 3] = self.d[0, n - 1]

        Rzt = mat([[cos(th[n - 1, c]), -sin(th[n - 1, c]), 0, 0],
                   [sin(th[n - 1, c]), cos(th[n - 1, c]), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]], copy=False)

        Rxa = mat([[1, 0, 0, 0],
                   [0, cos(self.alph[0, n - 1]), -sin(self.alph[0, n - 1]), 0],
                   [0, sin(self.alph[0, n - 1]), cos(self.alph[0, n - 1]), 0],
                   [0, 0, 0, 1]], copy=False)

        A_i = T_d * Rzt * T_a * Rxa

        return A_i

    """
    [cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)    a*cos(theta)
     sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)   a*sin(theta)
         0              sin(alpha)              cos(alpha)             d
         0                  0                       0                  1       ]
    """

    def HTrans(self, th, c):
        A_1 = self.AH(1, th, c)
        A_2 = self.AH(2, th, c)
        A_3 = self.AH(3, th, c)
        A_4 = self.AH(4, th, c)
        A_5 = self.AH(5, th, c)
        A_6 = self.AH(6, th, c)

        T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

        return T_06

    # INVERSE KINEMATICS - Geometric
    def invKine_Geometric(self, desired_pos):  # T60
        th = mat(np.zeros((6, 8)))
        P_05 = (desired_pos * mat([0, 0, -self.d6, 1]).T - mat([0, 0, 0, 1]).T)

        # **** theta1 ****

        psi = atan2(P_05[2 - 1, 0], P_05[1 - 1, 0])
        phi = acos(self.d4 / sqrt(P_05[2 - 1, 0] * P_05[2 - 1, 0] + P_05[1 - 1, 0] * P_05[1 - 1, 0]))
        # The two solutions for theta1 correspond to the shoulder
        # being either left or right
        th[0, 0:4] = pi / 2 + psi + phi
        th[0, 4:8] = pi / 2 + psi - phi
        th = th.real

        # **** theta5 ****

        cl = [0, 4]  # wrist up or down
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = T_10 * desired_pos
            th[4, c:c + 2] = + acos((T_16[2, 3] - self.d4) / self.d6)
            th[4, c + 2:c + 4] = - acos((T_16[2, 3] - self.d4) / self.d6)

        th = th.real

        # **** theta6 ****
        # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_16 = linalg.inv(T_10 * desired_pos)
            th[5, c:c + 2] = atan2((-T_16[1, 2] / sin(th[4, c])), (T_16[0, 2] / sin(th[4, c])))

        th = th.real

        # **** theta3 ****
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = self.AH(6, th, c)
            T_54 = self.AH(5, th, c)
            T_14 = (T_10 * desired_pos) * linalg.inv(T_54 * T_65)
            P_13 = T_14 * mat([0, -self.d4, 0, 1]).T - mat([0, 0, 0, 1]).T
            t3 = cmath.acos((linalg.norm(P_13) ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))  # norm ?
            th[2, c] = t3.real
            th[2, c + 1] = -t3.real

        # **** theta2 and theta 4 ****

        cl = [0, 1, 2, 3, 4, 5, 6, 7]
        for i in range(0, len(cl)):
            c = cl[i]
            T_10 = linalg.inv(self.AH(1, th, c))
            T_65 = linalg.inv(self.AH(6, th, c))
            T_54 = linalg.inv(self.AH(5, th, c))
            T_14 = (T_10 * desired_pos) * T_65 * T_54
            P_13 = T_14 * mat([0, -self.d4, 0, 1]).T - mat([0, 0, 0, 1]).T

            # theta 2
            th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(self.a3 * sin(th[2, c]) / linalg.norm(P_13))
            # theta 4
            T_32 = linalg.inv(self.AH(3, th, c))
            T_21 = linalg.inv(self.AH(2, th, c))
            T_34 = T_32 * T_21 * T_14
            th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
        th = th.real

        return th

    # INVERSE KINEMATICS - Analytic

    """
    T = [nx ox ax px
         ny oy ay py
         nz oz az pz 
         0  0  0  1  ]
    """

    def invKine_Analytic(self, desire_pose):
        th = mat(np.zeros((6, 8)))

        # **** theta 1 ****
        m1 = self.d6 * desire_pose[1, 2] - desire_pose[1, 3]
        n1 = desire_pose[0, 2] * self.d6 - desire_pose[0, 3]
        th[0, 0:4] = atan2(m1, n1) - atan2(self.d4, sqrt(m1 ** 2 + n1 ** 2 - self.d4 ** 2))
        th[0, 4:8] = atan2(m1, n1) - atan2(self.d4, -sqrt(m1 ** 2 + n1 ** 2 - self.d4 ** 2))

        # **** theta 5 ****
        cl = [0, 4]
        for i in range(0, len(cl)):
            c = cl[i]
            filter_t5 = desire_pose[0, 2] * sin(th[0, c]) - desire_pose[1, 2] * cos(th[0, c])
            if filter_t5 <= 1:
                th[4, c:c + 2] = acos(filter_t5)
                th[4, c + 2:c + 4] = -acos(filter_t5)

        # **** theta 6 ****
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            m6 = desire_pose[0, 0] * sin(th[0, c]) - desire_pose[1, 0] * cos(th[0, c])
            n6 = desire_pose[0, 1] * sin(th[0, c]) - desire_pose[1, 1] * cos(th[0, c])
            th[5, c:c + 2] = atan2(m6, n6) - atan2(sin(th[4, c]), 0)

        # **** theta 3 ****
        cl = [0, 2, 4, 6]
        for i in range(0, len(cl)):
            c = cl[i]
            m3 = self.d5 * (sin(th[5, c]) * (desire_pose[0, 0] * cos(th[0, c]) + desire_pose[1, 0] * sin(th[0, c])) +
                            cos(th[5, c]) * (desire_pose[0, 1] * cos(th[0, c]) + desire_pose[1, 1] * sin(th[0, c]))
                            ) - self.d6 * (desire_pose[0, 2] * cos(th[0, c]) + desire_pose[1, 2] * sin(th[0, c])) + \
                 desire_pose[0, 3] * cos(th[0, c]) + desire_pose[1, 3] * sin(th[0, c])
            n3 = desire_pose[2, 3] - self.d1 - desire_pose[2, 2] * self.d6 + self.d5 * (
                    desire_pose[2, 1] * cos(th[5, c]) + desire_pose[2, 0] * sin(th[5, c]))
            if (m3 ** 2 + n3 ** 2) <= (self.a2 + self.a3) ** 2:
                th[2, c] = acos((m3 ** 2 + n3 ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
                th[2, c + 1] = -acos((m3 ** 2 + n3 ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))

        # **** theta 2 ****
        cl = [0, 1, 2, 3, 4, 5, 6, 7]
        for i in range(0, len(cl)):
            c = cl[i]
            m3 = self.d5 * (sin(th[5, c]) * (desire_pose[0, 0] * cos(th[0, c]) + desire_pose[1, 0] * sin(th[0, c])) +
                            cos(th[5, c]) * (desire_pose[0, 1] * cos(th[0, c]) + desire_pose[1, 1] * sin(th[0, c]))
                            ) - self.d6 * (desire_pose[0, 2] * cos(th[0, c]) + desire_pose[1, 2] * sin(th[0, c])) + \
                 desire_pose[0, 3] * cos(th[0, c]) + desire_pose[1, 3] * sin(th[0, c])
            n3 = desire_pose[2, 3] - self.d1 - desire_pose[2, 2] * self.d6 + self.d5 * (
                    desire_pose[2, 1] * cos(th[5, c]) + desire_pose[2, 0] * sin(th[5, c]))

            s2 = ((self.a3 * cos(th[2, c]) + self.a2) * n3 - self.a3 * sin(th[2, c]) * m3) / (
                    self.a2 ** 2 + self.a3 ** 2 + 2 * self.a2 * self.a3 * cos(th[2, c]))
            c2 = (m3 + self.a3 * sin(th[2, c]) * s2) / (self.a3 * cos(th[2, c]) + self.a2)
            th[1, c] = atan2(s2, c2)

        # **** theta 4 ****
        for i in range(0, len(cl)):
            c = cl[i]
            s234 = -sin(th[5, c]) * (desire_pose[0, 0] * cos(th[0, c]) + desire_pose[1, 0] * sin(th[0, c])) - cos(
                th[5, c]) * (desire_pose[0, 1] * cos(th[0, c]) + desire_pose[1, 1] * sin(th[0, c]))
            c234 = desire_pose[2, 1] * cos(th[5, c]) + desire_pose[2, 0] * sin(th[5, c])
            th[3, c] = atan2(s234, c234) - th[1, c] - th[2, c]

        return th
