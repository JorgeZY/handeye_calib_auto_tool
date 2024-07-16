#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   handeye_calib.py
@Time    :   2024/03/07 14:59:15
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""


import transforms3d as tfs
import numpy as np
import math


# convert to homogenous matrix
def get_matrix_eular_radu(x, y, z, rx, ry, rz):
    rmat = tfs.euler.euler2mat(math.radians(rx), math.radians(ry), math.radians(rz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), rmat, [1, 1, 1])
    return rmat


# compute the skewness of the data set
def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def rot2quat_minimal(m):
    quat = tfs.quaternions.mat2quat(m[0:3, 0:3])
    return quat[1:]


def quat_minimal2rot(q):
    p = np.dot(q.T, q)
    w = np.sqrt(np.subtract(1, p[0][0]))
    return tfs.quaternions.quat2mat([w, q[0], q[1], q[2]])



class HandEyeCalib:
    def __init__(self, hand, camera):
        self.Hgs = []
        self.Hcs = []
        self.Hgijs = []
        self.Hcijs = []
        self.A = []
        self.B = []
        self.hand = hand
        self.camera = camera

    def calibrate_hand_eye(self):
        for i in range(0, len(self.hand), 6):
            self.Hgs.append(get_matrix_eular_radu(self.hand[i], self.hand[i + 1], self.hand[i + 2], self.hand[i + 3], self.hand[i + 4], self.hand[i + 5]))
            self.Hcs.append(get_matrix_eular_radu(self.camera[i], self.camera[i + 1], self.camera[i + 2], self.camera[i + 3], self.camera[i + 4],self.camera[i + 5]))
        # Hcij = Hcj*inv(Hci)
        # Hgij = inv(Hgj)*Hgi
        # quaternion to rotation vector --> q = [cos(theta/2), nx*sin(theta/2), ny*sin(theta/2), nz*sin(theta/2)]^T
        # rotation vector to quaternion --> theta = 2arcos(q0); [nx, ny, nz]^T = [q1, q2, q3]^T / sin(theta/2)
        # Pr = 2*sin(theta/2)*[nx, ny, nz]^T
        size = 0
        for i in range(len(self.Hgs)):
            for j in range(i + 1, len(self.Hgs)):
                size += 1
                Hgij = np.dot(np.linalg.inv(self.Hgs[j]), self.Hgs[i])
                self.Hgijs.append(Hgij)
                Pgij = np.dot(2, rot2quat_minimal(Hgij))

                Hcij = np.dot(self.Hcs[j], np.linalg.inv(self.Hcs[i]))
                self.Hcijs.append(Hcij)
                Pcij = np.dot(2, rot2quat_minimal(Hcij))

                self.A.append(skew(np.add(Pgij, Pcij)))
                self.B.append(np.subtract(Pcij, Pgij))

        MA = np.asarray(self.A).reshape(size * 3, 3)
        MB = np.asarray(self.B).reshape(size * 3, 1)
        Pcg_ = np.dot(np.linalg.pinv(MA), MB)
        pcg_norm = np.dot(np.conjugate(Pcg_).T, Pcg_)
        Pcg = np.sqrt(np.add(1, np.dot(Pcg_.T, Pcg_)))
        Pcg = np.dot(np.dot(2, Pcg_), np.linalg.inv(Pcg))
        Rcg = quat_minimal2rot(np.divide(Pcg, 2)).reshape(3, 3)

        self.A.clear()
        self.B.clear()
        id = 0
        for i in range(len(self.Hgs)):
            for j in range(i + 1, len(self.Hgs)):
                Hgij = self.Hgijs[id]
                Hcij = self.Hcijs[id]
                self.A.append(np.subtract(Hgij[0:3, 0:3], np.eye(3, 3)))
                self.B.append(np.subtract(np.dot(Rcg, Hcij[0:3, 3:4]), Hgij[0:3, 3:4]))
                id += 1

        MA = np.asarray(self.A).reshape(size * 3, 3)
        MB = np.asarray(self.B).reshape(size * 3, 1)
        Tcg = np.dot(np.linalg.pinv(MA), MB).reshape(3, )

        return tfs.affines.compose(Tcg, np.squeeze(Rcg), [1, 1, 1])


if __name__ == "__main__":
    hand = [1.1988093940033604, -0.42405585264804424, 0.18828251788562061, 151.3390418721659, -18.612399542280507,
            153.05074895025035,
            1.1684831621733476, -0.183273375514656, 0.12744868246620855, -161.57083804238462, 9.07159838346732,
            89.1641128844487,
            1.1508343174145468, -0.22694301453461405, 0.26625166858469146, 177.8815855486261, 0.8991159570568988,
            77.67286224959672]
    camera = [-0.16249272227287292, -0.047310635447502136, 0.4077761471271515, -56.98037030812389, -6.16739631361851,
              -115.84333735802369,
              0.03955405578017235, -0.013497642241418362, 0.33975949883461, -100.87129330834215, -17.192685528625265,
              -173.07354634882094,
              -0.08517949283123016, 0.00957852229475975, 0.46546608209609985, -90.85270962096058, 0.9315977976503153,
              175.2059707654342]

    calib = HandEyeCalib(hand, camera)
    result = calib.calibrate_hand_eye()
    print(result)

