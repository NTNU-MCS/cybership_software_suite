#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no
# Year: 2022
# Copyright (C) 2023 NTNU Marine Cybernetics Laboratory

import numpy as np
from scipy import optimize
import math


def rad2pipi(x):
    y = np.arctan2(np.sin(x), np.cos(x))
    return y


def ssa(angle):
    return np.mod(angle + np.pi, 2*np.pi) - np.pi


def Rzyx(psi):
    """
    Rzyx(psi) computes the rotation matrix, R in SO(3), using the
    zyx convention and Euler angle representation.
    """

    R = np.array([[math.cos(psi), -math.sin(psi), 0],
                  [math.sin(psi), math.cos(psi), 0],
                  [0, 0, 1]])
    return R


def R2(psi):
    R2 = np.array([[math.cos(psi), -math.sin(psi)],
                  [math.sin(psi), math.cos(psi)]])
    return R2


def yaw2quat(psi):
    """
    Return the quternions of yaw
    """
    q1 = np.cos(psi/2)
    q4 = np.sin(psi/2)
    quat = np.array([q1, 0, 0, q4])
    return quat

# def quat2eul(w, x, y, z):
#    """
#    Returns the ZYX roll-pitch-yaw angles from a quaternion.
#    """
#    q = np.array((w, x, y, z))
#    #if np.abs(np.linalg.norm(q) - 1) > 1e-6:
#    #   raise RuntimeError('Norm of the quaternion must be equal to 1')
#
#    eta = q[0]
#    eps = q[1:]
#
#    S = np.array([
#        [0, -eps[2], eps[1]],
#        [eps[2], 0, -eps[0]],
#        [-eps[1], eps[0], 0]
#    ])
#
#    R = np.eye(3) + 2 * eta * S + 2 * np.linalg.matrix_power(S, 2)
#
#    if np.abs(R[2, 0]) > 1.0:
#        raise RuntimeError('Solution is singular for pitch of +- 90 degrees')
#
#    roll = np.arctan2(R[2, 1], R[2, 2])
#    pitch = -np.arcsin(R[2, 0])
#    yaw = np.arctan2(R[1, 0], R[0, 0])
#
#    return np.array([roll, pitch, yaw])

def quat2eul(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


def string2array(string):
    """
    dynamic_reconfigure does not handle arrays, so gains like L1 or KP are strings on the form "x11,x12,x13"
    in the server to limit the number of variables. This function converts the string into a numpy array when
    they are retrieved. Very scuffed, but if it works, it works :^)
    """
    return np.array(list(map(float, string.split(','))))