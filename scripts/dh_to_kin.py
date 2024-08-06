#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert a robot defined in Denavit-Hartenberg convention to Product
of Exponentials.
"""

import numpy as np


def rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """ Create a rotation matrix given an axis and an angle.

    Args:
        axis (np.array): The axis of rotation.
        angle (float): The angle of rotation.

    Returns:
        np.array: The rotation matrix
    """
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    one_minus_cos = 1 - cos_angle

    axis = axis / np.linalg.norm(axis)
    x, y, z = axis

    return np.array([
        [
            cos_angle + x*x*one_minus_cos,
            x*y*one_minus_cos - z*sin_angle,
            x*z*one_minus_cos + y*sin_angle
        ], [
            x*y*one_minus_cos + z*sin_angle,
            cos_angle + y*y * one_minus_cos,
            y*z*one_minus_cos - x*sin_angle
        ], [
            x*z*one_minus_cos - y*sin_angle,
            y*z*one_minus_cos + x*sin_angle,
            cos_angle + z*z*one_minus_cos
        ]
    ])


def dh_to_kin(alpha_vec: np.ndarray, a_vec: np.ndarray, d_vec: np.ndarray) -> dict:
    """ Convert a robot defined in Denavit-Hartenberg convention to
    Product of Exponentials.

    Args:
        alpha_vec (np.array): The alpha vector.
        a_vec (np.array): The a vector.
        d_vec (np.array): The d vector.

    Returns:
        dict: The kinematic structure of the robot.
    """
    N = len(alpha_vec)
    kin = {
        'joint_type': np.zeros(N),
        'H': np.full((3, N), np.nan),
        'P': np.full((3, N+1), np.nan)
    }

    kin['P'][:, 0] = [0, 0, 0]
    kin['H'][:, 0] = [0, 0, 1]
    R = np.eye(3)

    for i in range(N):
        # Translate d_i along z_{i-1}
        # Move a along x_{i-1}
        kin['P'][:, i+1] = d_vec[i] * R[:, 2] + a_vec[i] * R[:, 0]

        # Rotate by alpha along x_{i-1}
        R = rot(R[:, 0], alpha_vec[i]) @ R

        if i == N - 1:
            kin['RT'] = rot(R[:, 0], alpha_vec[i])
        else:
            kin['H'][:, i+1] = R[:, 2]  # Joint axis is z axis

    return kin


if __name__ == '__main__':
    alpha_vec = np.array([0, -np.pi / 2, 0, 0, -np.pi / 2, np.pi / 2])
    a_vec = np.array([0, -0.13585, 0.425, 0.39225, 0, 0])
    d_vec = np.array([0.089159, 0, -0.1197, 0, 0.093, 0.0823])

    kin = dh_to_kin(alpha_vec, a_vec, d_vec)

    # Set print options for better readability
    np.set_printoptions(precision=4, suppress=True)

    print("Joint Types:", kin['joint_type'])
    print("H Matrix:\n", np.array2string(kin['H'].T, separator=','))
    print("P Matrix:\n", np.array2string(kin['P'].T, separator=','))
    if 'RT' in kin:
        print("RT Matrix:\n", kin['RT'])
