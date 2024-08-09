#!/usr/bin/python3
# -*- coding: utf-8 -*-

""" Helper functions for mathematical operations. """
import numpy as np
from scipy.spatial.transform import Rotation as R

PI = np.pi


def x_trans_matrix(val: float) -> np.ndarray:
    """ Create a translation matrix along the x-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        np.array: The translation matrix.
    """
    k = np.eye(4, 4)
    k[0, 3] = val

    return k


def y_trans_matrix(val: float) -> np.ndarray:
    """ Create a translation matrix along the y-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        np.array: The translation matrix.
    """
    k = np.eye(4, 4)
    k[1, 3] = val

    return k


def z_trans_matrix(val: float) -> np.ndarray:
    """ Create a translation matrix along the z-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        np.array: The translation matrix.
    """
    k = np.eye(4, 4)
    k[2, 3] = val

    return k


def x_rot_matrix(in_theta: float) -> np.ndarray:
    """ Create a rotation matrix along the x-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        np.array: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[1, 1] = np.cos(in_theta)
    mat_id[2, 1] = np.sin(in_theta)
    mat_id[1, 2] = -np.sin(in_theta)
    mat_id[2, 2] = np.cos(in_theta)

    return mat_id


def y_rot_matrix(in_theta: float) -> np.ndarray:
    """ Create a rotation matrix along the y-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        np.array: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[0, 0] = np.cos(in_theta)
    mat_id[2, 0] = -np.sin(in_theta)
    mat_id[0, 2] = np.sin(in_theta)
    mat_id[2, 2] = np.cos(in_theta)

    return mat_id


def z_rot_matrix(in_theta: float) -> np.ndarray:
    """ Create a rotation matrix along the z-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        np.array: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[0, 0] = np.cos(in_theta)
    mat_id[1, 0] = np.sin(in_theta)
    mat_id[0, 1] = -np.sin(in_theta)
    mat_id[1, 1] = np.cos(in_theta)

    return mat_id


def axis_rot(axis: np.ndarray, angle: float) -> np.ndarray:
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


def get_transformation(
        theta_val: np.array, d_val: np.array, a_val: np.array, alpha_val: np.array,
        print_pose=False
) -> np.ndarray:
    """ Get the transformation matrix given the DH parameters.

    Args:
        theta_val (np.array): The theta values.
        d_val (np.array): The d values.
        a_val (np.array): The a values.
        alpha_val (np.array): The alpha values.
        print_coordinates (bool): Whether to print the position and quaternion.

    Returns:
        np.array: The transformation matrix.
    """
    if not (np.size(theta_val) == np.size(d_val) == np.size(a_val) == np.size(alpha_val)):
        raise ValueError("All inputs must have the same length")

    coord_num = np.size(theta_val)
    m = np.identity(coord_num)

    for i in range(coord_num):
        m = (
            m @ z_rot_matrix(theta_val[i]) @ z_trans_matrix(d_val[i]) @
            x_rot_matrix(alpha_val[i]) @ x_trans_matrix(a_val[i])
        )

    if print_pose:
        pos, quat = tf_matrix_to_pos_quat(m)
        print(f"Position: {pos}")
        print(f"Quaternion: {quat}")

    return m


def tf_matrix_to_pos_quat(tf_matrix: np.ndarray) -> tuple:
    """ Convert a transformation matrix to position and quaternion.

    Args:
        tf_matrix (np.array): The transformation matrix.

    Returns:
        tuple: The position and quaternion.
    """
    pos = tf_matrix[:3, 3]
    rot_matrix = tf_matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    rot = R.from_matrix(rot_matrix)
    quat = rot.as_quat()  # Returns (x, y, z, w)

    return pos, quat
