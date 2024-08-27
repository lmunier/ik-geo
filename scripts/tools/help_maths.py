#!/usr/bin/python3
# -*- coding: utf-8 -*-

""" Helper functions for mathematical operations. """
import numpy as np

from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as R


def x_trans_matrix(val: float) -> NDArray[np.generic]:
    """Create a translation matrix along the x-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        NDArray[np.generic]: The translation matrix.
    """
    k = np.eye(4, 4)
    k[0, 3] = val

    return k


def y_trans_matrix(val: float) -> NDArray[np.generic]:
    """Create a translation matrix along the y-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        NDArray[np.generic]: The translation matrix.
    """
    k = np.eye(4, 4)
    k[1, 3] = val

    return k


def z_trans_matrix(val: float) -> NDArray[np.generic]:
    """Create a translation matrix along the z-axis.

    Args:
        val (float): The value to translate by.

    Returns:
        NDArray[np.generic]: The translation matrix.
    """
    k = np.eye(4, 4)
    k[2, 3] = val

    return k


def x_rot_matrix(in_theta: float) -> NDArray[np.generic]:
    """Create a rotation matrix along the x-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        NDArray[np.generic]: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[1, 1] = np.cos(in_theta)
    mat_id[2, 1] = np.sin(in_theta)
    mat_id[1, 2] = -np.sin(in_theta)
    mat_id[2, 2] = np.cos(in_theta)

    return mat_id


def y_rot_matrix(in_theta: float) -> NDArray[np.generic]:
    """Create a rotation matrix along the y-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        NDArray[np.generic]: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[0, 0] = np.cos(in_theta)
    mat_id[2, 0] = -np.sin(in_theta)
    mat_id[0, 2] = np.sin(in_theta)
    mat_id[2, 2] = np.cos(in_theta)

    return mat_id


def z_rot_matrix(in_theta: float) -> NDArray[np.generic]:
    """Create a rotation matrix along the z-axis.

    Args:
        in_theta (float): The angle of rotation.

    Returns:
        NDArray[np.generic]: The rotation matrix.
    """
    mat_id = np.eye(4, 4)
    mat_id[0, 0] = np.cos(in_theta)
    mat_id[1, 0] = np.sin(in_theta)
    mat_id[0, 1] = -np.sin(in_theta)
    mat_id[1, 1] = np.cos(in_theta)

    return mat_id


def axis_rot(axis: NDArray[np.generic], angle: float) -> NDArray[np.generic]:
    """Create a rotation matrix given an axis and an angle.

    Args:
        axis (NDArray[np.generic]): The axis of rotation.
        angle (float): The angle of rotation.

    Returns:
        NDArray[np.generic]: The rotation matrix
    """
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    one_minus_cos = 1 - cos_angle

    axis = axis / np.linalg.norm(axis)
    x, y, z = axis

    return np.array(
        [
            [
                cos_angle + x * x * one_minus_cos,
                x * y * one_minus_cos - z * sin_angle,
                x * z * one_minus_cos + y * sin_angle,
            ],
            [
                x * y * one_minus_cos + z * sin_angle,
                cos_angle + y * y * one_minus_cos,
                y * z * one_minus_cos - x * sin_angle,
            ],
            [
                x * z * one_minus_cos - y * sin_angle,
                y * z * one_minus_cos + x * sin_angle,
                cos_angle + z * z * one_minus_cos,
            ],
        ]
    )


def get_transformation(
    theta_val: np.array,
    d_val: np.array,
    a_val: np.array,
    alpha_val: np.array,
    print_pose=False,
) -> NDArray[np.generic]:
    """Get the transformation matrix given the DH parameters.

    Args:
        theta_val (NDArray[np.generic]): The theta values.
        d_val (NDArray[np.generic]): The d values.
        a_val (NDArray[np.generic]): The a values.
        alpha_val (NDArray[np.generic]): The alpha values.
        print_coordinates (bool): Whether to print the position and quaternion.

    Returns:
        NDArray[np.generic]: The transformation matrix.
    """
    trans_size = 4  # Transformation matrix size is 4x4

    if not (
        np.size(theta_val) == np.size(d_val) == np.size(a_val) == np.size(alpha_val)
    ):
        raise ValueError("All inputs must have the same length")

    m = np.identity(trans_size)
    n_dofs = np.size(theta_val)

    for i in range(n_dofs):
        m = (
            m
            @ z_rot_matrix(theta_val[i])
            @ z_trans_matrix(d_val[i])
            @ x_rot_matrix(alpha_val[i])
            @ x_trans_matrix(a_val[i])
        )

    if print_pose:
        pos, quat = tf_matrix_to_pos_quat(m)
        print(f"Position: {pos}")
        print(f"Quaternion: {quat}")

    return m


def quat_to_rot_matrix(quat: NDArray[np.generic]) -> NDArray[np.generic]:
    """Convert a quaternion into a rotation matrix.

    Args:
        quat (NDArray[np.generic]): An array of four elements representing the quaternion [qx, qy, qz, qw]

    Returns:
        NDArray[np.generic]: A 3x3 rotation matrix
    """
    qx, qy, qz, qw = quat

    # Compute the rotation matrix elements
    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)

    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy * qz - qx * qw)

    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])


def quat_conjugate(q: NDArray[np.generic]) -> NDArray[np.generic]:
    """Compute the conjugate of a quaternion in the format [x, y, z, w].

    Args:
        q (NDArray[np.generic]): The quaternion in the format [x, y, z, w].

    Returns:
        NDArray[np.generic]: The conjugate of the quaternion.
    """
    return [-q[0], -q[1], -q[2], q[3]]


def quat_multiply(q1: NDArray[np.generic], q2: NDArray[np.generic]):
    """Compute the product of two quaternions in the format [x, y, z, w].

    Args:
        q1 (NDArray[np.generic]): The first quaternion.
        q2 (NDArray[np.generic]): The second quaternion.

    Returns:
        NDArray[np.generic]: The product of the two quaternions.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_error(q1: NDArray[np.generic], q2: NDArray[np.generic]) -> float:
    """Compute the error angle between two quaternions in the format [x, y, z, w].

    Args:
        q1 (NDArray[np.generic]): The first quaternion.
        q2 (NDArray[np.generic]): The second quaternion.

    Returns:
        float: The error angle between the two quaternions.
    """
    # Normalize the quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute the inverse of q2 (which is its conjugate for unit quaternions)
    q2_inv = quat_conjugate(q2)
    q_prod = quat_multiply(q1, q2_inv)

    vector_part = q_prod[:3]
    vector_norm = np.linalg.norm(vector_part)

    return 2 * np.arcsin(vector_norm)


def tf_matrix_to_pos_quat(tf_matrix: NDArray[np.generic]) -> tuple:
    """Extract the position and quaternion from a transformation matrix.

    Args:
        tf_matrix (NDArray[np.generic]): A 4x4 transformation matrix.

    Returns:
        tuple: A tuple containing the position and quaternion.
    """
    pos = tf_matrix[:3, 3]
    rot_matrix = tf_matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    rot = R.from_matrix(rot_matrix)
    quat = rot.as_quat()  # Returns (x, y, z, w)

    return pos, quat


def rot_matrix_to_quat(R: NDArray[np.generic]) -> NDArray[np.generic]:
    """Convert a rotation matrix into a quaternion.

    Args:
        R (NDArray[np.generic]): A 3x3 rotation matrix

    Returns:
        NDArray[np.generic]: An array of four elements representing the quaternion [qx, qy, qz, qw]
    """
    m00, m01, m02 = R[0][0], R[0][1], R[0][2]
    m10, m11, m12 = R[1][0], R[1][1], R[1][2]
    m20, m21, m22 = R[2][0], R[2][1], R[2][2]

    trace = m00 + m11 + m22

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m21 - m12) * s
        qy = (m02 - m20) * s
        qz = (m10 - m01) * s
    elif (m00 > m11) and (m00 > m22):
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw])
