#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np


from ik_geo import Robot
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as R


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from scripts.help_maths import get_transformation as get_tf

a_list = [0, -0.425, -0.39225, 0.0, 0, 0.0]
d_list = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0]
alpha_list = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]
offset_theta_list = [0, 0, 0, 0, 0, 0]
rots_list = [1, 1, 1, 1, 1, 1]

# Sample assumes you have numpy installed globally
# from trac_ik_python.trac_ik import IK

ROBOT = None


def run_ik_hardcoded(pos: NDArray[np.generic], quat: NDArray[np.generic]):
    """Run the hardcoded inverse kinematics.

    Args:
        pos (NDArray[np.generic]): The position.
        quat (NDArray[np.generic]): The quaternion.
    """
    print("\nRunning hardcoded inverse kinematics:\n-----------------------------")
    # Create the robot, type ur5
    # This can be done with string literals as well as factory methods (shown below)
    global ROBOT
    ROBOT = Robot.ur5()
    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major)
    # The second argument is the position vector (3x1)
    rotation_mat = quaternion_to_rotation_matrix(quat)
    translation_mat = pos_quat_to_transformation_matrix(pos)

    solutions = ROBOT.get_ik_sorted(rotation_mat, translation_mat)
    for q, error, ls in solutions:
        print(
            "Solution: (" + ((f"Least Squares, error={error}") if ls else "Exact") + ")"
        )
        for qVal in q:
            print(qVal)

        compare_results(ROBOT, q, pos, quat)
        print("-----------------------------")
    print("-----------------------------")


def run_ik_general(pos: NDArray[np.generic], quat: NDArray[np.generic]):
    """Run the general inverse kinematics.

    Args:
        pos (NDArray[np.generic]): The position.
        quat (NDArray[np.generic]): The quaternion
    """
    print("\nRunning general inverse kinematics:\n-----------------------------")

    # Create the kinematics
    # Paremeters are deconstructed h matrix and p matrix
    # I'm using the code from the Irb6640 to for real kinematics
    hMat = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
        ]
    )
    pMat = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0892],
            [-0.425, 0.0, 0.0],
            [-0.3922, 0.0, 0.0],
            [0.0, -0.1091, 0.0],
            [0.0, 0.0, -0.0946],
            [0.0, -0.0823, 0.0],
        ]
    )

    # MUST SET THE KINEMATICS OBJECT BEFORE RUNNING IK IN GENERAL CASE
    global ROBOT
    ROBOT = Robot.three_parallel_two_intersecting(hMat, pMat)

    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major deconstructed)
    # The second argument is the position vector (3x1)
    rotation_mat = quaternion_to_rotation_matrix(quat)
    translation_mat = pos.reshape(3, 1)

    print(f"Rotation Matrix:\n{rotation_mat}")
    print(f"Translation Matrix:\n{translation_mat}")

    solutions = ROBOT.get_ik_sorted(rotation_mat, translation_mat)
    for q, error, ls in solutions:
        print(
            "Solution: (" + ((f"Least Squares, error={error}") if ls else "Exact") + ")"
        )
        for qVal in q:
            print(qVal)

        compare_results(ROBOT, q, pos, quat)
        print("-----------------------------")
    print("-----------------------------")


def test_trac_ik(pos: NDArray[np.generic], quat: NDArray[np.generic]):
    """Test the trac_ik library.

    Args:
        pos (NDArray[np.generic]): The position.
        quat (NDArray[np.generic]): The quaternion
    """
    print("\nRunning trac_ik inverse kinematics:\n-----------------------------")
    ik_solver = IK("base_link", "virtual_link", solve_type="Speed", timeout=0.01)

    # fmt: off
    result = ik_solver.get_ik(
        [0, 0, 0, 0, 0, 0],
        pos[0], pos[1], pos[2],
        quat[0], quat[1], quat[2], quat[3],
        bx=1e-6, by=1e-6, bz=1e-6,
        brx=1e-6, bry=1e-6, brz=1e-6
    )  # X, Y, Z, QX, QY, QZ, QW
    # fmt : on

    print(result)


def quaternion_to_rotation_matrix(quat: NDArray[np.generic]) -> NDArray[np.generic]:
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


def quaternion_conjugate(q: NDArray[np.generic]) -> NDArray[np.generic]:
    """Compute the conjugate of a quaternion in the format [x, y, z, w].

    Args:
        q (NDArray[np.generic]): The quaternion in the format [x, y, z, w].

    Returns:
        NDArray[np.generic]: The conjugate of the quaternion.
    """
    return [-q[0], -q[1], -q[2], q[3]]


def quaternion_multiply(q1: NDArray[np.generic], q2: NDArray[np.generic]):
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


def quaternion_error(q1: NDArray[np.generic], q2: NDArray[np.generic]) -> float:
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
    q2_inv = quaternion_conjugate(q2)

    # Compute the quaternion product q1 * q2^(-1)
    q_prod = quaternion_multiply(q1, q2_inv)

    # Extract the vector part of the quaternion product
    vector_part = q_prod[:3]

    # Compute the norm of the vector part
    vector_norm = np.linalg.norm(vector_part)

    # Compute the angle theta
    theta = 2 * np.arcsin(vector_norm)

    return theta


def tf_matrix_to_pos_quat(tf_matrix: NDArray[np.generic]) -> tuple:
    """Extract the position and quaternion from a transformation matrix.

    Args:
        tf_matrix (NDArray[np.generic]): A 4x4 transformation matrix.

    Returns:
        tuple: A tuple containing the position and quaternion.
    """
    # Extract the position vector
    pos = tf_matrix[:3, 3]

    # Extract the rotation matrix
    rot_matrix = tf_matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    rot = R.from_matrix(rot_matrix)
    quat = rot.as_quat()  # Returns (x, y, z, w)

    return pos, quat


def rotation_matrix_to_quaternion(R: NDArray[np.generic]) -> NDArray[np.generic]:
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


def compare_results(
    robot: Robot,
    q: NDArray[np.generic],
    pos: NDArray[np.generic],
    quat: NDArray[np.generic],
):
    """Compare the results of the forward kinematics with the given position and quaternion.

    Args:
        robot (Robot): The robot object.
        q (NDArray[np.generic]): The joint angles.
        pos (NDArray[np.generic]): The position.
        quat (NDArray[np.generic]): The quaternion.
    """
    print("\nRunning forward kinematics:")

    forward = robot.forward_kinematics(q)
    print(np.array(forward[0]))

    new_pos = np.array(forward[1])
    new_quat = np.array(rotation_matrix_to_quaternion(forward[0]))

    error_quat = quaternion_error(new_quat, quat)
    error_pos = np.linalg.norm(new_pos - pos)

    print(f"New forward position: {new_pos}")
    print(f"New forward quaternion: {new_quat}")

    print(f"Error quaternion: {error_quat}")
    print(f"Error position: {error_pos}")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    pos = np.array([0.3, 0.209, 0.6])
    quat = np.array([-0.579, 0.579, -0.406, 0.406])

    # run_ik_hardcoded(pos, quat)
    run_ik_general(pos, quat)
    # test_trac_ik(pos, quat)

    q = []
    q.append([-2.8100, -0.1258, -1.6935, 0.5788, 1.6822, 2.8286])
    q.append([-2.8100, -1.7286, 1.6935, -1.2055, 1.6822, 2.8286])
    q.append([-2.8100, -0.1258, -1.6935, -2.5628, -1.6822, -0.3130])
    q.append([-2.8100, -1.7286, 1.6935, 1.9361, -1.6822, -0.3130])
    q.append([0.9790, -1.4130, -1.6935, 1.3363, 1.2838, -0.9501])
    q.append([0.9790, -3.0158, 1.6935, -0.4479, 1.2838, -0.9501])
    q.append([0.9790, -1.4130, -1.6935, -1.8053, -1.2838, 2.1915])
    q.append([0.9790, -3.0158, 1.6935, 2.6937, -1.2838, 2.1915])

    for qi in q:
        quat = rotation_matrix_to_quaternion(ROBOT.forward_kinematics(qi)[0])
        pos = ROBOT.forward_kinematics(qi)[1]

        print(f"Forward Kinematics :")
        print(f"Position: {pos}")
        print(f"Quaternion: {quat}\n")

    trac_ik_theta_vec = np.array(
        [
            1.1195474479938143,
            -1.1844174982118663,
            -1.7769988076059189,
            -0.8513708314989512,
            0.5611631778261401,
            0.5899065214467356,
        ]
    )
    ee = get_tf(trac_ik_theta_vec, d_list, a_list, alpha_list)
