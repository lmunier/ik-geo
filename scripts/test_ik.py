#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np

from ik_geo import Robot
from tools import help_maths as hm
from numpy.typing import NDArray

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
    rotation_mat = hm.quat_to_rot_matrix(quat)
    translation_mat = pos.reshape(3, 1)

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
    rotation_mat = hm.quat_to_rot_matrix(quat)
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

    new_pos = np.array(forward[1])
    new_quat = np.array(hm.rot_matrix_to_quat(forward[0]))

    error_quat = hm.quat_error(new_quat, quat)
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
        quat = hm.rot_matrix_to_quat(ROBOT.forward_kinematics(qi)[0])
        pos = ROBOT.forward_kinematics(qi)[1]

        print(f"Forward Kinematics :")
        print(f"Position: {pos}")
        print(f"Quaternion: {quat}\n")
