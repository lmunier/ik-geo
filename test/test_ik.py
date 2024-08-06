from ik_geo import Robot

# Sample assumes you have numpy installed globally
import numpy as np
# from trac_ik_python.trac_ik import IK


def run_ik_hardcoded(pos: list, quat: list):
    """ Run the hardcoded inverse kinematics.

    Args:
        pos (list): The position.
        quat (list): The quaternion.
    """
    print("\nRunning hardcoded inverse kinematics:\n-----------------------------")
    # Create the robot, type ur5
    # This can be done with string literals as well as factory methods (shown below)
    robot = Robot.ur5()
    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major)
    # The second argument is the position vector (3x1)
    rotation_mat = quaternion_to_rotation_matrix(quat)
    translation_mat = pos_quat_to_transformation_matrix(pos)

    solutions = robot.get_ik_sorted(
        rotation_mat, translation_mat
    )
    for (q, error, ls) in solutions:
        print(
            "Solution: (" +
            ((f"Least Squares, error={error}") if ls else "Exact") + ")"
        )
        for qVal in q:
            print(qVal)

        compare_results(robot, q, pos, quat)
        print("-----------------------------")
    print("-----------------------------")


def run_ik_general(pos: list, quat: list):
    """ Run the general inverse kinematics.

    Args:
        pos (list): The position.
        quat (list): The quaternion
    """
    print("\nRunning general inverse kinematics:\n-----------------------------")

    # Create the kinematics
    # Paremeters are deconstructed h matrix and p matrix
    # I'm using the code from the Irb6640 to for real kinematics
    mey = [0.0, -1.0, 0.0]
    ez = [0.0, 0.0, 1.0]
    mez = [0.0, 0.0, -1.0]
    hMat = np.array(
        [
            [0., 0., 1.],
            [0., 0., 1.],
            [0., 1., 0.],
            [0., 1., 0.],
            [0., 1., 0.],
            [0., 0., -1.]
        ]
    )
    pMat = np.array(
        [
            [0., 0., 0.],
            [0., 0., 0.0892],
            [-0.1358, 0., 0.],
            [0.425, -0.1197, -0.],
            [0.3922, 0., 0.],
            [0., 0.093, 0.],
            [0., 0., -0.0823]
        ]
    )

    # MUST SET THE KINEMATICS OBJECT BEFORE RUNNING IK IN GENERAL CASE
    robot = Robot.three_parallel_two_intersecting(hMat, pMat)

    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major deconstructed)
    # The second argument is the position vector (3x1)
    rotation_mat = quaternion_to_rotation_matrix(quat)
    translation_mat = pos_quat_to_transformation_matrix(pos)

    solutions = robot.get_ik(
        rotation_mat, translation_mat
    )

    for (q, ls) in solutions:
        # print(
        #     "Solution: (" +
        #     ((f"Least Squares, error={error}") if ls else "Exact") + ")"
        # )
        for qVal in q:
            print(qVal)

        compare_results(robot, q, pos, quat)
        print("-----------------------------")
    print("-----------------------------")


def test_trac_ik(pos: list, quat: list):
    """ Test the trac_ik library.

    Args:
        pos (list): The position.
        quat (list): The quaternion
    """
    print("\nRunning trac_ik inverse kinematics:\n-----------------------------")
    ik_solver = IK(
        'base_link',
        'virtual_link',
        solve_type="Speed",
        timeout=0.01
    )

    result = ik_solver.get_ik(
        [0, 0, 0, 0, 0, 0],
        pos[0], pos[1], pos[2],
        quat[0], quat[1], quat[2], quat[3],
        bx=1e-6, by=1e-6, bz=1e-6,
        brx=1e-6, bry=1e-6, brz=1e-6
    )  # X, Y, Z, QX, QY, QZ, QW

    print(result)


def quaternion_to_rotation_matrix(quat: list) -> np.ndarray:
    """
    Convert a quaternion into a rotation matrix.

    Args:
        quat (list): A list of four elements representing the quaternion [qx, qy, qz, qw]

    Returns:
        np.ndarray: A 3x3 rotation matrix
    """
    qx, qy, qz, qw = quat

    # Compute the rotation matrix elements
    r00 = 1 - 2 * (qy ** 2 + qz ** 2)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)

    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx ** 2 + qz ** 2)
    r12 = 2 * (qy * qz - qx * qw)

    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx ** 2 + qy ** 2)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])


def rotation_matrix_to_quaternion(R: np.ndarray) -> list:
    """
    Convert a rotation matrix into a quaternion.

    Args:
        R (np.ndarray): A 3x3 rotation matrix

    Returns:
        list: A list of four elements representing the quaternion [qx, qy, qz, qw]
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

    return [qx, qy, qz, qw]


def pos_quat_to_transformation_matrix(pos: list):
    """ Convert position and quaternion to a translation_vector matrix. """
    return np.array(pos).reshape(3, 1)


def compare_results(robot: Robot, q: list, pos: list, quat: list):
    """ Compare the results of the forward kinematics with the given position and quaternion.

    Args:
        robot (Robot): The robot object.
        q (list): The joint angles.
        pos (list): The position.
        quat (list): The quaternion.
    """
    print("\nRunning forward kinematics:")
    pos = np.array(pos)
    quat = np.array(quat)

    forward = robot.forward_kinematics(q)

    new_pos = np.array(forward[1])
    new_quat = np.array(rotation_matrix_to_quaternion(forward[0]))

    error_quat = np.linalg.norm(new_quat - quat)
    error_pos = np.linalg.norm(new_pos - pos)

    print(f"New forward position: {forward[1]}")
    print(f"Error quaternion: {error_quat}")
    print(f"Error position: {error_pos}")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    pos = [0.3, 0.209, 0.6]
    quat = [-0.579, 0.579, -0.406, 0.406]

    # run_ik_hardcoded(pos, quat)
    run_ik_general(pos, quat)
    # test_trac_ik(pos, quat)
