import ik_geo

# Sample assumes you have numpy installed globally
import numpy as np


def run_ik_hardcoded():
    print("\nRunning hardcoded inverse kinematics:\n-----------------------------")
    # Create the robot, type ur5
    # This can be done with string literals as well as factory methods (shown below)
    robot = ik_geo.Robot("Ur5")
    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major)
    # The second argument is the position vector (3x1)
    solutions = robot.get_ik(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0]
    )
    for (q, ls) in solutions:
        print("Soution: (" + ("Least Squares" if ls else "Exact")+ ")")
        for qVal in q:
            print(qVal)
        print("-----------------------------")
    print("-----------------------------")


def run_ik_general():
    print("\nRunning general inverse kinematics:\n-----------------------------")
    # Create the robot, type sphericalTwoParallel
    robot = ik_geo.Robot("SphericalTwoIntersecting")

    # Create the kinematics
    # Paremeters are deconstructed h matrix and p matrix
    # I'm using the code from the Irb6640 to for real kinematics
    zv = [0.0, 0.0, 0.0]
    ex = [1.0, 0.0, 0.0]
    ey = [0.0, 1.0, 0.0]
    ez = [0.0, 0.0, 1.0]
    hMat = np.array([ez, ey, ey, ex, ey, ex])
    pMat = np.array(
        [zv, [0.32, 0, 0.78], [0, 0, 1.075], [1.1425, 0, 0.2], ez, ez, [0.2, 0, 0]]
    )
    kinematics = ik_geo.KinematicsObject(hMat, pMat)

    # MUST SET THE KINEMATICS OBJECT BEFORE RUNNING IK IN GENERAL CASE
    robot.set_kinematics(kinematics)

    print("Kinematics object set")

    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major deconstructed)
    # The second argument is the position vector (3x1)
    solutions = robot.get_ik_sorted([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], [-1, 3, 0])
    
    for (q, error, ls) in solutions:
        print("Soution: (" + ((f"Least Squares, error={error}") if ls else "Exact")+ ")")
        for qVal in q:
            print(qVal)
        print("-----------------------------")
    print("-----------------------------")



# Running with factory methods
def run_ik_with_factory():
    print(
        "\nRunning inverse kinematics with factory methods:\n-----------------------------"
    )
    # You can also use the factory methods to create the robot
    robot = ik_geo.Robot.irb6640()
    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major)
    # The second argument is the position vector (3x1)
    rotMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    posVector = np.array([0, 0, 0])

    solutions = robot.get_ik(rotMatrix, posVector)
    for (q, ls) in solutions:
        print("Soution: (" + ("Least Squares" if ls else "Exact")+ ")")
        for qVal in q:
            print(qVal)
        print("-----------------------------")
    print("-----------------------------")


run_ik_hardcoded()
run_ik_general()
run_ik_with_factory()
