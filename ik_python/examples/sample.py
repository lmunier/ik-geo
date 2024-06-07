import ik_python
# Sample assumes you have numpy installed globally
import numpy as np

def run_ik_hardcoded():
    print("\nRunning hardcoded inverse kinematics:\n-----------------------------")
    # Create the robot, type ur5
    robot = ik_python.Robot("ur5")
    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major)
    # The second argument is the position vector (3x1)
    (qVals, isLs) = robot.get_ik([[1,0,0],[0,1,0],[0,0,1]], [0, 0, 0]);
    print("qVals: ", qVals)
    print("ils: ", isLs)
    print("-----------------------------")
 
def run_ik_general():
    print("\nRunning general inverse kinematics:\n-----------------------------")
    # Create the robot, type sphericalTwoParallel
    robot = ik_python.Robot("SphericalTwoIntersecting")

    # Create the kinematics
    # Paremeters are deconstructed h matrix and p matrix
    # I'm using the code from the Irb6640 to for real kinematics
    zv = [0.0,0.0,0.0]
    ex = [1.0,0.0,0.0]
    ey = [0.0,1.0,0.0]
    ez = [0.0,0.0,1.0]
    hMat = np.array([ez,ey,ey,ex,ey,ex])
    pMat = np.array([zv, [.32,0,.78], [0,0,1.075], [1.1425,0, 0.2], ez, ez, [0.2,0,0]])
    kinematics = ik_python.KinematicsObject(hMat, pMat)

    # MUST SET THE KINEMATICS OBJECT BEFORE RUNNING IK IN GENERAL CASE
    robot.set_kinematics(kinematics)

    print("Kinematics object set")


    # Get the inverse kinematics
    # The first argument is the rotation matrix (3x3, row major deconstructed)
    # The second argument is the position vector (3x1)
    (qVals, isLs) = robot.get_ik([[1,0,0],[0,1,0],[0,0,1]], [0, 0, 0]);
    print("qVals: ", qVals)
    print("ils: ", isLs)

    
run_ik_hardcoded()
run_ik_general()
