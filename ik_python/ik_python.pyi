from typing import List, Tuple


class KinematicsObject:
    """
    Representation of the kinematics for the robot

    :param hMatrix: The h matrix to use (array of vectors of rotation axes)
    :param pMatrix: The p matrix to use (array of vectors of displacements between joints)
    """
    def __init__(self, hMatrix: List[List[float]], pMatrix: List[List[float]]) -> None:
        ...

class Robot:
    """
    Representation of the robot for inverse kinematics

    :param robotType: The type of robot to create, either a specific hardcoded bot or a general type of bot
    Options for robotType: Irb6640, KukaR800FixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, RrcFixedQ6, SphericalBot, YumiFixedQ3,
    SphericalTwoParallel, SphericalTwoIntersecting, Spherical, ThreeParallelTwoIntersecting, ThreeParallel, TwoParallel, TwoIntersecting, GenSixDof
    """
    def __init__(self, robotType: str) -> None:
        ...

    @classmethod
    def set_kinematics(self, kinematics: KinematicsObject) -> None:
        """
        Set the kinematics object for the robot

        :param kinematics: The kinematics object to use
        """
        ...

    def get_ik(self, rotationMatrix: List[List[float]], positionVector: List[float]) -> Tuple[List[float], bool]:
        """
        Get the inverse kinematics for the robot

        :param rotationMatrix: The rotation matrix to use for the inverse kinematics
        :param positionVector: The position vector to use for the inverse kinematics
        :return: A tuple containing the rotation values of each joint and whether the solution is least squares
        """
        ...
    def forward_kinematics(self, qVals) -> Tuple[List[List[float]], List[float]]:
        """
        Get the forward kinematics for the robot, not implemented for hardcoded bots

        :param qVals: The rotation values of each joint 
        :return: A tuple containing the rotation matrix and position vector
        """
        ...