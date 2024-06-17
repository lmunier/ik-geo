from typing import List, Tuple, Annotated, Literal
import numpy as np
from numpy.typing import NDArray

class KinematicsObject:
    """
    Representation of the kinematics for the robot

    Args:
        h_matrix: The h matrix to use (array of vectors of rotation axes)
        p_matrix: The p matrix to use (array of vectors of displacements between joints)
    """

    def __init__(
        self,
        h_matrix: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p_matrix: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> None: ...

RobotType = (
    Literal["Irb6640"]
    | Literal["KukaR800FixedQ3"]
    | Literal["Ur5"]
    | Literal["ThreeParallelBot"]
    | Literal["TwoParallelBot"]
    | Literal["RrcFixedQ6"]
    | Literal["SphericalBot"]
    | Literal["YumiFixedQ3"]
    | Literal["SphericalTwoParallel"]
    | Literal["SphericalTwoIntersecting"]
    | Literal["Spherical"]
    | Literal["ThreeParallelTwoIntersecting"]
    | Literal["ThreeParallel"]
    | Literal["TwoParallel"]
    | Literal["TwoIntersecting"]
    | Literal["GenSixDof"]
)

class Robot:
    """
    Representation of the robot for inverse kinematics

    Args:
        robot_type: The type of robot to create, either a specific hardcoded bot or a general type of bot
    """

    def __init__(self, robot_type: RobotType) -> None: ...
    @classmethod
    def set_kinematics(cls, kinematics: KinematicsObject) -> None:
        """
        Set the kinematics object for the robot


        Args:
            kinematics: The kinematic description of the robot
        """

    def get_ik(
        self,
        R: (
            Annotated[NDArray[np.generic], Literal[3, 3]]
            | Annotated[List[List[float]], [3, 3]]
        ),
        t: Annotated[NDArray[np.generic], Literal[3]] | Annotated[List[float], [3]],
    ) -> Tuple[List[float], bool]:
        """
        Compute the inverse kinematics solutions for the robot.

        Args:
            R: The rotation matrix to use for the inverse kinematics
            t: The position vector to use for the inverse kinematics
        Returns:
            A tuple containing the rotation values of each joint and whether the solution is least squares
        """

    def forward_kinematics(
        self,
        q: Annotated[List[float], [6]] | Annotated[NDArray[np.generic], Literal[6]],
    ) -> Tuple[List[List[float]], List[float]]:
        """
        Get the forward kinematics for the robot, not implemented for hardcoded bots

        Args:
            q: The rotation values of each joint
        Returns:
            A tuple containing the rotation matrix and position vector
        """

    @classmethod
    def irb6640(cls) -> "Robot":
        return cls("Irb6640")

    @classmethod
    def kuka_r800_fixed_q3(cls) -> "Robot":
        return cls("KukaR800FixedQ3")

    @classmethod
    def ur5(cls) -> "Robot":
        return cls("Ur5")

    @classmethod
    def three_parallel_bot(cls) -> "Robot":
        return cls("ThreeParallelBot")

    @classmethod
    def two_parallel_bot(cls) -> "Robot":
        return cls("TwoParallelBot")

    @classmethod
    def rrc_fixed_q6(cls) -> "Robot":
        return cls("RrcFixedQ6")

    @classmethod
    def spherical_two_intersecting(cls) -> "Robot":
        return cls("SphericalTwoIntersecting")

    @classmethod
    def spherical_two_parallel(cls) -> "Robot":
        return cls("SphericalTwoParallel")

    @classmethod
    def spherical(cls) -> "Robot":
        return cls("Spherical")

    @classmethod
    def three_parallel_two_intersecting(cls) -> "Robot":
        return cls("ThreeParallelTwoIntersecting")

    @classmethod
    def three_parallel(cls) -> "Robot":
        return cls("ThreeParallel")

    @classmethod
    def two_parallel(cls) -> "Robot":
        return cls("TwoParallel")

    @classmethod
    def two_intersecting(cls) -> "Robot":
        return cls("TwoIntersecting")

    @classmethod
    def gen_six_dof(cls) -> "Robot":
        return cls("GenSixDof")

    @classmethod
    def spherical_bot(cls) -> "Robot":
        return cls("SphericalBot")

    @classmethod
    def yumi_fixed_q3(cls) -> "Robot":
        return cls("YumiFixedQ3")
