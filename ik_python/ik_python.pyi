from typing import List, Tuple, Annotated, Literal
import numpy as np
from numpy.typing import NDArray

class KinematicsObject:
    """
    Representation of the kinematics for the robot

    :param hMatrix: The h matrix to use (array of vectors of rotation axes)
    :param pMatrix: The p matrix to use (array of vectors of displacements between joints)
    """

    def __init__(
        self,
        hMatrix: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        pMatrix: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> None: ...

class Robot:
    """
    Representation of the robot for inverse kinematics

    :param robotType: The type of robot to create, either a specific hardcoded bot or a general type of bot
    Options for robotType: Irb6640, KukaR800FixedQ3, Ur5, ThreeParallelBot, TwoParallelBot, RrcFixedQ6, SphericalBot, YumiFixedQ3,
    SphericalTwoParallel, SphericalTwoIntersecting, Spherical, ThreeParallelTwoIntersecting, ThreeParallel, TwoParallel, TwoIntersecting, GenSixDof
    """

    def __init__(
        self,
        robotType: (
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
        ),
    ) -> None: ...
    @classmethod
    def set_kinematics(cls, kinematics: KinematicsObject) -> None:
        """
        Set the kinematics object for the robot

        :param kinematics: The kinematics object to use
        """
        ...

    def get_ik(
        self,
        rotationMatrix: (
            Annotated[NDArray[np.generic], Literal[3, 3]]
            | Annotated[List[List[float]], [3, 3]]
        ),
        positionVector: (
            Annotated[NDArray[np.generic], Literal[3]] | Annotated[List[float], [3]]
        ),
    ) -> Tuple[List[float], bool]:
        """
        Get the inverse kinematics for the robot

        :param rotationMatrix: The rotation matrix to use for the inverse kinematics
        :param positionVector: The position vector to use for the inverse kinematics
        :return: A tuple containing the rotation values of each joint and whether the solution is least squares
        """
        ...

    def forward_kinematics(
        self,
        qVals: Annotated[List[float], [6]] | Annotated[NDArray[np.generic], Literal[6]],
    ) -> Tuple[List[List[float]], List[float]]:
        """
        Get the forward kinematics for the robot, not implemented for hardcoded bots

        :param qVals: The rotation values of each joint
        :return: A tuple containing the rotation matrix and position vector
        """
        ...
    # Factory methods for each robot type
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
