from typing import List, Tuple, Annotated, Literal
import numpy as np
from numpy.typing import NDArray


class Robot:
    """
    Representation of the robot for inverse kinematics.

    Must construct with factory methods, default constructor raises NotImplementedError.
    """
    def __init__(self) -> None:
        """
        Invalid constructor, will raise a NotImplementedError

        Use the factory methods instead.
        """
        raise NotImplementedError("Initialize with factory methods")

    def get_ik(
        self,
        R: (
            Annotated[NDArray[np.generic], Literal[3, 3]]
            | Annotated[List[List[float]], [3, 3]]
        ),
        t: Annotated[NDArray[np.generic], Literal[3]] | Annotated[List[float], [3]],
    ) -> List[Tuple[List[float], bool]]:
        """
        Compute the inverse kinematics solutions for the robot.

        Args:
            R: The rotation matrix to use for the inverse kinematics
            t: The position vector to use for the inverse kinematics
        Returns:
            A list of tuples containing solutions.
            A solution contains the rotation values of each joint and whether the solution is least squares
        """

    def get_ik_sorted(
        self,
        R: (
            Annotated[NDArray[np.generic], Literal[3, 3]]
            | Annotated[List[List[float]], [3, 3]]
        ),
        t: Annotated[NDArray[np.generic], Literal[3]] | Annotated[List[float], [3]],
    ) -> List[Tuple[List[float], float, bool]]:
        """
        Compute the inverse kinematics solutions for the robot, as well as the error of each solution, sorted by error.

        Args:
            R: The rotation matrix to use for the inverse kinematics
            t: The position vector to use for the inverse kinematics
        Returns:
            A list of tuples containing solutions.
            A solution contains the rotation values of each joint, the error of the solution, and whether the solution is least squares
                They are sorted by error from least to greatest
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
    def irb6640(cls) -> "Robot": ...

    # @classmethod
    # def kuka_r800_fixed_q3(cls) -> "Robot":
    #     ...

    @classmethod
    def ur5(cls) -> "Robot": ...
    @classmethod
    def three_parallel_bot(cls) -> "Robot": ...
    @classmethod
    def two_parallel_bot(cls) -> "Robot": ...
    @classmethod
    def spherical_bot(cls) -> "Robot": ...

    # @classmethod
    # def rrc_fixed_q6(cls) -> "Robot":
    #     return cls("RrcFixedQ6")

    # @classmethod
    # def yumi_fixed_q3(cls) -> "Robot":
    #     return cls("YumiFixedQ3")

    @classmethod
    def spherical_two_intersecting(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def spherical_two_parallel(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def spherical(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def three_parallel_two_intersecting(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def three_parallel(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def two_parallel(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def two_intersecting(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass

    @classmethod
    def gen_six_dof(
        cls,
        h: (
            Annotated[NDArray[np.generic], Literal[6, 3]]
            | Annotated[List[List[float]], [6, 3]]
        ),
        p: (
            Annotated[NDArray[np.generic], Literal[7, 3]]
            | Annotated[List[List[float]], [7, 3]]
        ),
    ) -> "Robot":
        pass
