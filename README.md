# IK-Geo

This implementation of the core IK-Geo algorithms was adapted from [IK-Geo](https://github.com/rpiRobotics/ik-geo). For details on the algorithms used, refer to the [original source](https://github.com/rpiRobotics/ik-geo/tree/6409ee92e93561c4f805390f1dac894af85a1625/rust) or to the paper ["IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition"](https://arxiv.org/abs/2211.05737).

## To install

This can be installed from pypi with:

```bash
pip install ik_geo
```

To install the package from this repository locally, use:

```bash
pip install .
```

## To Use

Refer to `examples/sample.py` for a full code example.
To compute the IK solutions for a specific robot, you can either select one of the hardcoded robots available or provide your own kinematics. If you provide your own kinematics, you must do so as a [Product of Exponentials (POE)](https://en.wikipedia.org/wiki/Product_of_exponentials_formula). This is either with 6 or 7 `h` vectors (rotation axes) and 7 or 8 `p` vectors (displacements), respectively. Note that 7-joint bots are not yet supported, so the user must choose one joint to fix to give 6 `h` vectors and 7 `p` vectors

Once you have your kinematics, you need to choose the correct decomposition strategy from: { "spherical_two_parallel", "spherical_two_intersecting", "spherical", "three_parallel_two_intersecting", "three_parallel", "two_parallel", "two_intersecting", "gen_six_dof" } to use. If you choose the wrong one, you will get wrong answers.

Once you have configured your IK solver, you can get a list of IK solutions by calling the desired ik function:

```python
from ik_geo import Robot

h # 6x3 array
p # 7x3 array

robot = Robot.spherical_two_intersecting(h,p)

R = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
t = [-1, 3, 0]

# For all possible solutions, as well as whether or not they are least squares
solutions = robot.get_ik(R, t)

# For all possible IK solutions sorted by error
solutions = robot.get_ik_sorted(R, t)
```

## Performance

While this implementation can be used on a wide range of manipulators, it performs much better on when the solution can be found entirely analytically. The following table shows which method is used for each type of kinematics:

| Solution Type | Robot Kinematic Family                             | Example                            |
| ------------- | -------------------------------------------------- | ---------------------------------- |
| Closed-form   | Spherical joint                                    | Franka Production 3, fixed $q_5$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | KUKA LBR iiwa 7 R800 , fixed $q_3$ |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | ABB IRB 6640                       |
|               | Three parallel axes                                | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | Universal Robots UR5               |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 1D search     | Two intersecting axes                              | Kassow Robots KR810, fixed $q_7$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | FANUC CRX-10iA/L                   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | Kawasaki KJ125                     |
|               | Two parallel axes                                  | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
|               | Two intersecting axes $k, k+2$                     | ABB YuMi, fixed $q_3$              |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | RRC K-1207i, fixed $q_6$           |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 2D search     | General 6R                                         | Kassow Robots KR810, fixed $q_6$   |
