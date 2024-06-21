# IK-Geo

This implementation of the core IK-Geo algorithms was adapted from [IK-Geo](https://github.com/rpiRobotics/ik-geo). For details on the algorithms used, refer to the [original source](https://github.com/rpiRobotics/ik-geo/tree/6409ee92e93561c4f805390f1dac894af85a1625/rust) or to the paper ["IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition"](https://arxiv.org/abs/2211.05737).

## To install

This can be installed from pypi with the command `pip install ik_geo`.
To install the package from this repository locally, use:

```bash
cd ik_python
pip install .
```

## To Use

Refer to `examples/python/sample.py` for a full code example.
To compute the IK solutions for a specific robot, you can either select one of the hardcoded robots available or provide your own kinematics. If you provide your own kinematics, you must do so as a [Product of Exponentials (POE)](https://en.wikipedia.org/wiki/Product_of_exponentials_formula). This is either with 6 or 7 `h` vectors (rotation axes) and 7 or 8 p vectors (displacements), respectively.

Once you have your kinematics, you need to choose the correct decomposition strategy from: { "SphericalTwoParallel", "SphericalTwoIntersecting", "Spherical", "ThreeParallelTwoIntersecting", "ThreeParallel", "TwoParallel", "TwoIntersecting" } to use. If you choose the wrong one, you will get wrong answers.

Once you have configured your IK solver, you can get a list of IK solutions by calling the desired ik function:

```python

kinematics = ik_geo.KinematicsObject(H, P)

robot = ik_geo.Robot("SphericalTwoIntersecting")
robot.set_kinematics(kinematics)
R = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
t = [-1, 3, 0]

# For all possible IK solutions sorted by error
solutions = robot.get_ik_sorted(R, t)

# For a single one with 0 error, or for the best least squares solution
solutions = robot.get_ik(R, t)
```
