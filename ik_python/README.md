# IK-Geo
This is a fork from [IK-Geo](https://github.com/rpiRobotics/ik-geo).
See the full fork [here](https://github.com/Verdant-Evolution/ik-geo).

Inverse kinematics and subproblem solutions from ["IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition"](https://arxiv.org/abs/2211.05737) implemented in MATLAB, C++, Rust, and Python. We also include examples and timing tests.  IK-Geo is the fastest general IK solver based on published literature. In this unifying approach, IK for any 6-DOF all-revolute (6R) manipulator is decomposed into six canonical geometric subproblems solved by intersecting circles with other geometric objects. . IK-Geo finds all IK solutions including singular solutions and sometimes least-squares solutions by solving for subproblem solutions in all cases, including in a continuous and sometimes least-squares sense when a solution does not exist.

We also connect our geometric method with polynomial-based method: 1D and 2D search solutions may be converted to a polynomial in the tangent half-angle of one joint. Examples are found in the [subproblem-polynomial](https://github.com/rpiRobotics/subproblem-polynomial) repo.

For 7-DOF inverse kinematics using the Shoulder-Elbow-Wrist (SEW) angle, please see the [stereo-sew](https://github.com/rpiRobotics/stereo-sew) repo.

For diagrams, see the [matlab-diagrams](https://github.com/aelias36/matlab-diagrams) repo.

## Python wrapper

This is a python wrapper for the Rust implementation of inverse kinematics using PyO3 maturin.  It is implemented [here](https://github.com/Verdant-Evolution/ik-geo/tree/main/ik_python)

## To install

This can be installed with the command `pip install ik_geo`. 

## To use

The API documentation is at [`ik_geo.pyi`](https://github.com/Verdant-Evolution/ik-geo/blob/main/ik_python/ik_geo.pyi), and there will be typechecking with mypy installed.
For sample usage, check [here](https://github.com/Verdant-Evolution/ik-geo/tree/main/examples/python)

## To build/develop

This requires [PyO3 maturin](https://pyo3.rs/v0.21.2/) to be installed.
This package can be installed with `pip install ik-geo`, 
or from the this directory with `pip install .`

If you plan on developing, you can do it within the python virtual environment, or outside.
When inside, you can use the command `maturin develop` and it will automatically install in your environment.
Outisde of the virtual environment, you can use the following command to build and install with pip:
`source .env/bin/activate && maturin build --out dist && deactivate && pip install dist/* --force-reinstall`


## Notes

There is a GitHub workflow in [`../.github/workflows`](https://github.com/Verdant-Evolution/ik-geo/tree/main/.github/workflows/ik_python.yml) to automatically build this when any changes are made in this folder or to the Rust implementation  
If the commit or pull request is tagged for release, it will also publish to PyPI, so long as the `PYPI_API_TOKEN` secret is set in the GitHub project.  
The GitHub workflow is currently unable to build the NlOpt rust library for many a few Linux Architectures, such as ARMv7.
Temporarily, they have been removed from the build.