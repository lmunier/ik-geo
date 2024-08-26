#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert a robot defined in Denavit-Hartenberg convention to Product
of Exponentials.
"""

import os
import argparse
import numpy as np

from yaml_parser import extract_param_from_yaml as parse_yaml
from help_maths import axis_rot


def dh_to_kin(d_val: np.ndarray, a_val: np.ndarray, alpha_val: np.ndarray) -> dict:
    """Convert a robot defined in Denavit-Hartenberg convention to
    Product of Exponentials.

    Args:
        alpha_val (np.array): The alpha vector.
        a_vec (np.array): The a vector.
        d_vec (np.array): The d vector.

    Returns:
        dict: The kinematic structure of the robot.
    """
    N = len(alpha_val)
    kin = {
        "joint_type": np.zeros(N),
        "H": np.full((3, N), np.nan),
        "P": np.full((3, N + 1), np.nan),
    }

    kin["P"][:, 0] = [0, 0, 0]
    kin["H"][:, 0] = [0, 0, 1]
    R = np.eye(3)

    for i in range(N):
        # Translate d_i along z_{i-1}
        # Move a along x_{i-1}
        kin["P"][:, i + 1] = d_val[i] * R[:, 2] + a_val[i] * R[:, 0]

        # Rotate by alpha along x_{i-1}
        R = axis_rot(R[:, 0], alpha_val[i]) @ R

        if i == N - 1:
            kin["RT"] = axis_rot(R[:, 0], alpha_val[i])
        else:
            kin["H"][:, i + 1] = R[:, 2]  # Joint axis is z axis

    return kin


def main(d_val, a_val, alpha_val):
    kin = dh_to_kin(d_val, a_val, alpha_val)

    # Set print options for better readability
    np.set_printoptions(precision=4, suppress=True)

    print("Joint Types:", kin["joint_type"])
    print("H Matrix:\n", np.array2string(kin["H"].T, separator=","))
    print("P Matrix:\n", np.array2string(kin["P"].T, separator=","))
    if "RT" in kin:
        print("RT Matrix:\n", kin["RT"])


if __name__ == "__main__":
    parent_dir = os.path.dirname(os.path.abspath(os.path.join(__file__, os.pardir)))

    parser = argparse.ArgumentParser(description="DH to Kinematics Converter")
    parser.add_argument(
        "--yaml_file",
        type=str,
        default=parent_dir + "/config/dh_config.yaml",
        help="Path to the YAML file",
    )

    parser.add_argument(
        "--config",
        type=str,
        default="default",
        help="Configuration name inside the YAML file",
    )

    args = parser.parse_args()

    d_val, a_val, alpha_val = None, None, None
    try:
        d_val, a_val, alpha_val = parse_yaml(args.yaml_file, args.config)
        print(f"d: {d_val}, a: {a_val}, alpha: {alpha_val}")
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(e)

    print("\nRunning DH to Kinematics conversion:")
    print(f"YAML file: {args.yaml_file} - Configuration: {args.config}")
    print(f"DH Parameters: d={d_val}, a={a_val}, alpha={alpha_val}")
    main(d_val, a_val, alpha_val)
