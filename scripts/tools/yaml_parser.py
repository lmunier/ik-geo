#!/usr/bin/python3
# -*- coding: utf-8 -*-

""" Helper functions to parse yaml configuration files. """
import yaml

from typing import Union
from math import pi


def extract_param_from_yaml(file_path: str, config: str = "default") -> tuple:
    """Extract DH parameters from a YAML file.

    Args:
        file_path (str): Path to the YAML file.
        config (str): The configuration to extract.

    Returns:
        tuple: The extracted parameters.
    """
    with open(file_path, "r") as file:
        try:
            parameters = yaml.safe_load(file)
            if config not in parameters:
                raise KeyError(
                    f"The configuration '{config}' does not exist in the YAML file."
                )

            dh_config = parameters[config]
            required_keys = ["d", "a", "alpha"]
            for key in required_keys:
                if key not in dh_config:
                    raise KeyError(
                        f"The key '{key}' is missing in the configuration '{config}'."
                    )

            d_val, a_val = dh_config["d"], dh_config["a"]
            alpha_val = evaluate_expression(dh_config["alpha"])

            return d_val, a_val, alpha_val
        except yaml.YAMLError as exc:
            raise ValueError(f"Error reading YAML file: {exc}")


def evaluate_expression(value: Union[int, float, str, list]) -> Union[int, float]:
    """Evaluate the expressions in the list.

    Args:
        value: The value to evaluate.

    Returns:
        The evaluated value.
    """
    if isinstance(value, list):
        evaluation = []
        for val in value:
            evaluation.append(evaluate_expression(val))

        return evaluation
    else:
        if isinstance(value, str):
            return eval(value, {"pi": pi})
        else:
            return value
