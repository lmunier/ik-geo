# IK PYTHON

This is a python api for the rust implementation of inverse kinematics using PyO3 maturin

## To build/develop

This requires [PyO3 maturin](https://pyo3.rs/v0.21.2/) to be installed.

In the `ik_python` directory, you can run this to install it:
```bash
python -m venv .env
source .env/bin/activate
pip install maturin
```

If you plan on developing, you can do it within the python virtual environment, or outside.
When inside, you can use the command `maturin develop` and it will automatically install in your environment.
Outisde of the virtual environment, you can use the following command to build and install with pip:
`source .env/bin/activate && maturin build --out dist && deactivate && pip install dist/* --force-reinstall`


## To use

Import the `ik_python` package in your code

The API documentation is at `ik_python.pyi`, and there will be typechecking with mypy installed.

Look at `examples/sample.py` for sample usage
