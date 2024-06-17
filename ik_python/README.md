# IK PYTHON

This is a python api for the rust implementation of inverse kinematics using PyO3 maturin

There is a GitHub workflow in `../.github/workflows` to automatically build this when any changes are made in this folder or to the Rust implementation  
If the commit or pull request is tagged for release, it will also publish to PyPI, so long as the `PYPI_API_TOKEN` secret is set in the GitHub project.  
This workflow will not currently run as the test cases fail, but to run it anyway, comment out all instances of `needs: test` in the `ik_geo_python.yml` workflow file

## To build/develop

This requires [PyO3 maturin](https://pyo3.rs/v0.21.2/) to be installed.

In the `ik_geo_python` directory, you can run this to install it:
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

Import the `ik_geo_python` package in your code

The API documentation is at `ik_geo_python.pyi`, and there will be typechecking with mypy installed.

Look at `examples/sample.py` for sample usage

## Notes

The GitHub workflow for building this currently fails because the test cases don't pass

This is due to imperfect 1d search in the Rust implementation that this is building off of.

