# IK Geo C++

This is a wrapper for the Rust implementation for C++

## To build

In this directory, run `cmake . && make`.  This will create a dist folder with three files:
 - `ik_geo.h`, a header file to include in a c++ project
 - `libik_cpp.a` (or `libik_cpp.lib` in Windows), a static library that must be included to use ik_geo in c++.
 - `libik_geo.a` (or `libik_geo.lib` in Windows), another static library that must be included as well.

## To use

Include the three files from the build step in the project, and look at `ik_geo.h` for function signatures.

There is an example usage of this in `../examples/cpp`

## To do

 - Test Windows implementation
 - Create a GitHub workflow for generating the dist files
 - Create a moveit package
 - Add a wrapper for forward kinematics and allow for calculating error.
