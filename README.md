# Quadruped Three-Revolute-Joint (3R) IK

A small C++ library for computing inverse kinematics (IK) solutions for quadruped robots with 3R legs. This implementation gratefully uses the modeling equations provided in Prof. Kris Hauser's [online course notes]((https://motion.cs.illinois.edu/RoboticSystems)) from CS 498 at the University of Illinois Urbana-Champaign.

Tested on Ubuntu 22.04 (WSL).

## Build
- Clone this repo somewhere on your Linux machine: ```git clone https://github.com/juansala/quad_3R_IK.git```
- ```cd quad_3r_ik && mkdir build```
- ```cd build && cmake ..```
- ```cmake --build .```

## Run Examples
- Run the demo ```quadruped_ik```
- ```./build/demo```