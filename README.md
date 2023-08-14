<h1>Stewart-Platform</h1> 

This repository contains the Hexapod class, a Python-based representation of a hexapod robot. The Hexapod class provides functions to simulate the inverse and forward kinematics of the robot, along with visualization functions to show the robot in a 3D space.

<H2> Features</H2>

Configuration of the robot body and leg dimensions.\
Inverse Kinematics (IK) computation.\
Forward Kinematics (FK) computation with iterative error correction.\
3D visualization of the robot with base, body, and leg representations.\
Computation of body transformation using Euler angles and rotation matrices.\
Jacobian computation for kinematic transformations.

<H2>Methods</H2>

<H3>Initialization</H3>
__init__(self, upper_rad=150, lower_rad=250, upper_lim=1000, lower_lim=10, alpha=60, beta=60, euler_order='zyz'): Initializes the Hexapod object with specified dimensions and angles.

<H3>Kinematics</H3>
ik(self, pose): Computes the Inverse Kinematics for a given pose.

fk(self, leg_lengths, error, guess_pose): Computes the Forward Kinematics iteratively until the error is minimal.

<H3>Visualization</H3>
show_robot(self): Visualizes the robot in 3D space.

<H3>Internals</H3>
Several methods to handle robot transformations, Euler angle conversions, Jacobian computation, and other utility functions.
