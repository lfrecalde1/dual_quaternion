# Dual Quaternion ROS

This is a brief explanation of how to install and run this package on another machine.

Create a new conda environment using Python 3.8 (for example, named "dual_quat") and activate it with the following commands:

```bash
conda create -n dual_quat python=3.8
conda activate dual_quat
```
This project includes an installation file that contains all the necessary dependencies for Python and ROS. Inside the conda environment, execute the following commands:

```bash
chmod +x install_python.sh
./install_python.sh
```

## ROS Workspace

Create a ros workspace using the following commands:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```
Inside the ROS workspace /src must be included:
[dual_quaternion](https://github.com/lfrecalde1/dual_quaternion).

Since the creation of the conda virtual environment, it should be activated before compiling the packages; therefore, execute the following command:

```bash
catkin_make -DPYTHON_EXECUTABLE=~/miniconda3/envs/dual_quat/bin/python
source devel/setup.bash
```
## Differential Kinematics Quaternions Control

It is possible to execute quaternion-based kinematic control, where the purpose is to converge to a desired orientation.

To run the controller taking the shortest path based on numpy, execute the following command:


```bash
roslaunch dual_quaternion quat_numpy.launch
```

<p float="left">
    <img src="videos/trajectory_kine.gif" width="600"  />
 </p>

To run the controller taking the shortest path based on Casadi, execute the following command:


```bash
roslaunch dual_quaternion quat_casadi.launch
```

<p float="left">
    <img src="videos/trajectory_kine.gif" width="600"  />
 </p>

The results of this formulation are presented in the following images.

<p float="left">
    <img src="images/quat_error.png" width="600"  />
 </p>
<p float="left">
    <img src="images/norm.png" width="600"  />
 </p>

## Differential Kinematics DualQuaternions Control

To run the controller taking the shortest path based on numpy, execute the following command:


```bash
roslaunch dual_quaternion dualquaternion.launch
```

<p float="left">
    <img src="videos/DualQuat_trajectory_1.gif" width="600"  />
 </p>

To run the controller taking the shortest path based on Casadi, execute the following command:

```bash
roslaunch dual_quaternion dualquaternion_casadi.launch
```
<p float="left">
    <img src="videos/Dualquat_trajectory_2.gif" width="600"  />
 </p>