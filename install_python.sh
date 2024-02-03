#!/bin/bash
echo "Installing Python libraries..."
pip install numpy==1.24.4
pip install scipy==1.7.2
pip install sympy
pip install matplotlib
pip install casadi==3.6.3
pip install pluggy==1.3.0
pip install pyyaml
pip install -U rospkg
pip install empy
sudo apt install -y ros-noetic-moveit