#!/bin/bash
echo "Installing Python libraries..."
pip install numpy==1.21.2
pip install scipy==1.7.2
pip install sympy
pip install matplotlib
pip install casadi==3.5.1
pip install pluggy==1.3.0
pip install pyyaml
pip install -U rospkg
pip install empy
sudo apt install -y ros-noetic-moveit