#!/bin/bash
echo "Installing Python libraries..."
pip install numpy==1.24.4
pip install scipy==1.8.1
pip install attrs==19.2.0
pip install pluggy==1.3.0
pip install tomli
pip install sympy
pip install matplotlib
pip install casadi==3.6.3
pip install pyyaml
pip install -U rospkg
pip install empy
sudo apt install -y ros-noetic-moveit
pip install jupyter