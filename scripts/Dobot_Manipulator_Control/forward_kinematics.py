#!/home/fer/miniconda3/bin/python
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from casadi import Function
from casadi import jacobian

# Auxiliar values to create the dual quaternion
# Link 1 and Joint 1
theta_1 = ca.MX.sym('theta_1', 1, 1)
n_1 = ca.MX([0.0, 0.0, 1.0])
q_1 = ca.vertcat(ca.cos(theta_1/2), ca.sin(theta_1/2)@n_1)
t_1 = ca.MX([0.0, 0.0, 0.0, 0.1283])


# Auxiliar Transformation Link2
aux_2_1 = np.pi/2
n_2_1 = ca.MX([1.0, 0.0, 0.0])
q_2_1 = ca.vertcat(ca.cos(aux_2_1/2), ca.sin(aux_2_1/2)@n_2_1)
t_2_1 = ca.MX([0.0, 0.0, 0.0, 0.0])

aux_2_2 = -np.pi/2
n_2_2 = ca.MX([0.0, 0.0, 1.0])
q_2_2 = ca.vertcat(ca.cos(aux_2_2/2), ca.sin(aux_2_2/2)@n_2_2)
t_2_2 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Joint Link 2
theta_2 = ca.MX.sym('theta_2', 1, 1)
n_2 = ca.MX([0.0, 0.0, 1.0])
q_2 = ca.vertcat(ca.cos(theta_2/2), ca.sin(theta_2/2)@n_2)
t_2 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Joint Link 3
theta_3 = ca.MX.sym('theta_3', 1, 1)
n_3 = ca.MX([0.0, 0.0, 1.0])
q_3 = ca.vertcat(ca.cos(theta_3/2), ca.sin(theta_3/2)@n_3)
t_3 = ca.MX([0.0, -0.274, 0.0, 0.0])

# Auxiliar Transformation Link4
aux_4_1 = -np.pi/2
n_4_1 = ca.MX([0.0, 0.0, 1.0])
q_4_1 = ca.vertcat(ca.cos(aux_4_1/2), ca.sin(aux_4_1/2)@n_4_1)
t_4_1 = ca.MX([0.0, -0.23, 0.0, 0.1283])

# Joint link 4
theta_4 = ca.MX.sym('theta_4', 1, 1)
n_4 = ca.MX([0.0, 0.0, 1.0])
q_4 = ca.vertcat(ca.cos(theta_4/2), ca.sin(theta_4/2)@n_4)
t_4 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link5
aux_5_1 = np.pi/2
n_5_1 = ca.MX([1.0, 0.0, 0.0])
q_5_1 = ca.vertcat(ca.cos(aux_5_1/2), ca.sin(aux_5_1/2)@n_5_1)
t_5_1 = ca.MX([0.0, 0.0, -0.116, 0.0])

# Joint 5
theta_5 = ca.MX.sym('theta_5', 1, 1)
n_5 = ca.MX([0.0, 0.0, 1.0])
q_5 = ca.vertcat(ca.cos(theta_5/2), ca.sin(theta_5/2)@n_5)
t_5 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link6
aux_6_1 = -np.pi/2
n_6_1 = ca.MX([1.0, 0.0, 0.0])
q_6_1 = ca.vertcat(ca.cos(aux_6_1/2), ca.sin(aux_6_1/2)@n_6_1)
t_6_1 = ca.MX([0.0, 0.0, 0.105, 0.0])

# Joint 6
theta_6 = ca.MX.sym('theta_6', 1, 1)
n_6 = ca.MX([0.0, 0.0, 1.0])
q_6 = ca.vertcat(ca.cos(theta_6/2), ca.sin(theta_6/2)@n_6)
t_6 = ca.MX([0.0, 0.0, 0.0, 0.0])

# DualQuaternion from axis and position
Q1_pose =  DualQuaternion.from_pose(quat = q_1, trans = t_1)
Q2_1_pose =  DualQuaternion.from_pose(quat = q_2_1, trans = t_2_1)
Q2_2_pose =  DualQuaternion.from_pose(quat = q_2_2, trans = t_2_2)
Q2_pose =  DualQuaternion.from_pose(quat = q_2, trans = t_2)
Q3_pose =  DualQuaternion.from_pose(quat = q_3, trans = t_3)
Q4_1_pose =  DualQuaternion.from_pose(quat = q_4_1, trans = t_4_1)
Q4_pose =  DualQuaternion.from_pose(quat = q_4, trans = t_4)
Q5_1_pose =  DualQuaternion.from_pose(quat = q_5_1, trans = t_5_1)
Q5_pose =  DualQuaternion.from_pose(quat = q_5, trans = t_5)
Q6_1_pose =  DualQuaternion.from_pose(quat = q_6_1, trans = t_6_1)
Q6_pose =  DualQuaternion.from_pose(quat = q_6, trans = t_6)


def forward_kinematics_casadi_link1():
    # Compute the Pose based of the quaternion and the trasnation
    values = Q1_pose.get[:, 0]
    f_pose = Function('f_pose', [theta_1], [values])
    return f_pose

def forward_kinematics_casadi_link2():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q2_1_pose * Q2_2_pose * Q2_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_2], [values])
    return f_pose

def forward_kinematics_casadi_link3():
    # Compute the Pose based of the quaternion and the trasnation
    values = Q3_pose.get[:, 0]
    f_pose = Function('f_pose', [theta_3], [values])
    return f_pose

def forward_kinematics_casadi_link4():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q4_1_pose  * Q4_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_4], [values])
    return f_pose

def forward_kinematics_casadi_link5():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q5_1_pose  * Q5_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_5], [values])
    return f_pose

def forward_kinematics_casadi_link6():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q6_1_pose  * Q6_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_6], [values])
    return f_pose

def forward_kinematics_casadi():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q1_pose * Q2_1_pose * Q2_2_pose * Q2_pose * Q3_pose * Q4_1_pose  * Q4_pose * Q5_1_pose  * Q5_pose * Q6_1_pose  * Q6_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6], [values])
    return f_pose

def jacobian_casadi():
    # Compute Jacobian using Dual quaternions
    tranformation = Q1_pose * Q2_1_pose * Q2_2_pose * Q2_pose * Q3_pose * Q4_1_pose  * Q4_pose * Q5_1_pose  * Q5_pose * Q6_1_pose  * Q6_pose
    symbolic_actions = ca.vertcat(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
    J = ca.jacobian(tranformation.get[:, 0], symbolic_actions)
    f_jacobian = Function("Function_J", [symbolic_actions], [J])
    return f_jacobian
    