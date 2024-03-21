import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from casadi import Function
from casadi import jacobian
from acados_template import AcadosModel

# Auxiliar values to create the dual quaternion
theta_1_aux = ca.MX.sym('theta_1_aux', 1, 1)
nx_aux = ca.MX.sym('nx_aux', 1, 1)
ny_aux = ca.MX.sym('ny_aux', 1, 1)
nz_aux = ca.MX.sym('nz_aux', 1, 1)
n_1_aux = ca.vertcat(nx_aux, ny_aux, nz_aux)
q_1_aux = ca.vertcat(ca.cos(theta_1_aux/2), ca.sin(theta_1_aux/2)@n_1_aux)
tx_1_aux = ca.MX.sym("tx_1_aux", 1, 1)
ty_1_aux = ca.MX.sym("ty_1_aux", 1, 1)
tz_1_aux = ca.MX.sym("tz_1_aux", 1, 1)
t_1_aux = ca.vertcat(0.0, tx_1_aux, ty_1_aux, tz_1_aux)

# DualQuaternion from axis and position
Q1_pose =  DualQuaternion.from_pose(quat = q_1_aux, trans = t_1_aux)

def dualquat_from_pose_casadi():
    # Compute the Pose based of the quaternion and the trasnation
    values = Q1_pose.get[:, 0]
    f_pose = Function('f_pose', [theta_1_aux, nx_aux, ny_aux, nz_aux, tx_1_aux, ty_1_aux, tz_1_aux], [values])
    return f_pose

