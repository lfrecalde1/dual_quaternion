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
qw_1_aux = ca.MX.sym('qw_1_aux', 1, 1)
qx_1_aux = ca.MX.sym('qx_1_aux', 1, 1)
qy_1_aux = ca.MX.sym('qy_1_aux', 1, 1)
qz_1_aux = ca.MX.sym('qz_1_aux', 1, 1)
q_1_aux = ca.vertcat(qw_1_aux, qx_1_aux, qy_1_aux, qz_1_aux)
tx_1_aux = ca.MX.sym("tx_1_aux", 1, 1)
ty_1_aux = ca.MX.sym("ty_1_aux", 1, 1)
tz_1_aux = ca.MX.sym("tz_1_aux", 1, 1)
t_1_aux = ca.vertcat(0.0, tx_1_aux, ty_1_aux, tz_1_aux)

qw_2_aux = ca.MX.sym('qw_2_aux', 1, 1)
qx_2_aux = ca.MX.sym('qx_2_aux', 1, 1)
qy_2_aux = ca.MX.sym('qy_2_aux', 1, 1)
qz_2_aux = ca.MX.sym('qz_2_aux', 1, 1)
q_2_aux = ca.vertcat(qw_2_aux, qx_2_aux, qy_2_aux, qz_2_aux)
tx_2_aux = ca.MX.sym("tx_2_aux", 1, 1)
ty_2_aux = ca.MX.sym("ty_2_aux", 1, 1)
tz_2_aux = ca.MX.sym("tz_2_aux", 1, 1)
t_2_aux = ca.vertcat(0.0, tx_2_aux, ty_2_aux, tz_2_aux)

# DualQuaternion from axis and position
Q1_pose =  DualQuaternion.from_pose(quat = q_1_aux, trans = t_1_aux)
Q2_pose =  DualQuaternion.from_pose(quat = q_2_aux, trans = t_2_aux)

def dualquat_from_pose_casadi():
    # Compute the Pose based of the quaternion and the trasnation
    values = Q1_pose.get[:, 0]
    f_pose = Function('f_pose', [qw_1_aux, qx_1_aux, qy_1_aux, qz_1_aux, tx_1_aux, ty_1_aux, tz_1_aux], [values])
    return f_pose

def dual_multiplication(q1, q2):
    q_2 = ca.vertcat(q2[0], q2[1], q2[2], q2[3], q2[4], q2[5], q2[6], q2[7])
    quat_d_data = q1[0:4]
    dual_d_data =  q1[4:8]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0], -quat_d_data[1], -quat_d_data[2], -quat_d_data[3]),
                                ca.horzcat(quat_d_data[1], quat_d_data[0], -quat_d_data[3], quat_d_data[2]),
                                ca.horzcat(quat_d_data[2], quat_d_data[3], quat_d_data[0], -quat_d_data[1]),
                                ca.horzcat(quat_d_data[3], -quat_d_data[2], quat_d_data[1], quat_d_data[0]))

    H_d_plus = ca.vertcat(ca.horzcat(dual_d_data[0], -dual_d_data[1], -dual_d_data[2], -dual_d_data[3]),
                                ca.horzcat(dual_d_data[1], dual_d_data[0], -dual_d_data[3], dual_d_data[2]),
                                ca.horzcat(dual_d_data[2], dual_d_data[3], dual_d_data[0], -dual_d_data[1]),
                                ca.horzcat(dual_d_data[3], -dual_d_data[2], dual_d_data[1], dual_d_data[0]))
    zeros = ca.DM.zeros(4, 4)
    Hplus = ca.vertcat(ca.horzcat(H_r_plus, zeros),
                        ca.horzcat(H_d_plus, H_r_plus))


    dual_product = Hplus @ q_2

    return dual_product
    
