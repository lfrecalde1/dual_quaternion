import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion_body
from casadi import Function

# Defining Dual Quaternion informtatio
qw_1 = ca.MX.sym('qw_1', 1, 1)
qx_1 = ca.MX.sym('qx_1', 1, 1)
qy_1 = ca.MX.sym('qy_1', 1, 1)
qz_1 = ca.MX.sym('qz_1', 1, 1)
q_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1)
dw_1 = ca.MX.sym("dw_1", 1, 1)
dx_1 = ca.MX.sym("dx_1", 1, 1)
dy_1 = ca.MX.sym("dy_1", 1, 1)
dz_1 = ca.MX.sym("dz_1", 1, 1)
d_1 = ca.vertcat(dw_1, dx_1, dy_1, dz_1)

# Creating auxiliar variables
dual_1_data = ca.vertcat(qw_1, qx_1, qy_1, qz_1, dw_1,dx_1, dy_1, dz_1)

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

# Auxiliar variable thata includes the symbolic variables
aux_symbolic_total_aux = ca.vertcat(theta_1_aux, nx_aux, ny_aux, nz_aux, tx_1_aux, ty_1_aux, tz_1_aux)

# Defining Desired Frame
qw_1d = ca.MX.sym('qw_1d', 1, 1)
qx_1d = ca.MX.sym('qx_1d', 1, 1)
qy_1d = ca.MX.sym('qy_1d', 1, 1)
qz_1d = ca.MX.sym('qz_1d', 1, 1)
q_1d = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d)

dw_1d = ca.MX.sym("dw_1d", 1, 1)
dx_1d = ca.MX.sym("dx_1d", 1, 1)
dy_1d = ca.MX.sym("dy_1d", 1, 1)
dz_1d = ca.MX.sym("dz_1d", 1, 1)
d_1d = ca.vertcat(dw_1d, dx_1d, dy_1d, dz_1d)

# Symbolic Variables
dual_1d_data = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d)

# Defining the desired Velocity using symbolics
vx_1d = ca.MX.sym("vx_1d", 1, 1)
vy_1d = ca.MX.sym("vy_1d", 1, 1)
vz_1d = ca.MX.sym("vz_1d", 1, 1)

wx_1d = ca.MX.sym("wx_1d", 1, 1)
wy_1d = ca.MX.sym("wy_1d", 1, 1)
wz_1d = ca.MX.sym("wz_1d", 1, 1)

Vd = ca.vertcat(0.0, vx_1d, vy_1d, vz_1d)
Wd = ca.vertcat(0.0, wx_1d, wy_1d, wz_1d)

# Symbolic variables desired velocities
w_1d_data = ca.vertcat(wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)

# Defining the control gains using symbolic variables
kr1 = ca.MX.sym("kr1", 1, 1)
kr2 = ca.MX.sym("kr2", 1, 1)
kr3 = ca.MX.sym("kr3", 1, 1)

kd1 = ca.MX.sym("kd1", 1, 1)
kd2 = ca.MX.sym("kd2", 1, 1)
kd3 = ca.MX.sym("kd3", 1, 1)


Kr = ca.vertcat(0.0, kr1, kr2, kr3)
Kd = ca.vertcat(0.0, kd1, kd2, kd3)

k_data = ca.vertcat(kr1, kr2, kr3, kd1, kd2, kd3)

# Sample time symbolic
ts = ca.MX.sym("ts", 1, 1)

# Creating states of the current dualquaternion
Q1 = DualQuaternion_body(q_real= Quaternion(q = q_1), q_dual = Quaternion(q = d_1))

# Creating the desired quaternion
Q1d = DualQuaternion_body(q_real= Quaternion(q = q_1d), q_dual = Quaternion(q = d_1d))

# Creating the Desired dualquaternion twist
W1d = DualQuaternion_body(q_real= Quaternion(q = Wd), q_dual= Quaternion(q= Vd))

# Gain Matrix as dualquaternion
Kp =  DualQuaternion_body(q_real= Quaternion(q = Kr), q_dual= Quaternion(q= Kd))

# DualQuaternion from axis and position
Q1_pose =  DualQuaternion_body.from_pose(quat = q_1_aux, trans = t_1_aux)

def dual_quat_casadi():
    values = Q1.get[:, 0]
    f_trans = Function('f_trans', [dual_1_data], [values])
    return f_trans

def dualquat_trans_casadi():
    values = Q1.get_trans.get[:, 0]
    f_trans = Function('f_trans', [dual_1_data], [values])
    return f_trans

def dualquat_quat_casadi():
    values = Q1.get_quat.get[:, 0]
    f_quat = Function('f_quat', [dual_1_data], [values])
    return f_quat

def dualquat_from_pose_casadi():
    values = Q1_pose.get[:, 0]
    f_quat = Function('f_quat', [theta_1_aux, nx_aux, ny_aux, nz_aux, tx_1_aux, ty_1_aux, tz_1_aux], [values])
    return f_quat


def quatdot(quat, omega):
    quat_data = quat.get_real
    dual_data =  quat.get_dual

    # Auxiliary variable in order to avoid numerical issues
    norm_r, norm_d = quat.norm_dual
    K_quat = 2
    quat_error = 1 - norm_r
    dual_error = norm_d

    aux_1 = quat_data * (K_quat*quat_error)
    aux_2 = dual_data * (K_quat*dual_error)

    aux_dual = DualQuaternion_body(q_real = aux_1, q_dual = aux_2)

    q_dot = (1/2)*(quat * omega) + aux_dual
    #q_dot = (1/2)*(quat * omega)
    return q_dot

def f_rk4_casadi(quat= Q1d, omega=W1d, ts=ts, quat_data=dual_1d_data, omega_data=w_1d_data, ts_data = ts):
    # Function that computes 
    k1 = quatdot(quat, omega)
    k2 = quatdot(quat + (1/2)*k1*ts, omega)
    k3 = quatdot(quat + (1/2)*k2*ts, omega)
    k4 = quatdot(quat + (1)*k3*ts, omega)
    # Compute forward Euler method
    quat = quat + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*ts
    f_rk4 = Function('f_rk4', [quat_data, omega_data, ts_data], [quat.get[:, 0]])
    return f_rk4

def dual_velocity_casadi(W = W1d, dual = Q1d, W_data = w_1d_data, dual_data = dual_1d_data):
    w = W.get_real
    v = W.get_dual
    p = dual.get_trans

    real = w
    dual = v + w.cross(p)
    
    dual_velocity = DualQuaternion_body(q_real = real, q_dual = dual)
    aux = dual_velocity.get[:, 0]
    velocity = ca.vertcat(aux[1, 0], aux[2, 0], aux[3, 0], aux[5, 0], aux[6, 0], aux[7, 0])
    f_velocity = Function('f_velocity', [W_data, dual_data], [velocity])
    return f_velocity

def dual_velocity_body_casadi(dual_velocity = W1d, Q_current = Q1d, W_data = w_1d_data, dual_data = dual_1d_data):
    # Get Real and dual values 
    real = dual_velocity.get_real
    dual = dual_velocity.get_dual

    # Compute the Linear Velocity
    quat = Q_current.get_quat
    quat_c = quat.conjugate()
    v = dual
    w = real

    # Transformation to the body frame
    v_body = quat * v * quat_c

    dual_velocity = DualQuaternion_body(q_real = w, q_dual = v_body)
    aux = dual_velocity.get[:, 0]
    velocity = ca.vertcat(aux[1, 0], aux[2, 0], aux[3, 0], aux[5, 0], aux[6, 0], aux[7, 0])
    f_velocity = Function('f_velocity', [W_data, dual_data], [velocity])
    return f_velocity

def velocities_body_from_twist_casadi(dual_velocity = W1d, Q_current = Q1d, W_data = w_1d_data, dual_data = dual_1d_data):
    # Get Real and dual values 
    w_body = dual_velocity.get_real
    dual = dual_velocity.get_dual

    # Compute the Linear Velocity
    quat = Q_current.get_quat
    quat_c = quat.conjugate()
    p = Q_current.get_trans
    v_body = dual - w_body.cross(p)
    v_body = quat_c * v_body * quat


    dual_velocity = DualQuaternion_body(q_real = w_body, q_dual = v_body)
    aux = dual_velocity.get[:, 0]
    velocity = ca.vertcat(aux[1, 0], aux[2, 0], aux[3, 0], aux[5, 0], aux[6, 0], aux[7, 0])
    f_velocity = Function('f_velocity', [W_data, dual_data], [velocity])
    return f_velocity


def dual_control_casadi(qd = Q1d, wd = W1d, q =Q1, kp = Kp, qd_data = dual_1d_data, wd_data = w_1d_data, q_data = dual_1_data, kp_data = k_data):

    q_e_aux = qd.conjugate() * q
    
    condition1 = q_e_aux.get[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux.get[:, 0]
    expr2 = -q_e_aux.get[:, 0]

    # Nested if_else to implement multiple branches
    q_error = ca.if_else(condition1, expr1, expr2) 

    q_e = DualQuaternion_body(q_real=Quaternion(q = q_error[0:4, 0]), q_dual= Quaternion(q = q_error[4:8, 0]))
    # Apply log mapping
    q_e_ln = q_e.ln_control()

    # Conjugate
    q_e_c = q_e.conjugate()

    # Control Law 
    U = -2*q_e_ln.vector_dot_product(kp) + q_e_c * wd * q_e
    aux = U.get[:, 0]
    control_values = ca.vertcat(aux[1, 0], aux[2, 0], aux[3, 0], aux[5, 0], aux[6, 0], aux[7, 0])

    f_control = Function('f_control', [qd_data, wd_data, q_data, kp_data], [control_values])
    return f_control

def lyapunov_casadi(qd = Q1d, q =Q1, qd_data = dual_1d_data, q_data = dual_1_data):
    #  Control Error Split Values
    q_e_aux = qd.conjugate() * q
    
    condition1 = q_e_aux.get[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux.get[:, 0]
    expr2 = -q_e_aux.get[:, 0]

    # Nested if_else to implement multiple branches
    q_error = ca.if_else(condition1, expr1, expr2) 

    q_e = DualQuaternion_body(q_real=Quaternion(q = q_error[0:4, 0]), q_dual= Quaternion(q = q_error[4:8, 0]))

    q_e_ln = q_e.ln_control()

    P =  1*ca.MX.eye(8)

    q_e_ln_data = q_e_ln.get[:, 0]

    norm_lie = q_e_ln_data.T@P@q_e_ln_data
    
    v = norm_lie
    v_f = Function('v_f', [qd_data, q_data], [v])
    return v_f