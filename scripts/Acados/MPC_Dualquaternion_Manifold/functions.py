import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion_body
from casadi import Function
from casadi import jacobian
from acados_template import AcadosModel

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
dual_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1, dw_1,dx_1, dy_1, dz_1)

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
dual_1d = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d)

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
w_1d = ca.vertcat(wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)

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

# Control Actions
tau =  ca.MX.sym("tau", 3, 1)
F =  ca.MX.sym("F", 1, 1)

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

# Compute short path to the desired quaternion
q_error = ca.MX.sym('q_error', 8, 1)

# Define the function
f_error = ca.Function('f_error', [q_error], [ca.if_else(q_error[0, 0] >= 0, q_error, -q_error)])


# Cross product funtion
v1 = ca.MX.sym('v1', 3, 1)
v2 = ca.MX.sym('v2', 3, 1)


# Manually calculate the cross product of v1 and v2
cross_product = ca.vertcat(v1[1]*v2[2] - v1[2]*v2[1],
                           v1[2]*v2[0] - v1[0]*v2[2],
                           v1[0]*v2[1] - v1[1]*v2[0])
f_cross =  ca.Function('f_cross', [v1, v2], [cross_product])

# Quaternion rotation
def rotation_casadi():
    quat_aux_1 = ca.MX.sym('quat_aux_1', 4, 1)
    vector_aux_1 = ca.MX.sym('vector_aux_1', 3, 1)
    vector = ca.vertcat(0.0, vector_aux_1)

    quat = quat_aux_1
    quat_c = ca.vertcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0])

    H_plus_q = ca.vertcat(ca.horzcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0]),
                                ca.horzcat(quat[1, 0], quat[0, 0], -quat[3, 0], quat[2, 0]),
                                ca.horzcat(quat[2, 0], quat[3, 0], quat[0, 0], -quat[1, 0]),
                                ca.horzcat(quat[3, 0], -quat[2, 0], quat[1, 0], quat[0, 0]))

    aux_value = H_plus_q@vector

    H_plus_aux = ca.vertcat(ca.horzcat(aux_value[0, 0], -aux_value[1, 0], -aux_value[2, 0], -aux_value[3, 0]),
                                ca.horzcat(aux_value[1, 0], aux_value[0, 0], -aux_value[3, 0], aux_value[2, 0]),
                                ca.horzcat(aux_value[2, 0], aux_value[3, 0], aux_value[0, 0], -aux_value[1, 0]),
                                ca.horzcat(aux_value[3, 0], -aux_value[2, 0], aux_value[1, 0], aux_value[0, 0]))



    vector_i = H_plus_aux@quat_c


    f_rot =  ca.Function('f_rot', [quat_aux_1, vector_aux_1], [vector_i[1:4, 0]])
    return f_rot

f_rotacion = rotation_casadi()

def rotation_inverse_casadi():
    quat_aux_1 = ca.MX.sym('quat_aux_1', 4, 1)
    vector_aux_1 = ca.MX.sym('vector_aux_1', 3, 1)
    vector = ca.vertcat(0.0, vector_aux_1)

    quat = quat_aux_1
    quat_c = ca.vertcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0])

    H_plus_q_c = ca.vertcat(ca.horzcat(quat_c[0, 0], -quat_c[1, 0], -quat_c[2, 0], -quat_c[3, 0]),
                                ca.horzcat(quat_c[1, 0], quat_c[0, 0], -quat_c[3, 0], quat_c[2, 0]),
                                ca.horzcat(quat_c[2, 0], quat_c[3, 0], quat_c[0, 0], -quat_c[1, 0]),
                                ca.horzcat(quat_c[3, 0], -quat_c[2, 0], quat_c[1, 0], quat_c[0, 0]))

    aux_value = H_plus_q_c@vector

    H_plus_aux = ca.vertcat(ca.horzcat(aux_value[0, 0], -aux_value[1, 0], -aux_value[2, 0], -aux_value[3, 0]),
                                ca.horzcat(aux_value[1, 0], aux_value[0, 0], -aux_value[3, 0], aux_value[2, 0]),
                                ca.horzcat(aux_value[2, 0], aux_value[3, 0], aux_value[0, 0], -aux_value[1, 0]),
                                ca.horzcat(aux_value[3, 0], -aux_value[2, 0], aux_value[1, 0], aux_value[0, 0]))



    vector_b = H_plus_aux@quat


    f_rot_inv =  ca.Function('f_rot_inv', [quat_aux_1, vector_aux_1], [vector_b[1:4, 0]])
    return f_rot_inv


f_rotacion_inverse = rotation_inverse_casadi()

def dual_quat_casadi():
    values = Q1.get[:, 0]
    dualquaternion_f = Function('dualquaternion_f', [dual_1_data], [values])
    return dualquaternion_f

def dualquat_trans_casadi():
    values = Q1.get_trans.get[:, 0]
    f_trans = Function('f_trans', [dual_1_data], [values])
    return f_trans

def dualquat_get_real_casadi():
    values = Q1.get_real.get[:, 0]
    f_real = Function('f_real', [dual_1_data], [values])
    return f_real

def dualquat_get_dual_casadi():
    values = Q1.get_dual.get[:, 0]
    f_dual = Function('f_dual', [dual_1_data], [values])
    return f_dual

get_real = dualquat_get_real_casadi()
get_dual = dualquat_get_dual_casadi()
get_trans = dualquat_trans_casadi()

def dualquat_quat_casadi():
    values = Q1.get_quat.get[:, 0]
    f_quat = Function('f_quat', [dual_1_data], [values])
    return f_quat

get_quat = dualquat_quat_casadi()

def dualquat_from_pose_casadi():
    values = Q1_pose.get[:, 0]
    f_pose = Function('f_pose', [theta_1_aux, nx_aux, ny_aux, nz_aux, tx_1_aux, ty_1_aux, tz_1_aux], [values])
    return f_pose


def quatdot_simple(quat, omega):
    quat_data = quat[0:4, 0]
    dual_data =  quat[4:8, 0]

    # Auxiliary variable in order to avoid numerical issues
    norm_r = ca.dot(quat_data, quat_data)
    K_quat = 2
    norm_d = (quat_data.T@dual_data)

    quat_error = 1 - norm_r
    dual_error = norm_d

    aux_1 = quat_data * (K_quat*quat_error)
    aux_2 = dual_data * (K_quat*dual_error)

    aux_dual = ca.vertcat(aux_1, aux_2)

    # Creatin aux Variables
    H_r_plus = ca.vertcat(ca.horzcat(quat_data[0, 0], -quat_data[1, 0], -quat_data[2, 0], -quat_data[3, 0]),
                                ca.horzcat(quat_data[1, 0], quat_data[0, 0], -quat_data[3, 0], quat_data[2, 0]),
                                ca.horzcat(quat_data[2, 0], quat_data[3, 0], quat_data[0, 0], -quat_data[1, 0]),
                                ca.horzcat(quat_data[3, 0], -quat_data[2, 0], quat_data[1, 0], quat_data[0, 0]))

    H_d_plus = ca.vertcat(ca.horzcat(dual_data[0, 0], -dual_data[1, 0], -dual_data[2, 0], -dual_data[3, 0]),
                                ca.horzcat(dual_data[1, 0], dual_data[0, 0], -dual_data[3, 0], dual_data[2, 0]),
                                ca.horzcat(dual_data[2, 0], dual_data[3, 0], dual_data[0, 0], -dual_data[1, 0]),
                                ca.horzcat(dual_data[3, 0], -dual_data[2, 0], dual_data[1, 0], dual_data[0, 0]))
    zeros = ca.MX.zeros(4, 4)
    Hplus = ca.vertcat(ca.horzcat(H_r_plus, zeros),
                        ca.horzcat(H_d_plus, H_r_plus))

    # Auxiliar variable veloicities
    omega = ca.vertcat(0.0, omega[0, 0], omega[1, 0], omega[2, 0], 0.0, omega[3, 0], omega[4, 0], omega[5, 0])
    q_dot = (1/2)*(Hplus@ omega) + aux_dual
    return q_dot

def f_rk4_casadi_simple(quat= dual_1d, omega=w_1d, ts=ts):
    # Function that computes 
    k1 = quatdot_simple(quat, omega)
    k2 = quatdot_simple(quat + (1/2)*k1*ts, omega)
    k3 = quatdot_simple(quat + (1/2)*k2*ts, omega)
    k4 = quatdot_simple(quat + (1)*k3*ts, omega)
    # Compute forward Euler method
    quat = quat + (1/6)*ts*(k1 + 2*k2 + 2*k3 + k4)
    #quat = quat + (k1)*ts
    f_rk4 = Function('f_rk4', [dual_1d, w_1d, ts], [quat])
    return f_rk4

def dual_velocity_casadi(twist = w_1d, dualquat = dual_1d):

    twist = ca.vertcat(0.0, twist[0:3, 0], 0.0, twist[3:6, 0])
    w_aux = get_real(twist)
    w = w_aux[1:4, 0]

    v_aux = get_dual(twist)
    v = v_aux[1:4, 0]

    p_aux = get_trans(dualquat)
    p = p_aux[1:4, 0]

    real = w
    dual = v + ca.cross(w, p)
    
    velocity = ca.vertcat(real, dual)
    f_velocity = Function('f_velocity', [w_1d, dual_1d], [velocity])
    return f_velocity


def velocities_from_twist_casadi(twist = w_1d, dualquat = dual_1d):
    # Get Real and dual values 
    twist = ca.vertcat(0.0, twist[0:3, 0], 0.0, twist[3:6, 0])

    w_aux = get_real(twist)
    w = w_aux[1:4, 0]

    dual_aux = get_dual(twist)
    dual = dual_aux[1:4, 0]

    p_aux = get_trans(dualquat)
    p = p_aux[1:4, 0]

    v_i = dual - ca.cross(w, p)

    velocity = ca.vertcat(w, v_i)
    f_velocity = Function('f_velocity', [w_1d, dual_1d], [velocity])
    return f_velocity


velocity = velocities_from_twist_casadi()

def quatTorot_c(quat):
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # R                                                          - rotational matrix

    # Normalized quaternion
    q = quat
    q = q/(q.T@q)

    # Create empty variable
    #q_hat = ca.MX.zeros(3, 3)
    #q_hat[0, 1] = -q[3]
    #q_hat[0, 2] = q[2]
    #q_hat[1, 2] = -q[1]
    #q_hat[1, 0] = q[3]
    #q_hat[2, 0] = -q[2]
    #q_hat[2, 1] = q[1]

    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Q = ca.vertcat(
        ca.horzcat(q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)),
        ca.horzcat(2*(q1*q2+q0*q3), q0**2+q2**2-q1**2-q3**2, 2*(q2*q3-q0*q1)),
        ca.horzcat(2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2+q3**2-q1**2-q2**2))

    # Compute Rotational Matrix
    #R = ca.MX.eye(3) + 2 * (q_hat@q_hat) + 2 * q[0] * q_hat
    R = Q
    return R

def dual_aceleraction_casadi(omega = w_1d, dual = dual_1d, force = F, torques = tau):
    # Constant values 
    J = ca.MX.zeros(3, 3)
    J[0, 0] = 2.64e-3
    J[1, 1] = 2.64e-3
    J[2, 2] = 4.96e-3
    J_1 = ca.inv(J)
    e3 = ca.MX.zeros(3, 1)
    e3[2, 0] = 1.0
    g = 9.8
    m = 1

    # Compute linear and angular velocity from twist velocity
    angular_b_linear_i = velocity(omega, dual)
    w = angular_b_linear_i[0:3, 0]
    v = angular_b_linear_i[3:6, 0]
    p = get_trans(dual)
    q = get_quat(dual)
    p = p[1:4, 0]

    # Compute unforced part
    a = ca.cross(-J_1@w, J@w)
    F_r = a
    F_d = ca.cross(a, p) + ca.cross(w, v) - g*e3

    # Compute forced part
    U_r = J_1@torques
    U_d = ((force*(f_rotacion(q, e3)))/m) + ca.cross(J_1@torques, p)

    T_r = F_r + U_r
    T_d = F_d + U_d
    T = ca.vertcat(T_r, T_d)

    return T

def f_rk4_dynamics_casadi_simple(omega=w_1d, quat = dual_1, force = F, torques = tau, ts=ts):
    # Function that computes 
    k1 = dual_aceleraction_casadi(omega, quat, force, torques)
    k2 = dual_aceleraction_casadi(omega + (1/2)*k1*ts, quat, force, torques)
    k3 = dual_aceleraction_casadi(omega + (1/2)*k2*ts, quat, force, torques)
    k4 = dual_aceleraction_casadi(omega + (1)*k3*ts, quat, force, torques)
    # Compute forward Euler method
    omega = omega + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*ts
    #quat = quat + (k1)*ts
    f_rk4_dynamics = Function('f_rk4_dynamics', [w_1d, dual_1, F, tau, ts], [omega])
    return f_rk4_dynamics