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
dual_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1, dw_1,dx_1, dy_1, dz_1)

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


# Creating states of the current dualquaternion
Q1 = DualQuaternion_body(q_real= Quaternion(q = q_1), q_dual = Quaternion(q = d_1))

# Creating the desired quaternion
Q1d = DualQuaternion_body(q_real= Quaternion(q = q_1d), q_dual = Quaternion(q = d_1d))

# Creating the Desired dualquaternion twist
W1d = DualQuaternion_body(q_real= Quaternion(q = Wd), q_dual= Quaternion(q= Vd))

# Gain Matrix as dualquaternion
Kp =  DualQuaternion_body(q_real= Quaternion(q = Kr), q_dual= Quaternion(q= Kd))


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

f_rotation = rotation_casadi()

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


def dual_quat_casadi():
    values = Q1.get[:, 0]
    dualquaternion_f = Function('dualquaternion_f', [dual_1], [values])
    return dualquaternion_f

def dualquat_trans_casadi():
    values = Q1.get_trans.get[:, 0]
    f_trans = Function('f_trans', [dual_1], [values])
    return f_trans

def dualquat_get_real_casadi():
    values = Q1.get_real.get[:, 0]
    f_real = Function('f_real', [dual_1], [values])
    return f_real

def dualquat_get_dual_casadi():
    values = Q1.get_dual.get[:, 0]
    f_dual = Function('f_dual', [dual_1], [values])
    return f_dual

get_real = dualquat_get_real_casadi()
get_dual = dualquat_get_dual_casadi()
get_trans = dualquat_trans_casadi()

def dualquat_quat_casadi():
    values = Q1.get_quat.get[:, 0]
    f_quat = Function('f_quat', [dual_1], [values])
    return f_quat

get_quat = dualquat_quat_casadi()


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
    zeros = ca.DM.zeros(4, 4)
    Hplus = ca.vertcat(ca.horzcat(H_r_plus, zeros),
                        ca.horzcat(H_d_plus, H_r_plus))

    # Auxiliar variable veloicities
    omega = ca.vertcat(0.0, omega[0, 0], omega[1, 0], omega[2, 0], 0.0, omega[3, 0], omega[4, 0], omega[5, 0])
    q_dot = (1/2)*(Hplus@ omega) + aux_dual
    return q_dot

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

def dual_aceleraction_casadi(dual, omega, u, L):
    # Split Control Actions
    force = u[0, 0]
    torques = u[1:4, 0]

    # System Matrices
    J = ca.DM.zeros(3, 3)
    J[0, 0] = L[1]
    J[1, 1] = L[2]
    J[2, 2] = L[3]
    J_1 = ca.inv(J)
    e3 = ca.DM.zeros(3, 1)
    e3[2, 0] = 1.0
    g = L[4]
    m = L[0]

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
    U_d = ((force*(f_rotation(q, e3)))/m) + ca.cross(J_1@torques, p)

    T_r = F_r + U_r
    T_d = F_d + U_d
    T = ca.vertcat(T_r, T_d)

    return T

def error_manifold(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    quat_d_data = qd_conjugate[0:4]
    dual_d_data =  qd_conjugate[4:8]

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


    q_e_aux = Hplus @ q
    
    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux
    expr2 = -q_e_aux

    q_error = ca.if_else(condition1, expr1, expr2) 
    # Check shortest path

    Q3_pose =  ca.DM.zeros(8, 1)
    Q3_pose[0, 0] = 1.0
    
    q_e_ln = Q3_pose - q_error
    return q_e_ln

def error_lie(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    quat_d_data = qd_conjugate[0:4]
    dual_d_data =  qd_conjugate[4:8]

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

    q_e_aux = Hplus @ q
    
    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux
    expr2 = -q_e_aux

    q_error = ca.if_else(condition1, expr1, expr2) 
    # Check shortest path

    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error_real[0])

    ## Dual Part
    H_error_real_plus = ca.vertcat(ca.horzcat(q_error_real_c[0, 0], -q_error_real_c[1, 0], -q_error_real_c[2, 0], -q_error_real_c[3, 0]),
                                ca.horzcat(q_error_real_c[1, 0], q_error_real_c[0, 0], -q_error_real_c[3, 0], q_error_real_c[2, 0]),
                                ca.horzcat(q_error_real_c[2, 0], q_error_real_c[3, 0], q_error_real_c[0, 0], -q_error_real_c[1, 0]),
                                ca.horzcat(q_error_real_c[3, 0], -q_error_real_c[2, 0], q_error_real_c[1, 0], q_error_real_c[0, 0]))

    trans_error = 2 * H_error_real_plus@q_error_dual
    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  angle*q_error_real[1, 0]/norm, angle*q_error_real[2, 0]/norm, angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, trans_error[1, 0], trans_error[2, 0], trans_error[3, 0])
    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)
    return q_e_ln

def error_quaternion(qd, q):
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0], qd[4, 0], -qd[5, 0], -qd[6, 0], -qd[7, 0])
    quat_d_data = qd_conjugate[0:4, 0]
    quaternion = q[0:4, 0]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0, 0], -quat_d_data[1, 0], -quat_d_data[2, 0], -quat_d_data[3, 0]),
                                ca.horzcat(quat_d_data[1, 0], quat_d_data[0, 0], -quat_d_data[3, 0], quat_d_data[2, 0]),
                                ca.horzcat(quat_d_data[2, 0], quat_d_data[3, 0], quat_d_data[0, 0], -quat_d_data[1, 0]),
                                ca.horzcat(quat_d_data[3, 0], -quat_d_data[2, 0], quat_d_data[1, 0], quat_d_data[0, 0]))



    q_e_aux = H_r_plus @ quaternion
    
    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux[:, 0]
    expr2 = -q_e_aux[:, 0]

    # Check shortest path
    q_error = ca.if_else(condition1, expr1, expr2) 

    norm = ca.norm_2(q_error[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error[0])

    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error[1, 0]/norm, (1/2)*angle*q_error[2, 0]/norm, (1/2)*angle*q_error[3, 0]/norm)


    # Sux variable in roder to get a norm
    #q_3_aux = ca.DM([1.0, 0.0, 0.0, 0.0])
    #Q3_pose =  ca.vertcat(q_3_aux)
    
    #q_e_ln = Q3_pose - q_error

    return ln_quaternion
def quadrotorModel(L: list)-> AcadosModel:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # model                                                      - Acados model
    model_name = 'quadrotor'
    constraint = ca.types.SimpleNamespace()
    # Defining Desired Frame
    qw_1d = ca.MX.sym('qw_1d', 1, 1)
    qx_1d = ca.MX.sym('qx_1d', 1, 1)
    qy_1d = ca.MX.sym('qy_1d', 1, 1)
    qz_1d = ca.MX.sym('qz_1d', 1, 1)

    dw_1d = ca.MX.sym("dw_1d", 1, 1)
    dx_1d = ca.MX.sym("dx_1d", 1, 1)
    dy_1d = ca.MX.sym("dy_1d", 1, 1)
    dz_1d = ca.MX.sym("dz_1d", 1, 1)
    
    # Defining the desired Velocity using symbolics
    vx_1d = ca.MX.sym("vx_1d", 1, 1)
    vy_1d = ca.MX.sym("vy_1d", 1, 1)
    vz_1d = ca.MX.sym("vz_1d", 1, 1)

    wx_1d = ca.MX.sym("wx_1d", 1, 1)
    wy_1d = ca.MX.sym("wy_1d", 1, 1)
    wz_1d = ca.MX.sym("wz_1d", 1, 1)


    X = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d,
                   wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)

    # Split States of the system
    twist_1 = X[8:14, 0]
    dualquat_1 = X[0:8, 0]

    # Auxiliary variables implicit function
    qw_1dot = ca.MX.sym('qw_1dot', 1, 1)
    qx_1dot = ca.MX.sym('qx_1dot', 1, 1)
    qy_1dot = ca.MX.sym('qy_1dot', 1, 1)
    qz_1dot = ca.MX.sym('qz_1dot', 1, 1)

    dw_1dot = ca.MX.sym("dw_1dot", 1, 1)
    dx_1dot = ca.MX.sym("dx_1dot", 1, 1)
    dy_1dot = ca.MX.sym("dy_1dot", 1, 1)
    dz_1dot = ca.MX.sym("dz_1dot", 1, 1)

    vx_1dot = ca.MX.sym("vx_1dot", 1, 1)
    vy_1dot = ca.MX.sym("vy_1dot", 1, 1)
    vz_1dot = ca.MX.sym("vz_1dot", 1, 1)

    wx_1dot = ca.MX.sym("wx_1dot", 1, 1)
    wy_1dot = ca.MX.sym("wy_1dot", 1, 1)
    wz_1dot = ca.MX.sym("wz_1dot", 1, 1)

    X_dot = ca.vertcat(qw_1dot, qx_1dot, qy_1dot, qz_1dot, dw_1dot, dx_1dot, dy_1dot, dz_1dot,
                       wx_1dot, wy_1dot, wz_1dot, vx_1dot, vy_1dot, vz_1dot)

    # Control Actions
    F_ref = ca.MX.sym('F_ref')
    tau_1_ref = ca.MX.sym('tau_1_ref')
    tau_2_ref = ca.MX.sym('tau_2_ref')
    tau_3_ref = ca.MX.sym('tau_3_ref')

    u = ca.vertcat(F_ref, tau_1_ref, tau_2_ref, tau_3_ref)

    dualdot = quatdot_simple(dualquat_1, twist_1)
    twistdot = dual_aceleraction_casadi(dualquat_1, twist_1, u, L)

    norm_q = ca.norm_2(get_quat(X[0:8]))
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = ca.vertcat(norm_q)
    constraint.min = 1.0
    constraint.max = 1.0
    # Explicit and implicit functions
    f_expl = ca.vertcat(dualdot, twistdot)
    f_impl = X_dot - f_expl
    p = ca.MX.sym('p', 18, 1)

    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = X
    model.xdot = X_dot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    return model, get_trans, get_quat, constraint, error_manifold, error_quaternion, error_lie
