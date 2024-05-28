import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from casadi import Function
from casadi import jacobian
from acados_template import AcadosModel

# Sample time symbolic
ts = ca.MX.sym("ts", 1, 1)

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
Q1 = DualQuaternion(q_real= Quaternion(q = q_1), q_dual = Quaternion(q = d_1))

# Creating the desired quaternion
Q1d = DualQuaternion(q_real= Quaternion(q = q_1d), q_dual = Quaternion(q = d_1d))

# Creating the Desired dualquaternion twist
W1d = DualQuaternion(q_real= Quaternion(q = Wd), q_dual= Quaternion(q= Vd))

# Quaternion rotation
def rotation_casadi():
    # Function that enables the rotation of a vector using quaternions

    # Creation of the symbolic variables for the quaternion and the vector
    quat_aux_1 = ca.MX.sym('quat_aux_1', 4, 1)
    vector_aux_1 = ca.MX.sym('vector_aux_1', 3, 1)

    # Defining the pure quaternion based on the vector information
    vector = ca.vertcat(0.0, vector_aux_1)

    # Compute conjugate of the quaternion
    quat = quat_aux_1
    quat_c = ca.vertcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0])

    # v' = q x v x q*
    # Rotation to the inertial frame

    H_plus_q = ca.vertcat(ca.horzcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0]),
                                ca.horzcat(quat[1, 0], quat[0, 0], -quat[3, 0], quat[2, 0]),
                                ca.horzcat(quat[2, 0], quat[3, 0], quat[0, 0], -quat[1, 0]),
                                ca.horzcat(quat[3, 0], -quat[2, 0], quat[1, 0], quat[0, 0]))

    # Computing the first multiplication
    aux_value = H_plus_q@vector

    # Multiplication by the conjugate part
    H_plus_aux = ca.vertcat(ca.horzcat(aux_value[0, 0], -aux_value[1, 0], -aux_value[2, 0], -aux_value[3, 0]),
                                ca.horzcat(aux_value[1, 0], aux_value[0, 0], -aux_value[3, 0], aux_value[2, 0]),
                                ca.horzcat(aux_value[2, 0], aux_value[3, 0], aux_value[0, 0], -aux_value[1, 0]),
                                ca.horzcat(aux_value[3, 0], -aux_value[2, 0], aux_value[1, 0], aux_value[0, 0]))

    # Computing the vector rotate respect the quaternion
    vector_i = H_plus_aux@quat_c

    # Create function
    f_rot =  ca.Function('f_rot', [quat_aux_1, vector_aux_1], [vector_i[1:4, 0]])
    return f_rot


def rotation_inverse_casadi():
    # Creation of the symbolic variables for the quaternion and the vector
    quat_aux_1 = ca.MX.sym('quat_aux_1', 4, 1)
    vector_aux_1 = ca.MX.sym('vector_aux_1', 3, 1)

    # Auxiliary pure quaternion based on the information of the vector
    vector = ca.vertcat(0.0, vector_aux_1)

    # Quaternion
    quat = quat_aux_1

    # Quaternion conjugate
    quat_c = ca.vertcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0])
    # v' = q* x v x q 
    # Rotation to the body Frame

    # QUaternion Multiplication vector form
    H_plus_q_c = ca.vertcat(ca.horzcat(quat_c[0, 0], -quat_c[1, 0], -quat_c[2, 0], -quat_c[3, 0]),
                                ca.horzcat(quat_c[1, 0], quat_c[0, 0], -quat_c[3, 0], quat_c[2, 0]),
                                ca.horzcat(quat_c[2, 0], quat_c[3, 0], quat_c[0, 0], -quat_c[1, 0]),
                                ca.horzcat(quat_c[3, 0], -quat_c[2, 0], quat_c[1, 0], quat_c[0, 0]))

    # First Multiplication
    aux_value = H_plus_q_c@vector

    # Quaternion multiplication second element
    H_plus_aux = ca.vertcat(ca.horzcat(aux_value[0, 0], -aux_value[1, 0], -aux_value[2, 0], -aux_value[3, 0]),
                                ca.horzcat(aux_value[1, 0], aux_value[0, 0], -aux_value[3, 0], aux_value[2, 0]),
                                ca.horzcat(aux_value[2, 0], aux_value[3, 0], aux_value[0, 0], -aux_value[1, 0]),
                                ca.horzcat(aux_value[3, 0], -aux_value[2, 0], aux_value[1, 0], aux_value[0, 0]))

    # Rotated vector repected to the body frame
    vector_b = H_plus_aux@quat

    # Defining function using casadi
    f_rot_inv =  ca.Function('f_rot_inv', [quat_aux_1, vector_aux_1], [vector_b[1:4, 0]])
    return f_rot_inv

# Creating functions which are going to be used later
f_rotation = rotation_casadi()
f_rotation_inverse = rotation_inverse_casadi()

def dual_quat_casadi():
    # Function that obtains the elements of the dual quaternion  real an dual part
    values = Q1.get[:, 0]
    dualquaternion_f = Function('dualquaternion_f', [dual_1], [values])
    return dualquaternion_f

def dualquat_trans_casadi():
    # Functions that computes the translation from a dual quaternion
    values = Q1.get_trans.get[:, 0]
    f_trans = Function('f_trans', [dual_1], [values])
    return f_trans

def dualquat_get_real_casadi():
    # Function that get the real part form the dual quaternon
    values = Q1.get_real.get[:, 0]
    f_real = Function('f_real', [dual_1], [values])
    return f_real

def dualquat_get_dual_casadi():
    # Function that gets the dual part of the dualquaternion
    values = Q1.get_dual.get[:, 0]
    f_dual = Function('f_dual', [dual_1], [values])
    return f_dual

def dualquat_quat_casadi():
    # Function that get the quaternion of the dualquaternion, this elemens is the same as the real part
    values = Q1.get_quat.get[:, 0]
    f_quat = Function('f_quat', [dual_1], [values])
    return f_quat

# Creating Functions
get_real = dualquat_get_real_casadi()
get_dual = dualquat_get_dual_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()

# Creation of dualquaternion kinemtics
def quatdot_simple(quat, omega):
    # Functions that computes the differential kinematics based on dualquaternions

    # Split values real and dual
    quat_data = quat[0:4, 0]
    dual_data =  quat[4:8, 0]

    # Auxiliary variable in order to avoid numerical issues
    #norm_r = ca.dot(quat_data, quat_data)
    K_quat = 10
    #norm_d = 2*(quat_data.T@dual_data)

    norm_r = ca.norm_2(quat_data)
    norm_d = 2* ca.dot(quat_data, dual_data)

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

# Runge Kutta 4th
def f_rk4_casadi_simple(quat= dual_1d, omega=w_1d, t_s=ts):
    # Function that computes 
    k1 = quatdot_simple(quat, omega)
    k2 = quatdot_simple(quat + (1/2)*k1*t_s, omega)
    k3 = quatdot_simple(quat + (1/2)*k2*t_s, omega)
    k4 = quatdot_simple(quat + (1)*k3*t_s, omega)
    # Compute forward Euler method
    quat = quat + (1/6)*t_s*(k1 + 2*k2 + 2*k3 + k4)
    #quat = quat + (k1)*ts
    f_rk4 = Function('f_rk4', [dual_1d, w_1d, ts], [quat])
    return f_rk4

def dual_velocity_casadi(twist = w_1d, dualquat = dual_1d):
    # Funtions that computes the twist based on dualquaternions
    twist = ca.vertcat(0.0, twist[0:3, 0], 0.0, twist[3:6, 0])
    w_aux = get_real(twist)
    w = w_aux[1:4, 0]

    v_aux = get_dual(twist)
    v = v_aux[1:4, 0]

    quat_aux = get_quat(dualquat)

    real = w
    dual = f_rotation_inverse(quat_aux, v)
    
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

    quat_aux = get_quat(dualquat)

    w_body = w
    v_inertial = f_rotation(quat_aux, dual)

    velocity = ca.vertcat(w_body, v_inertial)
    f_velocity = Function('f_velocity', [w_1d, dual_1d], [velocity])
    return f_velocity

def error_manifold(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    q_conjugate = ca.vertcat(q[0], -q[1], -q[2], -q[3], q[4], -q[5], -q[6], -q[7])
    quat_d_data = qd[0:4]
    dual_d_data =  qd[4:8]

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


    q_e_aux = Hplus @ q_conjugate
    
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

def error_manifold_norm(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    q_conjugate = ca.vertcat(q[0], -q[1], -q[2], -q[3], q[4], -q[5], -q[6], -q[7])
    quat_d_data = qd[0:4]
    dual_d_data =  qd[4:8]

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


    q_e_aux = Hplus @ q_conjugate
    
    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux
    expr2 = -q_e_aux

    q_error = ca.if_else(condition1, expr1, expr2) 
    # Check shortest path

    Q3_pose =  ca.DM.zeros(8, 1)
    Q3_pose[0, 0] = 1.0
    
    q_e_ln = Q3_pose - q_error
    q_real = q_e_ln[0:4]
    q_dual = q_e_ln[4:8]

    norm_q_real = ca.norm_2(q_real)
    norm_q_dual = ca.norm_2(q_dual)

    condition1_norm = norm_q_real[0, 0] > 0.0

    # Define expressions for each condition
    expr1_norm =  norm_q_real + ca.fabs(ca.dot(q_real[0:4, 0], q_dual[0:4, 0]))/(2*norm_q_real)
    expr2_norm = norm_q_dual

    norm = ca.if_else(condition1_norm, expr1_norm, expr2_norm) 
    return norm

def error_lie_norm():
    qd = ca.MX.sym('qd', 8, 1)
    q = ca.MX.sym('q', 8, 1)
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
    
    #condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    #expr1 =  q_e_aux
    #expr2 = -q_e_aux

    #q_error = ca.if_else(condition1, expr1, expr2) 
    q_error = q_e_aux
    # Check shortest path

    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error_real[0])

    ## Dual Part
    H_error_dual_plus = ca.vertcat(ca.horzcat(q_error_dual[0, 0], -q_error_dual[1, 0], -q_error_dual[2, 0], -q_error_dual[3, 0]),
                                ca.horzcat(q_error_dual[1, 0], q_error_dual[0, 0], -q_error_dual[3, 0], q_error_dual[2, 0]),
                                ca.horzcat(q_error_dual[2, 0], q_error_dual[3, 0], q_error_dual[0, 0], -q_error_dual[1, 0]),
                                ca.horzcat(q_error_dual[3, 0], -q_error_dual[2, 0], q_error_dual[1, 0], q_error_dual[0, 0]))

    trans_error = 2 * H_error_dual_plus@q_error_real_c
    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    #norm_ln_quaternion = ca.norm_2(ln_quaternion+ ca.np.finfo(np.float64).eps)
    #norm_ln_trans = ca.norm_2(ln_trans+ ca.np.finfo(np.float64).eps)

    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)
    norm_ln_quaternion = ca.norm_2(ln_quaternion)
    norm_ln_trans = ca.norm_2(ln_trans)
    norm = ca.norm_2(q_e_ln)
    f_norm = Function('f_norm', [qd, q], [norm])
    return f_norm

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
    
    #condition1 = q_e_aux[0, 0] > 0.0

    ## Define expressions for each condition
    #expr1 =  q_e_aux
    #expr2 = -q_e_aux

    #q_error = ca.if_else(condition1, expr1, expr2) 
    q_error = q_e_aux
    # Check shortest path

    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error_real[0])

    ## Dual Part
    H_error_dual_plus = ca.vertcat(ca.horzcat(q_error_dual[0, 0], -q_error_dual[1, 0], -q_error_dual[2, 0], -q_error_dual[3, 0]),
                                ca.horzcat(q_error_dual[1, 0], q_error_dual[0, 0], -q_error_dual[3, 0], q_error_dual[2, 0]),
                                ca.horzcat(q_error_dual[2, 0], q_error_dual[3, 0], q_error_dual[0, 0], -q_error_dual[1, 0]),
                                ca.horzcat(q_error_dual[3, 0], -q_error_dual[2, 0], q_error_dual[1, 0], q_error_dual[0, 0]))

    trans_error = 2 * H_error_dual_plus@q_error_real_c
    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    #norm_ln_quaternion = ca.norm_2(ln_quaternion+ ca.np.finfo(np.float64).eps)
    #norm_ln_trans = ca.norm_2(ln_trans+ ca.np.finfo(np.float64).eps)

    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)
    return q_e_ln

def error_lie_tan(qd, q):

    q_d_real = qd[0:4, 0]
    q_d_real_c = ca.vertcat(q_d_real[0, 0], -q_d_real[1, 0], -q_d_real[2, 0], -q_d_real[3, 0])
    q_d_dual = qd[4:8, 0]

    ## Real Part
    norm_d = ca.norm_2(q_d_real[1:4] + ca.np.finfo(np.float64).eps)
    angle_d = ca.atan2(norm_d, q_d_real[0])

    ## Dual Part
    H_d_dual_plus = ca.vertcat(ca.horzcat(q_d_dual[0, 0], -q_d_dual[1, 0], -q_d_dual[2, 0], -q_d_dual[3, 0]),
                                ca.horzcat(q_d_dual[1, 0], q_d_dual[0, 0], -q_d_dual[3, 0], q_d_dual[2, 0]),
                                ca.horzcat(q_d_dual[2, 0], q_d_dual[3, 0], q_d_dual[0, 0], -q_d_dual[1, 0]),
                                ca.horzcat(q_d_dual[3, 0], -q_d_dual[2, 0], q_d_dual[1, 0], q_d_dual[0, 0]))

    trans_d = 2 * H_d_dual_plus@q_d_real_c
    # Computing log map
    ln_quaternion_d = ca.vertcat(0.0,  (1/2)*angle_d*q_d_real[1, 0]/norm_d, (1/2)*angle_d*q_d_real[2, 0]/norm_d, (1/2)*angle_d*q_d_real[3, 0]/norm_d)
    ln_trans_d = ca.vertcat(0.0, (1/2)*trans_d[1, 0], (1/2)*trans_d[2, 0], (1/2)*trans_d[3, 0])
    q_d_ln = ca.vertcat(ln_quaternion_d, ln_trans_d)

    # Real Dual quaternion
    q_real = q[0:4, 0]
    q_real_c = ca.vertcat(q_real[0, 0], -q_real[1, 0], -q_real[2, 0], -q_real[3, 0])
    q_dual = q[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_real[0])

    ## Dual Part
    H_dual_plus = ca.vertcat(ca.horzcat(q_dual[0, 0], -q_dual[1, 0], -q_dual[2, 0], -q_dual[3, 0]),
                                ca.horzcat(q_dual[1, 0], q_dual[0, 0], -q_dual[3, 0], q_dual[2, 0]),
                                ca.horzcat(q_dual[2, 0], q_dual[3, 0], q_dual[0, 0], -q_dual[1, 0]),
                                ca.horzcat(q_dual[3, 0], -q_dual[2, 0], q_dual[1, 0], q_dual[0, 0]))

    trans = 2 * H_dual_plus@q_real_c
    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_real[1, 0]/norm, (1/2)*angle*q_real[2, 0]/norm, (1/2)*angle*q_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans[1, 0], (1/2)*trans[2, 0], (1/2)*trans[3, 0])
    q_ln = ca.vertcat(ln_quaternion, ln_trans)

    qe_ln = q_ln - q_d_ln

    return qe_ln

def error_dual_aux_casadi():
    qd = ca.MX.sym('qd', 8, 1)
    q = ca.MX.sym('q', 8, 1)
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
    
    q_error = q_e_aux
    # Check shortest path

    f_error_dual = Function('f_error_dual', [qd, q], [q_error])
    return f_error_dual

def error_lie_2(qd, q):
    q_conjugate = ca.vertcat(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7])
    quat_data = q_conjugate[0:4]
    dual_data =  q_conjugate[4:8]

    H_r_plus = ca.vertcat(ca.horzcat(quat_data[0], -quat_data[1], -quat_data[2], -quat_data[3]),
                                ca.horzcat(quat_data[1], quat_data[0], -quat_data[3], quat_data[2]),
                                ca.horzcat(quat_data[2], quat_data[3], quat_data[0], -quat_data[1]),
                                ca.horzcat(quat_data[3], -quat_data[2], quat_data[1], quat_data[0]))

    H_d_plus = ca.vertcat(ca.horzcat(dual_data[0], -dual_data[1], -dual_data[2], -dual_data[3]),
                                ca.horzcat(dual_data[1], dual_data[0], -dual_data[3], dual_data[2]),
                                ca.horzcat(dual_data[2], dual_data[3], dual_data[0], -dual_data[1]),
                                ca.horzcat(dual_data[3], -dual_data[2], dual_data[1], dual_data[0]))
    zeros = ca.DM.zeros(4, 4)
    Hplus = ca.vertcat(ca.horzcat(H_r_plus, zeros),
                        ca.horzcat(H_d_plus, H_r_plus))

    q_d_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    q_e_aux = Hplus @ q_d_conjugate
    
    #condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    #expr1 =  q_e_aux
    #expr2 = -q_e_aux

    #q_error = ca.if_else(condition1, expr1, expr2) 
    q_error = q_e_aux
    # Check shortest path

    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error_real[0])

    ## Dual Part
    H_error_dual_plus = ca.vertcat(ca.horzcat(q_error_dual[0, 0], -q_error_dual[1, 0], -q_error_dual[2, 0], -q_error_dual[3, 0]),
                                ca.horzcat(q_error_dual[1, 0], q_error_dual[0, 0], -q_error_dual[3, 0], q_error_dual[2, 0]),
                                ca.horzcat(q_error_dual[2, 0], q_error_dual[3, 0], q_error_dual[0, 0], -q_error_dual[1, 0]),
                                ca.horzcat(q_error_dual[3, 0], -q_error_dual[2, 0], q_error_dual[1, 0], q_error_dual[0, 0]))

    trans_error = 2 * H_error_dual_plus@q_error_real_c
    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    #norm_ln_quaternion = ca.norm_2(ln_quaternion+ ca.np.finfo(np.float64).eps)
    #norm_ln_trans = ca.norm_2(ln_trans+ ca.np.finfo(np.float64).eps)

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

    # Sux variable in roder to get a norm
    q_3_aux = ca.DM([1.0, 0.0, 0.0, 0.0])
    Q3_pose =  ca.vertcat(q_3_aux)
    
    q_e_ln = Q3_pose - q_error

    return q_e_ln
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
    w = omega[0:3, 0]
    v = omega[3:6, 0]
    p = get_trans(dual)
    q = get_quat(dual)
    p = p[1:4, 0]

    # Compute unforced part
    a = ca.cross(-J_1@w, J@w)
    F_r = a
    F_d = ca.cross(v, w) - g*(f_rotation_inverse(q, e3))

    # Compute forced part
    U_r = J_1@torques
    U_d = (force/m)@e3

    T_r = F_r + U_r
    T_d = F_d + U_d
    T = ca.vertcat(T_r, T_d)

    return T

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
    dot_real_dual = 2* ca.dot(X[0:4], X[4:8])
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = ca.vertcat(norm_q, dot_real_dual)
    constraint.min = 1.0
    constraint.max = 1.0
    constraint.min2 = 0.0
    constraint.max2 = 0.0
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
    return model, get_trans, get_quat, constraint, error_manifold, error_lie, error_quaternion, error_lie_norm, error_manifold_norm

def noise(x, noise):
    # Get position and quaternion
    dual = x[0:8]
    twist = x[8:14]
    trans = get_trans(dual)
    trans_np = np.array(trans[1:4]).reshape((3, ))
    quat_data = get_quat(dual)

    # Split noise
    noise_position = noise[0:3]

    # Translation part
    trans_noise = trans_np + noise_position
    trans_noise_aux = np.array([0.0, trans_noise[0], trans_noise[1], trans_noise[2]])

    # Rotational part
    noise_quat = noise[3:6]
    squared_norm_delta = noise_quat[0]*noise_quat[0] + noise_quat[1]*noise_quat[1] + noise_quat[2]*noise_quat[2]
    q_delta = np.zeros((4, 1))

    if squared_norm_delta > 0:
        norm_delta = np.sqrt(squared_norm_delta)
        sin_delta_by_delta = np.sin(norm_delta) / norm_delta
        q_delta[0, 0] = np.cos(norm_delta)
        q_delta[1, 0] = sin_delta_by_delta * noise_quat[0]
        q_delta[2, 0] = sin_delta_by_delta * noise_quat[1]
        q_delta[3, 0] = sin_delta_by_delta * noise_quat[2]
    else:
        q_delta[0, 0] = 1.0
        q_delta[1, 0] = 0.0
        q_delta[2, 0] = 0.0
        q_delta[3, 0] = 0.0

    H_r_plus = ca.vertcat(ca.horzcat(quat_data[0, 0], -quat_data[1, 0], -quat_data[2, 0], -quat_data[3, 0]),
                                ca.horzcat(quat_data[1, 0], quat_data[0, 0], -quat_data[3, 0], quat_data[2, 0]),
                                ca.horzcat(quat_data[2, 0], quat_data[3, 0], quat_data[0, 0], -quat_data[1, 0]),
                                ca.horzcat(quat_data[3, 0], -quat_data[2, 0], quat_data[1, 0], quat_data[0, 0]))
    H_r_plus = np.array(H_r_plus)
    quat_noise_aux = H_r_plus@q_delta
    Q1_pose =  DualQuaternion.from_pose(quat = quat_noise_aux, trans = trans_noise_aux)
    values_pose = np.array(Q1_pose.get[:, 0]).reshape((8, ))

    # Twist Noise
    values_twist_aux = noise[6:12]
    values_twist = np.array(twist).reshape((6, )) + values_twist_aux
    x_noise = np.array([values_pose[0], values_pose[1], values_pose[2], values_pose[3],
                        values_pose[4], values_pose[5], values_pose[6], values_pose[7],
                        values_twist[0], values_twist[1], values_twist[2],
                        values_twist[3], values_twist[4], values_twist[5]])
    return x_noise
def cost_quaternion_casadi():
    qd = ca.MX.sym('qd', 4, 1)
    q = ca.MX.sym('q', 4, 1)
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    quat_d_data = qd_conjugate[0:4, 0]
    quaternion = q[0:4, 0]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0, 0], -quat_d_data[1, 0], -quat_d_data[2, 0], -quat_d_data[3, 0]),
                                ca.horzcat(quat_d_data[1, 0], quat_d_data[0, 0], -quat_d_data[3, 0], quat_d_data[2, 0]),
                                ca.horzcat(quat_d_data[2, 0], quat_d_data[3, 0], quat_d_data[0, 0], -quat_d_data[1, 0]),
                                ca.horzcat(quat_d_data[3, 0], -quat_d_data[2, 0], quat_d_data[1, 0], quat_d_data[0, 0]))



    q_e_aux = H_r_plus @ quaternion
    

    # Check shortest path
    #condition1 = q_e_aux[0, 0] > 0.0

    ## Define expressions for each condition
    #expr1 =  q_e_aux
    #expr2 = -q_e_aux

    #q_error = ca.if_else(condition1, expr1, expr2) 
    q_error = q_e_aux

    norm = ca.norm_2(q_error[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error[0])

    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error[1, 0]/norm, (1/2)*angle*q_error[2, 0]/norm, (1/2)*angle*q_error[3, 0]/norm)

    cost = ln_quaternion.T@ln_quaternion

    # Sux variable in roder to get a norm
    f_cost = Function('f_cost', [qd, q], [cost])
    return f_cost

def cost_translation_casadi():
    td = ca.MX.sym('td', 4, 1)
    t = ca.MX.sym('t', 4, 1)

    te = td - t
    
    cost = te.T@te
    f_cost = Function('f_cost', [td, t], [cost])
    return f_cost

def ref_trajectory_agresive(t, ts, V_max, a_max, n):
        # Compute the desired Trajecotry of the system
        # INPUT 
        # t                                                - time
        # OUTPUT
        # xd, yd, zd                                       - desired position
        # theta                                            - desired orientation
        # theta_p                                          - desired angular velocity
        t = t
        r_max = (V_max**2)/a_max
        k = a_max/V_max
        r_min = (r_max)/n

        # Compute desired reference x y z
        xd = r_max * np.sin(k * t)
        yd = r_min * np.cos(k * t)
        zd = 5 * np.ones((t.shape[0], ))

        # Compute velocities
        xd_p = r_max * k * np.cos(k * t)
        yd_p = -r_min * k * np.sin(k * t)
        zd_p = 0 * np.ones((t.shape[0], ))

        # Compute acceleration
        xd_pp = - r_max * k * k * np.sin(k * t)
        yd_pp = - r_min * k * k * np.cos(k * t)
        zd_pp = 0 * np.ones((t.shape[0], ))

        # Compute jerk
        xd_ppp = - r_max * k * k * k * np.cos(k * t)
        yd_ppp =  r_min * k * k * k * np.sin(k * t) 
        zd_ppp = 0 * np.ones((t.shape[0], ))

        # Compute snap
        xd_pppp =  r_max * k * k * k * k * np.sin(k * t)
        yd_pppp =  r_min * k * k * k * k * np.cos(k * t) 
        zd_pppp = 0 * np.ones((t.shape[0], ))

        # Compute angular displacement
        theta = np.arctan2(yd_p, xd_p)
        theta = theta

        # Compute angular velocity
        theta_p = (1. / ((yd_p / xd_p) ** 2 + 1)) * ((yd_pp * xd_p - yd_p * xd_pp) / xd_p ** 2)
        theta_p[0] = 0.0

        theta_pp = np.zeros((theta.shape[0]))
        theta_pp[0] = 0.0

        # Compute the angular acceleration
        for k in range(1, theta_p.shape[0]):
            theta_pp[k] = (theta_p[k] - theta_p[k-1])/ts
        hd = np.vstack((xd, yd, zd))
        hd_p = np.vstack((xd_p, yd_p, zd_p))
        hd_pp = np.vstack((xd_pp, yd_pp, zd_pp))
        hd_ppp = np.vstack((xd_ppp, yd_ppp, zd_ppp))
        hd_pppp = np.vstack((xd_pppp, yd_pppp, zd_pppp))

        return hd, theta, hd_p, theta_p, hd_pp, hd_ppp, hd_pppp, theta_pp