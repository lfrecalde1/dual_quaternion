import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from casadi import Function
from casadi import jacobian
from acados_template import AcadosModel
from scipy.spatial.transform import Rotation as R
from functions import dualquat_from_pose_casadi

dualquat_from_pose = dualquat_from_pose_casadi()

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
dual_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1, dw_1, dx_1, dy_1, dz_1)

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

def quat_multi_casadi():
    # Function that enables the rotation of a vector using quaternions

    # Creation of the symbolic variables for the quaternion and the vector
    quat_aux_1 = ca.MX.sym('quat_aux_1', 4, 1)
    vector_aux_1 = ca.MX.sym('vector_aux_1', 4, 1)

    # Defining the pure quaternion based on the vector information
    vector = vector_aux_1

    # Compute conjugate of the quaternion
    quat = quat_aux_1

    H_plus_q = ca.vertcat(ca.horzcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0]),
                                ca.horzcat(quat[1, 0], quat[0, 0], -quat[3, 0], quat[2, 0]),
                                ca.horzcat(quat[2, 0], quat[3, 0], quat[0, 0], -quat[1, 0]),
                                ca.horzcat(quat[3, 0], -quat[2, 0], quat[1, 0], quat[0, 0]))

    # Computing the first multiplication
    aux_value = H_plus_q@vector

    f_multi =  ca.Function('f_multi', [quat_aux_1, vector_aux_1], [aux_value])
    return f_multi
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

dual_twist = dual_velocity_casadi()
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

def error_dual_split(qd, q):
    quat_d = get_quat(qd)
    quat = get_quat(q)

    t_d = get_trans(qd)
    t = get_trans(q)

    quat_d_c = ca.vertcat(quat_d[0, 0], -quat_d[1, 0], -quat_d[2, 0], -quat_d[3, 0])

    # QUaternion Multiplication vector form
    H_plus_q_d_c = ca.vertcat(ca.horzcat(quat_d_c[0, 0], -quat_d_c[1, 0], -quat_d_c[2, 0], -quat_d_c[3, 0]),
                                ca.horzcat(quat_d_c[1, 0], quat_d_c[0, 0], -quat_d_c[3, 0], quat_d_c[2, 0]),
                                ca.horzcat(quat_d_c[2, 0], quat_d_c[3, 0], quat_d_c[0, 0], -quat_d_c[1, 0]),
                                ca.horzcat(quat_d_c[3, 0], -quat_d_c[2, 0], quat_d_c[1, 0], quat_d_c[0, 0]))

    # First Multiplication
    qe = H_plus_q_d_c@quat
    te_aux = f_rotation_inverse(quat_d, t[1:4, 0]- t_d[1:4, 0])
    # te_aux = t[1:4] - t_d[1:4]

    dual_quaternion_error = dualquat_from_pose(qe[0, 0], qe[1, 0], qe[2, 0], qe[3, 0], te_aux[0, 0], te_aux[1, 0], te_aux[2, 0])
    return dual_quaternion_error

def error_dual(qd, q):
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

    q_error = Hplus @ q

    return q_error

def error_dual_right(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    quat_d_data = q[0:4]
    dual_d_data =  q[4:8]

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

    q_error = Hplus @ qd_conjugate

    return q_error

def ln_dual(q_error):
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
    ln_quaternion = ca.vertcat((1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat((1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)

    return q_e_ln

def quatdot_c(quat, omega):
    # Quaternion evolution guaranteeing norm 1 (Improve this section)
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    # Split values quaternion
    qw = quat[0, 0]
    qx = quat[1, 0]
    qy = quat[2, 0]
    qz = quat[3, 0]


    # Auxiliary variable in order to avoid numerical issues
    K_quat = 10
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    # Create skew matrix
    H_r_plus = ca.vertcat(ca.horzcat(quat[0, 0], -quat[1, 0], -quat[2, 0], -quat[3, 0]),
                                ca.horzcat(quat[1, 0], quat[0, 0], -quat[3, 0], quat[2, 0]),
                                ca.horzcat(quat[2, 0], quat[3, 0], quat[0, 0], -quat[1, 0]),
                                ca.horzcat(quat[3, 0], -quat[2, 0], quat[1, 0], quat[0, 0]))

    omega_quat = ca.vertcat(0.0, omega[0, 0], omega[1, 0], omega[2, 0])


    q_dot = (1/2)*(H_r_plus@omega_quat) + K_quat*quat_error*quat
    return q_dot

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

    X = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d)

    # Split States of the system
    u = ca.vertcat(wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)
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

    X_dot = ca.vertcat(qw_1dot, qx_1dot, qy_1dot, qz_1dot, dw_1dot, dx_1dot, dy_1dot, dz_1dot)

    # Dualquaternion dot
    
    twist_1 = dual_twist(u, dualquat_1)
    #twist_1 = u
    dualdot = quatdot_simple(dualquat_1, twist_1)


    norm_q = ca.norm_2(get_quat(X[0:8]))
    dot_real_dual = 2* ca.dot(X[0:4], X[4:8])
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = ca.vertcat(norm_q, dot_real_dual)
    constraint.min = 1.0
    constraint.max = 1.0
    constraint.min2 = 0.0
    constraint.max2 = 0.0
    # Explicit and implicit functions
    f_expl = ca.vertcat(dualdot)
    f_impl = X_dot - f_expl

    # Reference velocities
    p = ca.MX.sym('p', 14, 1)

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
    return model, get_trans, get_quat, constraint, error_dual, ln_dual

def quadrotorModelnominal(L: list)-> AcadosModel:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # model                                                      - Acados model
    model_name = 'quadrotor'
    constraint = ca.types.SimpleNamespace()
    # Split system parameters
    # States of the system
    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    z = ca.MX.sym('z')


    qw = ca.MX.sym('qw')
    q1 = ca.MX.sym('q1')
    q2 = ca.MX.sym('q2')
    q3 = ca.MX.sym('q3')


    X = ca.vertcat(x, y, z, qw, q1, q2, q3)

    # Auxiliary variables implicit function
    x_dot = ca.MX.sym('x_dot')
    y_dot = ca.MX.sym('y_dot')
    z_dot = ca.MX.sym('z_dot')


    qw_dot = ca.MX.sym('qw_dot')
    q1_dot = ca.MX.sym('q1_dot')
    q2_dot = ca.MX.sym('q2_dot')
    q3_dot = ca.MX.sym('q3_dot')


    X_dot = ca.vertcat(x_dot, y_dot, z_dot, qw_dot, q1_dot, q2_dot, q3_dot)


    # Defining the desired Velocity using symbolics
    vx_1d = ca.MX.sym("vx_1d", 1, 1)
    vy_1d = ca.MX.sym("vy_1d", 1, 1)
    vz_1d = ca.MX.sym("vz_1d", 1, 1)

    wx_1d = ca.MX.sym("wx_1d", 1, 1)
    wy_1d = ca.MX.sym("wy_1d", 1, 1)
    wz_1d = ca.MX.sym("wz_1d", 1, 1)

    # Split States of the system
    u = ca.vertcat(wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)

    # Split values
    quat = X[3:7, 0]
    omega = u[0:3]
    vel = u[3:6]
    qdot = quatdot_c(quat, omega)
    xdot = vel

    # Reference velocities
    p = ca.MX.sym('p', 13, 1)

    # Explicit and implicit functions
    f_expl = ca.vertcat(xdot, qdot)
    f_system = Function('system',[X, u], [f_expl])
    f_impl = X_dot - f_expl

    # Constraints quaternions
    norm_q = ca.norm_2(X[3:7, 0])
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = ca.vertcat(norm_q)
    constraint.min = 1.0
    constraint.max = 1.0


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
    return model, f_system, constraint, error_quaternion_li

def ref_trajectory_agresive(t, ts, mul):
        # Compute the desired Trajecotry of the system
        # INPUT 
        # t                                                - time
        # OUTPUT
        # xd, yd, zd                                       - desired position
        # theta                                            - desired orientation
        # theta_p                                          - desired angular velocity
        t = t
        Q = mul

        # Compute desired reference x y z
        xd = 4 * np.sin(mul * 0.04* t)
        yd = 4 * np.sin(mul * 0.08 * t)
        zd = 1 * np.sin(0.05*Q*t) + 1

        # Compute velocities
        # Compute velocities
        xd_p = 4 * mul * 0.04 * np.cos(mul * 0.04 * t)
        yd_p = 4 * mul * 0.08 * np.cos(mul * 0.08 * t)
        zd_p = 0.05 * Q * np.cos(0.05*Q * t)

        # Compute acceleration
        xd_pp = -4 * mul * mul * 0.04 * 0.04 * np.sin(mul * 0.04 * t)
        yd_pp = -4 * mul * mul * 0.08 * 0.08 * np.sin(mul * 0.08 * t);  
        zd_pp = -0.05 * 0.05 * Q * Q *  np.sin(0.05*Q * t)

        # Compute jerk
        xd_ppp = -4 * mul * mul * mul * 0.04 * 0.04 * 0.04 * np.cos(mul * 0.04 * t)
        yd_ppp = -4 * mul * mul * mul * 0.08 * 0.08 * 0.08 * np.cos(mul * 0.08 * t);  
        zd_ppp = -0.05 * 0.05 * 0.05* Q * Q * Q * np.cos(0.05*Q * t)

        # Compute snap
        xd_pppp = 4 * mul * mul * mul * mul * 0.04 * 0.04 * 0.04 * 0.04 * np.sin(mul * 0.04 * t)
        yd_pppp = 4 * mul * mul * mul * mul * 0.08 * 0.08 * 0.08 * 0.08 * np.sin(mul * 0.08 * t);  
        zd_pppp = 0.05 * 0.05 * 0.05 * 0.05 * Q * Q * Q * Q * np.sin(0.05*Q * t)

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

def compute_reference(t, ts, mul, L):
    # Drone Parameters
    m = L[0]
    Jxx = L[1]
    Jyy = L[2]
    Jzz = L[3]
    g = L[4]
    J = np.array([[Jxx, 0.0, 0.0], [0.0, Jyy, 0.0], [0.0, 0.0, Jzz]])

    # Inertial Frame 
    Zw = np.array([[0.0], [0.0], [1.0]])
    Xw = np.array([[1.0], [0.0], [0.0]])
    Yw = np.array([[0.0], [1.0], [0.0]])

    # Desired Flat outputs
    hd, theta, hd_p, theta_p, hd_pp, hd_ppp, hd_pppp, theta_pp = ref_trajectory_agresive(t, ts, mul)

    # Empty vector for the internal values

    alpha =  np.zeros((3, hd.shape[1]), dtype=np.double)
    beta =  np.zeros((3, hd.shape[1]), dtype=np.double)

    # Desired Orientation matrix
    Yc = np.zeros((3, hd.shape[1]), dtype=np.double)
    Xc = np.zeros((3, hd.shape[1]), dtype=np.double)
    Zc = np.zeros((3, hd.shape[1]), dtype=np.double)

    # Auxiliary Body frame
    Yb = np.zeros((3, hd.shape[1]), dtype=np.double)
    Xb = np.zeros((3, hd.shape[1]), dtype=np.double)
    Zb = np.zeros((3, hd.shape[1]), dtype=np.double)

    # Quaternions
    q = np.zeros((4, hd.shape[1]), dtype=np.double)

    f = np.zeros((1, hd.shape[1]), dtype=np.double)
    M = np.zeros((3, hd.shape[1]), dtype=np.double)

    # Angular vlocity
    w = np.zeros((3, hd.shape[1]), dtype=np.double)
    
    f_p = np.zeros((1, hd.shape[1]), dtype=np.double)

    # Angular acceleration
    w_p = np.zeros((3, hd.shape[1]), dtype=np.double)

    # Nominal torques of the system

    for k in range(0, hd.shape[1]):
        # Auxiliary variables
        alpha[:, k] = m*hd_pp[:, k] + m*g*Zw[:, 0]
        beta[:, k] = m*hd_pp[:, k] + m*g*Zw[:, 0]

        # Components Desired Orientation matrix
        Yc[:, k] = np.array([-np.sin(theta[k]), np.cos(theta[k]), 0])
        Xc[:, k] = np.array([ np.cos(theta[k]), np.sin(theta[k]), 0])
        Zc[:, k] = np.array([0.0, 0.0, 1.0])

        # Body frame that is projected to the desired orientation
        Xb[:, k] = (np.cross(Yc[:, k], alpha[:, k]))/(np.linalg.norm(np.cross(Yc[:, k], alpha[:, k])))
        Yb[:, k] = (np.cross(beta[:, k], Xb[:, k]))/(np.linalg.norm(np.cross(beta[:, k], Xb[:, k])))
        Zb[:, k] = np.cross(Xb[:, k], Yb[:, k])

        R_d = np.array([[Xb[0, k], Yb[0, k], Zb[0, k]], [Xb[1, k], Yb[1, k], Zb[1, k]], [Xb[2, k], Yb[2, k], Zb[2, k]]])
        r_d = R.from_matrix(R_d)
        quad_d_aux = r_d.as_quat()
        q[:, k] = np.array([quad_d_aux[3], quad_d_aux[0], quad_d_aux[1], quad_d_aux[2]])
        if k > 0:
            aux_dot = np.dot(q[:, k], q[:, k-1])
            if aux_dot < 0:
                q[:, k] = -q[:, k]
            else:
                q[:, k] = q[:, k]
        else:
            None
        # Compute nominal force of the in the body frame
        f[:, k] = np.dot(Zb[:, k], m*hd_pp[:, k] + m*g*Zw[:, 0])

        
        # Compute angular velocities
        # Elements of the vecto b
        b1 = m*np.dot(Xb[:, k], hd_ppp[:, k])
        b2 = -m*np.dot(Yb[:, k], hd_ppp[:, k])
        b3 = theta_p[k] * np.dot(Xc[:, k], Xb[:, k])

        b = np.array([[b1], [b2], [b3]], dtype=np.double)

        # Elements of the matrix A
        a11 = 0.0
        a12 = f[:, k]
        a13 = 0.0

        a21 = f[:, k]
        a22 = 0.0
        a23 = 0.0
        
        a31 = 0.0
        a32 = -np.dot(Yc[:, k], Zb[:, k])
        a33 = np.linalg.norm(np.cross(Yc[:, k], Zb[:, k]))

        # Inverse Matrix A
        A = np.array([[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]], dtype=np.double)
        A_1 = np.linalg.inv(A)

        # Compute nominal angular velocity
        aux_angular_velocity = A_1@b
        w[:, k] = aux_angular_velocity[:, 0]
        wx = w[0, k]
        wy = w[1, k]
        wz = w[2, k]

        # Time derivative of the force respect with the body axis
        f_p[:, k] = m*np.dot(Zb[:, k], hd_ppp[:, k])

        chi_1 = theta_pp[k] * np.dot(Xc[:, k], Xb[:, k])
        chi_2 = -2*theta_p[k] * wy * np.dot(Xc[:, k], Zb[:, k])
        chi_3 =  -wy * wx * np.dot(Yc[:, k], Yb[:, k])
        chi_4 =  2* theta_p[k] * wz * np.dot(Xc[:, k], Yb[:, k])
        chi_5 =  -wz*wx*np.dot(Yc[:, k], Zb[:, k])

        chi = chi_1 + chi_2 + chi_3 + chi_4 + chi_5

        # Compute angular accelerations of the system
        B1 = m*np.dot(Xb[:, k], hd_pppp[:, k]) - f[:, k]*wx*wz - 2*f_p[:, k]*wy
        B2 = -m*np.dot(Yb[:, k], hd_pppp[:, k]) -2 * f_p[:, k] * wx + f[:, k]*wy*wz
        B3 = chi

        B = np.array([[B1], [B2], [B3]], dtype=np.double)

        # Computing angular acceleration
        aux_angular_acce = A_1@B
        w_p[:, k] = aux_angular_acce[:, 0]
        aux_torque = J@w_p[:, k] + np.cross(w[:, k], J@w[:, k])
        # Compute torque
        M[:, k] = aux_torque


        # Updates nominal states from the differential flatness properties
    M_d = M            
    f_d = f            

    w_d = w
    w_d_d =w_p 

    h_d = hd
    h_d_d = hd_p
    h_d_dd = hd_pp
    h_d_ddd = hd_ppp
    h_d_dddd = hd_pppp

    psi_d = theta
    psi_d_d = theta_p
    psi_d_dd = theta_pp

    q_d = q

    return h_d, h_d_d, q_d, w_d, f_d, M_d

def error_quaternion_li(qk, qd, weight):
    q_aux = ca.vertcat(
     qd[0] * qk[0] + qd[1] * qk[1] + qd[2] * qk[2] + qd[3] * qk[3],
    -qd[1] * qk[0] + qd[0] * qk[1] + qd[3] * qk[2] - qd[2] * qk[3],
    -qd[2] * qk[0] - qd[3] * qk[1] + qd[0] * qk[2] + qd[1] * qk[3],
    -qd[3] * qk[0] + qd[2] * qk[1] - qd[1] * qk[2] + qd[0] * qk[3])

    q_att_denom = ca.sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)

    q_att = ca.vertcat(
    q_aux[0] * q_aux[1] - q_aux[2] * q_aux[3],
    q_aux[0] * q_aux[2] + q_aux[1] * q_aux[3],
    q_aux[3]) / q_att_denom

    return ca.transpose(q_att) @ weight @ q_att