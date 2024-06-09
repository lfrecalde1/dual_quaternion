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
    

    X = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d)

    # Split States of the system
    twist_1 = X[8:14, 0]
    u = twist_1
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

    X_dot = ca.vertcat(qw_1dot, qx_1dot, qy_1dot, qz_1dot, dw_1dot, dx_1dot, dy_1dot, dz_1dot)


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

    # Algebraic variables
    z = []
    p = []

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
    return model, get_trans, get_quat, constraint