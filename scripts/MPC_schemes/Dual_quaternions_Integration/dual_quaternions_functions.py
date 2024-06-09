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