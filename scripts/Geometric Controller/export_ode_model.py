
from acados_template import AcadosModel
from casadi import Function, MX, vertcat, sin, cos
import numpy as np
import casadi as ca
def quatTorot_c(quat):
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # Q                                                          - rotational matrix

    # Normalized quaternion
    q = quat
    #q = q/(q.T@q)


    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Q = ca.vertcat(
        ca.horzcat(q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)),
        ca.horzcat(2*(q1*q2+q0*q3), q0**2+q2**2-q1**2-q3**2, 2*(q2*q3-q0*q1)),
        ca.horzcat(2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2+q3**2-q1**2-q2**2))

    return Q
def rotation_matrix_error_c():
    # Desired Quaternion
    qd = ca.MX.sym('qd', 4, 1)

    # Current quaternion
    q = ca.MX.sym('q', 4, 1)

    Rd = quatTorot_c(qd)
    R = quatTorot_c(q)

    error_matrix = Rd.T@R
    f_error_matrix = Function('f_error_matrix', [qd, q], [error_matrix])

    return f_error_matrix

def Ad_c():
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
    f_ad =  ca.Function('f_ad', [quat_aux_1, vector_aux_1], [vector_i[1:4, 0]])
    return f_ad

def Ad_quat(q, v):
    # Function that enables the rotation of a vector using quaternions

    # Creation of the symbolic variables for the quaternion and the vector
    quat_aux_1 = q
    vector_aux_1 = v

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

    return vector_i[1:4, 0]

def quaternion_error_c():
    # Desired Quaternion
    qd = ca.MX.sym('qd', 4, 1)

    # Current quaternion
    q = ca.MX.sym('q', 4, 1)

    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    quat_d_data = qd_conjugate[0:4, 0]
    quaternion = q[0:4, 0]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0, 0], -quat_d_data[1, 0], -quat_d_data[2, 0], -quat_d_data[3, 0]),
                                ca.horzcat(quat_d_data[1, 0], quat_d_data[0, 0], -quat_d_data[3, 0], quat_d_data[2, 0]),
                                ca.horzcat(quat_d_data[2, 0], quat_d_data[3, 0], quat_d_data[0, 0], -quat_d_data[1, 0]),
                                ca.horzcat(quat_d_data[3, 0], -quat_d_data[2, 0], quat_d_data[1, 0], quat_d_data[0, 0]))

    q_e_aux = H_r_plus @ quaternion
    f_error_quat = Function('f_error_quat', [qd, q], [q_e_aux])
    return f_error_quat

def quaternion_conjugate_c():
    # Desired Quaternion
    qd = ca.MX.sym('qd', 4, 1)
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    f_conjugate_quat = Function('f_conjugate_quat', [qd], [qd_conjugate])
    return f_conjugate_quat

def quaternion_conjugate(qd):
    # Desired Quaternion
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    return qd_conjugate

def quaternion_error(qd, q):
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    quat_d_data = qd_conjugate[0:4, 0]
    quaternion = q[0:4, 0]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0, 0], -quat_d_data[1, 0], -quat_d_data[2, 0], -quat_d_data[3, 0]),
                                ca.horzcat(quat_d_data[1, 0], quat_d_data[0, 0], -quat_d_data[3, 0], quat_d_data[2, 0]),
                                ca.horzcat(quat_d_data[2, 0], quat_d_data[3, 0], quat_d_data[0, 0], -quat_d_data[1, 0]),
                                ca.horzcat(quat_d_data[3, 0], -quat_d_data[2, 0], quat_d_data[1, 0], quat_d_data[0, 0]))

    q_e_aux = H_r_plus @ quaternion
    return q_e_aux

def ln_quaternion_c():
    # Current quaternion
    q = ca.MX.sym('q', 4, 1)

    norm = ca.norm_2(q[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q[0])

    ln_quaternion = ca.vertcat((1/2)*angle*q[1, 0]/norm, (1/2)*angle*q[2, 0]/norm, (1/2)*angle*q[3, 0]/norm)

    f_ln_quat = Function('f_ln_matrix', [q], [ln_quaternion])
    return f_ln_quat

def ln_quaternion(q):
    # Current quaternion

    #norm = ca.norm_2(q[1:4] + ca.np.finfo(np.float64).eps)
    #angle = ca.atan2(norm, q[0])

    #ln_quaternion = ca.vertcat((1/2)*angle*q[1, 0]/norm, (1/2)*angle*q[2, 0]/norm, (1/2)*angle*q[3, 0]/norm)

    qv = q[1:4]
    qw = q[0]

    ln_taylor = 2 * (qv/qw)*(1 - (ca.norm_2(qv)/3*qw**2))
    #return ln_quaternion
    return ln_taylor
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
    # Split system parameters
    Jxx = L[0]
    Jyy = L[1]
    Jzz = L[2]

    qw = MX.sym('qw')
    q1 = MX.sym('q1')
    q2 = MX.sym('q2')
    q3 = MX.sym('q3')

    wx = MX.sym('wx')
    wy = MX.sym('wy')
    wz = MX.sym('wz')

    X = vertcat(qw, q1, q2, q3, wx, wy, wz)

    # Auxiliary variables implicit function
    qw_dot = MX.sym('qw_dot')
    q1_dot = MX.sym('q1_dot')
    q2_dot = MX.sym('q2_dot')
    q3_dot = MX.sym('q3_dot')

    wx_dot = MX.sym('wx_dot')
    wy_dot = MX.sym('wy_dot')
    wz_dot = MX.sym('wz_dot')

    X_dot = vertcat(qw_dot, q1_dot, q2_dot, q3_dot, wx_dot, wy_dot, wz_dot)

    # Control actions
    tau_1_ref = MX.sym('tau_1_ref')
    tau_2_ref = MX.sym('tau_2_ref')
    tau_3_ref = MX.sym('tau_3_ref')

    u = vertcat(tau_1_ref, tau_2_ref, tau_3_ref)

    # Inertial Matrix
    J = MX.zeros(3, 3)
    J[0, 0] = Jxx
    J[1, 1] = Jyy
    J[2, 2] = Jzz


    # Split values
    quat = X[0:4, 0]
    omega = X[4:7, 0]


    qdot = quatdot_c(quat, omega)
    aux = J@omega
    aux_cross = ca.cross(omega, aux)
    omega_dot = ca.inv(J)@(u - aux_cross)

    qw_d = MX.sym('qw_d')
    q1_d = MX.sym('q1_d')
    q2_d = MX.sym('q2_d')
    q3_d = MX.sym('q3_d')

    wx_d = MX.sym('wx_d')
    wy_d = MX.sym('wy_d')
    wz_d = MX.sym('wz_d')

    T1_d = MX.sym('T1_d')
    T2_d = MX.sym('T2_d')
    T3_d = MX.sym('T3_d')


    X_d = vertcat(qw_d, q1_d, q2_d, q3_d, wx_d, wy_d, wz_d, T1_d, T2_d, T3_d)
    p = X_d

    # Explicit and implicit functions
    f_expl = vertcat(qdot, omega_dot)
    f_system = Function('system',[X, u], [f_expl])
    f_impl = X_dot - f_expl

    # Constraints quaternions
    norm_q = ca.norm_2(X[0:4, 0])
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = vertcat(norm_q)
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
    return model, f_system, constraint, Ad_quat, quaternion_conjugate, quaternion_error, ln_quaternion
