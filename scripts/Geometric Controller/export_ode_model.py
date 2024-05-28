
from acados_template import AcadosModel
from casadi import Function, MX, vertcat, sin, cos
import numpy as np
import casadi as ca

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
    omega_dot = ca.inv(J)@(u[1:3] - aux_cross)

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
    return model, f_system, constraint, error_quaternion

def error_quaternion(qd, q):
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0])
    quat_d_data = qd_conjugate[0:4, 0]
    quaternion = q[0:4, 0]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0, 0], -quat_d_data[1, 0], -quat_d_data[2, 0], -quat_d_data[3, 0]),
                                ca.horzcat(quat_d_data[1, 0], quat_d_data[0, 0], -quat_d_data[3, 0], quat_d_data[2, 0]),
                                ca.horzcat(quat_d_data[2, 0], quat_d_data[3, 0], quat_d_data[0, 0], -quat_d_data[1, 0]),
                                ca.horzcat(quat_d_data[3, 0], -quat_d_data[2, 0], quat_d_data[1, 0], quat_d_data[0, 0]))



    q_e_aux = H_r_plus @ quaternion

    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux
    expr2 = -q_e_aux

    q_error = ca.if_else(condition1, expr1, expr2) 

    norm = ca.norm_2(q_error[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error[0])

    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error[1, 0]/norm, (1/2)*angle*q_error[2, 0]/norm, (1/2)*angle*q_error[3, 0]/norm)

    return ln_quaternion
