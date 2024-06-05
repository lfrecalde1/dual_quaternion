from acados_template import AcadosModel
from casadi import Function, MX, vertcat, sin, cos
import numpy as np
import casadi as ca

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
f_rotation_inverse = rotation_inverse_casadi()
def quatTorot_c(quat):
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # R                                                          - rotational matrix

    # Normalized quaternion
    q = quat
    #q = q/(q.T@q)

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
    m = L[0]
    Jxx = L[1]
    Jyy = L[2]
    Jzz = L[3]
    gravity = L[4]

    # States of the system
    x = MX.sym('x')
    y = MX.sym('y')
    z = MX.sym('z')

    vx = MX.sym('vx')
    vy = MX.sym('vy')
    vz = MX.sym('vz')

    qw = MX.sym('qw')
    q1 = MX.sym('q1')
    q2 = MX.sym('q2')
    q3 = MX.sym('q3')

    wx = MX.sym('wx')
    wy = MX.sym('wy')
    wz = MX.sym('wz')

    X = vertcat(x, y, z, vx, vy, vz, qw, q1, q2, q3, wx, wy, wz)

    # Auxiliary variables implicit function
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    z_dot = MX.sym('z_dot')

    vx_dot = MX.sym('vx_dot')
    vy_dot = MX.sym('vy_dot')
    vz_dot = MX.sym('vz_dot')

    qw_dot = MX.sym('qw_dot')
    q1_dot = MX.sym('q1_dot')
    q2_dot = MX.sym('q2_dot')
    q3_dot = MX.sym('q3_dot')

    wx_dot = MX.sym('wx_dot')
    wy_dot = MX.sym('wy_dot')
    wz_dot = MX.sym('wz_dot')

    X_dot = vertcat(x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot, qw_dot, q1_dot, q2_dot, q3_dot, wx_dot, wy_dot, wz_dot)

    # Control actions
    F_ref = MX.sym('F_ref')
    tau_1_ref = MX.sym('tau_1_ref')
    tau_2_ref = MX.sym('tau_2_ref')
    tau_3_ref = MX.sym('tau_3_ref')

    u = vertcat(F_ref, tau_1_ref, tau_2_ref, tau_3_ref)
    M = vertcat(tau_1_ref, tau_2_ref, tau_3_ref)
    F = vertcat(F_ref)

    # Inertial Matrix
    J = MX.zeros(3, 3)
    J[0, 0] = Jxx
    J[1, 1] = Jyy
    J[2, 2] = Jzz

    #Auxiliar variable
    e3 = MX.zeros(3, 1)
    e3[2, 0] = 1.0
    g = gravity*e3

    # Split values
    vel = X[3:6, 0]
    quat = X[6:10, 0]
    omega = X[10:13, 0]
    R = quatTorot_c(quat)

    # Rate of change of the system
    acc = ((u[0]*(R@e3))/m) - g

    qdot = quatdot_c(quat, omega)
    aux = J@omega
    aux_cross = ca.cross(omega, aux)
    omega_dot = ca.inv(J)@(u[1:4] - aux_cross)

    # Desired Trajectory
    x_d = MX.sym('x_d')
    y_d = MX.sym('y_d')
    z_d = MX.sym('z_d')

    vx_d = MX.sym('vx_d')
    vy_d = MX.sym('vy_d')
    vz_d = MX.sym('vz_d')

    qw_d = MX.sym('qw_d')
    q1_d = MX.sym('q1_d')
    q2_d = MX.sym('q2_d')
    q3_d = MX.sym('q3_d')

    wx_d = MX.sym('wx_d')
    wy_d = MX.sym('wy_d')
    wz_d = MX.sym('wz_d')

    F_d = MX.sym('F_d')
    T1_d = MX.sym('T1_d')
    T2_d = MX.sym('T2_d')
    T3_d = MX.sym('T3_d')


    X_d = vertcat(x_d, y_d, z_d, vx_d, vy_d, vz_d, qw_d, q1_d, q2_d, q3_d, wx_d, wy_d, wz_d, F_d, T1_d, T2_d, T3_d)
    p = X_d

    # Explicit and implicit functions
    f_expl = vertcat(vel, acc, qdot, omega_dot)
    f_system = Function('system',[X, u], [f_expl])
    f_impl = X_dot - f_expl

    # Constraints quaternions
    norm_q = ca.norm_2(X[6:10, 0])
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = vertcat(norm_q)
    constraint.min = 1.0
    constraint.max = 1.0

    # Compute short path to the desired quaternion
    q_error = ca.MX.sym('q_error', 4, 1)

    # Define the function
    f_error = ca.Function('f_error', [q_error], [ca.if_else(q_error[0] >= 0, q_error, -q_error)])

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
    return model, f_system, constraint, f_error, error_quaternion, f_rotation_inverse, error_quaternion_li, error_quaternion_z

def conjugate_quaternion(q):
    # Compute the conjugate of a specified quaternion
    # INPUT
    # q                                              - quaternion
    # OUTPUT 
    # q_c                                            - conjugate quaternion
    a1 = q[0]
    b1 = -q[1]
    c1 = -q[2]
    d1 = -q[3]
    q_c = vertcat(a1, b1, c1, d1)
    return q_c
def matrix_q(q):
    # Compute Q matrix for quaternion multiplication
    #INPUT
    # q                                                - quaternion
    #OUTPUT  
    # Q                                                - quaternion Matrix
    # Split variables
    a1 = q[0]
    b1 = q[1]
    c1 = q[2]
    d1 = q[3]

    Q = ca.vertcat(
        ca.horzcat(a1, -b1, -c1, -d1),
        ca.horzcat(b1, a1, -d1, c1),
        ca.horzcat(c1, d1, a1, -b1),
        ca.horzcat(d1, -c1, b1, a1))
    return Q

def quat_multiply(q1, q2):
    # Multiplication between quaternions
    # INPUT
    # q1                              - quaternion 1
    # q2                              - quaternion 2
    #OUTPUT
    # q1q2                            - q1 multiply q2
    Q = matrix_q(q1)
    q1q2 = Q@q2
    return q1q2

def quaternion_to_axis_angle(q):
    # Log quaternion
    # INPUT
    # q                              - quaternion 
    # OUTPUT 
    # theta                          - angle rotation
    # axis                           - axis where the rotation was executed
    # Normalize the quaternion
    q_norm = q

    # Extract the scalar and vector parts
    scalar_part = q_norm[0]
    vector_part = q_norm[1:4]


    # Compute the rotation angle (in radians)
    theta = ca.atan2(ca.norm_2(vector_part), scalar_part)
    theta_aux  = 0

    # Compute the rotation axis
    axis = vector_part / ca.norm_2(vector_part)
    axis_part_aux = vertcat(0, 0, 1)

    # Initialize the output
    #output = axis * theta

    # Check if theta is close to zero
    output = ca.if_else((theta*theta) < 2.2204e-1,
                        (theta_aux*axis_part_aux),
                        theta*axis)

    return output

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


    # Sux variable in roder to get a norm
    #q_3_aux = ca.DM([1.0, 0.0, 0.0, 0.0])
    #Q3_pose =  ca.vertcat(q_3_aux)
    
    #q_e_ln = Q3_pose - q_error

    return ln_quaternion

def error_quaternion_z(qd, q):
    quaternion = q[0:4, 0]
    quaternion_d = qd[0:4, 0]

    dot_product = quaternion_d.T@quaternion
    
    cost = (1 - ca.fabs(dot_product))
    return cost

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