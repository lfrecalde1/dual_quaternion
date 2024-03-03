import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion_body
from casadi import Function
from casadi import jacobian

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
    #quat = quat + (k1)*ts
    f_rk4 = Function('f_rk4', [quat_data, omega_data, ts_data], [quat.get[:, 0]])
    return f_rk4
    
def quatdot_simple(quat, omega):
    quat_data = quat[0:4]
    dual_data =  quat[4:8]

    # Auxiliary variable in order to avoid numerical issues
    norm_r = ca.dot(quat_data, quat_data)
    K_quat = 2
    norm_d = 2*(quat_data.T@dual_data)/1

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
    q_dot = (1/2)*(Hplus@ omega)
    #q_dot = (1/2)*(quat * omega)
    return q_dot

def f_rk4_casadi_simple(quat= dual_1d, omega=w_1d, ts=ts, quat_data=dual_1d_data, omega_data=w_1d_data, ts_data = ts):
    # Function that computes 
    k1 = quatdot_simple(quat, omega)
    k2 = quatdot_simple(quat + (1/2)*k1*ts, omega)
    k3 = quatdot_simple(quat + (1/2)*k2*ts, omega)
    k4 = quatdot_simple(quat + (1)*k3*ts, omega)
    # Compute forward Euler method
    quat = quat + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*ts
    #quat = quat + (k1)*ts
    f_rk4 = Function('f_rk4', [quat_data, omega_data, ts_data], [quat])
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

    q_error = f_error(q_e_aux.get[:, 0])

    q_e = DualQuaternion_body(q_real=Quaternion(q = q_error[0:4, 0]), q_dual= Quaternion(q = q_error[4:8, 0]))

    # Apply log mapping
    q_e_ln = q_e.ln_control()

    # Conjugate
    q_e_c = q_e.conjugate()

    # Control Law 
    U = -2*q_e_ln.vector_dot_product(kp) + q_e_c * wd * q_e
    aux = U.get[:, 0]
    control_values = ca.vertcat(aux[0, 0], aux[1, 0], aux[2, 0], aux[3, 0], aux[4, 0], aux[5, 0], aux[6, 0], aux[7, 0])

    f_control = Function('f_control', [qd_data, wd_data, q_data, kp_data], [control_values])
    return f_control


def lyapunov_casadi_simple(qd = dual_1d, q =dual_1, qd_data = dual_1d_data, q_data = dual_1_data):
    #  Control Error Split Values
    qd_conjugate = ca.vertcat(qd[0, 0], -qd[1, 0], -qd[2, 0], -qd[3, 0], qd[4, 0], -qd[5, 0], -qd[6, 0], -qd[7, 0])
    quat_data = qd_conjugate[0:4]
    dual_data =  qd_conjugate[4:8]

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


    q_e_aux = Hplus @ q
    
    condition1 = q_e_aux[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux[:, 0]
    expr2 = -q_e_aux[:, 0]

    # Check shortest path
    q_error = ca.if_else(condition1, expr1, expr2) 
    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    # Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = ca.atan2(norm, q_error_real[0])

    # Dual Part
    H_error_real_plus = ca.vertcat(ca.horzcat(q_error_real_c[0, 0], -q_error_real_c[1, 0], -q_error_real_c[2, 0], -q_error_real_c[3, 0]),
                                ca.horzcat(q_error_real_c[1, 0], q_error_real_c[0, 0], -q_error_real_c[3, 0], q_error_real_c[2, 0]),
                                ca.horzcat(q_error_real_c[2, 0], q_error_real_c[3, 0], q_error_real_c[0, 0], -q_error_real_c[1, 0]),
                                ca.horzcat(q_error_real_c[3, 0], -q_error_real_c[2, 0], q_error_real_c[1, 0], q_error_real_c[0, 0]))

    trans_error = 2 * H_error_real_plus@q_error_dual

    # Computing log map
    ln_quaternion = ca.vertcat(0.0,  (1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)

    P =  1*ca.MX.eye(8)
    P[0, 0] = 2
    P[1, 1] = 2
    P[2, 2] = 2
    P[3, 3] = 2

    norm_lie = q_e_ln.T@P@q_e_ln
    
    v = norm_lie
    v_f = Function('v_f', [qd_data, q_data], [v])
    return v_f

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
    P[0, 0] = 3
    P[1, 1] = 3
    P[2, 2] = 3
    P[3, 3] = 3

    q_e_ln_data = q_e_ln.get[:, 0]

    norm_lie = q_e_ln_data.T@P@q_e_ln_data
    
    v = norm_lie
    v_f = Function('v_f', [qd_data, q_data], [v])
    return v_f

def cost_casadi(qd = Q1d, q =Q1, qd_data = dual_1d_data, q_data = dual_1_data):
    #  Control Error Split Values
    q_e_aux = qd.conjugate() * q
    
    condition1 = q_e_aux.get[0, 0] > 0.0

    # Define expressions for each condition
    expr1 =  q_e_aux.get[:, 0]
    expr2 = -q_e_aux.get[:, 0]

    # Nested if_else to implement multiple branches
    q_error = ca.if_else(condition1, expr1, expr2) 

    q_e = DualQuaternion_body(q_real=Quaternion(q = q_error[0:4, 0]), q_dual= Quaternion(q = q_error[4:8, 0]))

    # Sux variable in roder to get a norm
    q_3_aux = ca.MX([1.0, 0.0, 0.0, 0.0])
    t_3_aux = ca.MX([0.0, 0.0, 0.0, 0.0])
    Q3_pose =  DualQuaternion_body.from_pose(quat = q_3_aux, trans = t_3_aux)
    
    q_e_ln = Q3_pose - q_e

    P =  1*ca.MX.eye(8)

    q_e_ln_data = q_e_ln.get[:, 0]

    norm_lie = q_e_ln_data.T@P@q_e_ln_data
    
    v = norm_lie

    v_f = Function('v_f', [qd_data, q_data], [v])
    return v_f

def lyapunov_dot_casadi(qd = Q1d, q =Q1, qd_data = dual_1d_data, q_data = dual_1_data):
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

    v_jacobian = jacobian(norm_lie, q.get[:, 0])
    
    v_p_f = Function('v_p_f', [qd_data, q_data], [v_jacobian])
    return v_p_f

def jacobian_casadi(qd = Q1d, qd_data = dual_1d_data):
    H = qd.H_plus_dual
    H = 1/2 * H
    f_J = Function('f_J', [qd_data], [H])
    return f_J


def system_evolution_casadi(N, ts):
    # Shape of the system
    n_states = dual_1d_data.numel()

    # Shape of the velocities

    n_controls = w_1d_data.numel()

    X = ca.MX.zeros(n_states, N + 1)
    U = ca.MX.sym('U', n_controls, N)
    P = ca.MX.sym('P', n_states)

    # Creatin dynamics function
    f = f_rk4_casadi_simple()

    X[:, 0] = P

    for k in range(N):
        st = X[:, k]
        con = U[:,k]
        X[:,k+1] = f(st, con, ts)

    f_prediction = ca.Function('f_prediction', [P, U], [X])

    return f_prediction

def optimization_casadi(N, ts):
    # Shape of the system
    n_states = dual_1d_data.numel()

    # Shape of the velocities

    n_controls = w_1d_data.numel()

    X = ca.MX.sym('X', n_states, N + 1)
    U = ca.MX.sym('U', n_controls, N)
    P = ca.MX.sym('P', n_states + n_states)

    # Creatin dynamics function
    f = f_rk4_casadi()
    lyapunov_cost =  lyapunov_casadi_simple()

    cost_fn = 0  # cost function
    g = X[:, 0] - P[:n_states]  # constraints in the equation

    for k in range(N):
        st = X[:, k]
        con = U[:, k]
        error = 2*lyapunov_cost(P[n_states:], st)
        R = ca.DM.eye(6)
        R[0, 0] = 0.02
        R[1, 1] = 0.02
        R[2, 2] = 0.02
        R[3, 3] = 0.01
        R[4, 4] = 0.01
        R[5, 5] = 0.01
        cost_fn = cost_fn + error + (con.T@R@con)
        st_next_RK4 = f(st, con, ts)
        st_next = X[:, k+1]
        g = ca.vertcat(g, st_next - st_next_RK4)

    # general Vector optimization
    OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
    )
    nlp_prob = {
        'f': cost_fn,
        'x': OPT_variables,
        'g': g,
        'p': P
        }

    opts = {'ipopt': {'max_iter': 100, 'print_level': 1, 'acceptable_tol': 1e-2, 'acceptable_obj_change_tol': 1e-2},'print_time': 1}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
    

    # Set bounds for control inputs
    lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
    ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

    lbx[0: n_states*(N+1): n_states] = -1     # X lower bound
    lbx[1: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[2: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[3: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[4: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[5: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[6: n_states*(N+1): n_states] = -ca.inf     # X lower bound
    lbx[7: n_states*(N+1): n_states] = -ca.inf     # X lower bound

    ubx[0: n_states*(N+1): n_states] = 1     # X lower bound
    ubx[1: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[2: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[3: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[4: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[5: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[6: n_states*(N+1): n_states] = ca.inf     # X lower bound
    ubx[7: n_states*(N+1): n_states] = ca.inf     # X lower bound

    lbx[n_states*(N+1):] = -3                  # v lower bound for all V
    ubx[n_states*(N+1):] = 3                  # v upper bound for all V


    args = {
        'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
        'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
        'lbx': lbx,
        'ubx': ubx
    }
    return solver, args

    
    
