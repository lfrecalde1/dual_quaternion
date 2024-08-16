#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import itertools
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control, plot_cost_total
from nav_msgs.msg import Odometry
from functions import dualquat_from_pose_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, rotation_casadi, rotation_inverse_casadi, dual_velocity_casadi, dual_quat_casadi, velocities_from_twist_casadi
from ode_acados import f_rk4_casadi_simple, noise, cost_quaternion_casadi, cost_translation_casadi
from ode_acados import error_dual_aux_casadi
from nmpc_acados import create_ocp_solver
from ode_acados import compute_flatness_states, trajectory
from acados_template import AcadosOcpSolver, AcadosSimSolver
import scipy.io
from scipy.io import savemat
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


# Creating Funtions based on Casadi
dualquat_from_pose = dualquat_from_pose_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()
velocity_from_twist = velocities_from_twist_casadi()
rot = rotation_casadi()
inverse_rot = rotation_inverse_casadi()
f_rk4 = f_rk4_casadi_simple()
cost_quaternion = cost_quaternion_casadi()
cost_translation = cost_translation_casadi()
error_dual_f = error_dual_aux_casadi()

Identification = scipy.io.loadmat('Separed_cost.mat') 
#Identification = scipy.io.loadmat('Separed_cost_without_velocities.mat') 
x_0 = Identification['x_init']

import os
script_dir = os.path.dirname(__file__)
#folder_path = os.path.join(script_dir, 'cost_with_velocities/')
folder_path = os.path.join(script_dir, 'cost_without_velocities/')

def get_odometry(odom_msg, dqd, name):
    # Function to send the Oritentation of the Quaternion
    # Get Information from the DualQuaternion
    t_d = get_trans(dqd)

    q_d = get_quat(dqd)

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = name
    odom_msg.pose.pose.position.x = t_d[1]
    odom_msg.pose.pose.position.y = t_d[2]
    odom_msg.pose.pose.position.z = t_d[3]

    odom_msg.pose.pose.orientation.x = q_d[1]
    odom_msg.pose.pose.orientation.y = q_d[2]
    odom_msg.pose.pose.orientation.z = q_d[3]
    odom_msg.pose.pose.orientation.w = q_d[0]
    return odom_msg

def send_odometry(odom_msg, odom_pub):
    # Function to send the orientation of the Quaternion
    odom_pub.publish(odom_msg)
    return None

def init_marker(marker_msg, x):
    marker_msg.header.frame_id = "world"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.ns = "trajectory"
    marker_msg.id = 0
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.02  # Line width
    marker_msg.color.a = 1.0  # Alpha
    marker_msg.color.r = 0.0  # Red
    marker_msg.color.g = 1.0  # Green
    marker_msg.color.b = 0.0  # Blue
    point = Point()
    point.x = x[0]
    point.y = x[1]
    point.z = x[2]
    points = [point]
    marker_msg.points = points
    return marker_msg, points

def send_marker(marker_msg, points, publisher, x):
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    point = Point()
    point.x = x[0]
    point.y = x[1]
    point.z = x[2]
    points.append(point)
    marker_msg.points = points
    publisher.publish(marker_msg)
    return marker_msg, points

def main(odom_pub_1, odom_pub_2, trajec_pub, L, x0, v_max, a_max, n, initial):
    # Split Values
    m = L[0]
    g = L[4]
    # Sample Time Defintion
    sample_time = 0.01
    t_inital = 2
    t_trajectory = 30
    t_final = 2

    hd, hd_d, qd, w_d, f_d, M_d, t = compute_flatness_states(L, x0[0:3], t_inital, t_trajectory, t_final, sample_time, 2, (initial + 1)*0.5)
    # Time defintion aux variable

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    t_N = 0.5
    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + sample_time, sample_time)
    N_prediction = N.shape[0]

    # Aux variables samplte time
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = sample_time*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")
    message_ros = "DualQuaternion Casadi NMPC "

    # Odometry Message
    quat_1_msg = Odometry()
    quat_1_d_msg = Odometry()
    
    # Defining initial condition of the system and verify properties
    tx1 = x0[0]
    ty1 = x0[1]
    tz1 = x0[2]
    qw1 = x0[3]
    qx1 = x0[4]
    qy1 = x0[5]
    qz1 = x0[6]

    # Initial Dualquaternion
    dual_1 = dualquat_from_pose(qw1, qx1, qy1,  qz1, tx1, ty1, tz1)
    angular_linear_1 = np.array([x0[7], x0[8], x0[9], x0[10], x0[11], x0[12]]) # Angular Body linear Inertial
    dual_twist_1 = dual_twist(angular_linear_1, dual_1)
    velocities  = dual_twist_1

    # Empty Matrices that are used to plot the results
    Q1_trans_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_quat_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_velocities_data = np.zeros((6, t.shape[0] + 1 - N_prediction), dtype=np.double)

    # Vector of the generalized states
    X = np.zeros((14, t.shape[0] + 1 - N_prediction), dtype=np.double)
    #X_aux = np.zeros((8, t.shape[0] + 1 - N_prediction), dtype=np.double)

    # Control Actions 
    u = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)
    F = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Update initial condition in the empty matrices
    Q1_trans_data[:, 0] = np.array(get_trans(dual_1)).reshape((4, ))
    Q1_quat_data[:, 0] = np.array(get_quat(dual_1)).reshape((4, ))
    Q1_velocities_data[:, 0] = np.array(velocities).reshape((6, ))
    X[:, 0] = np.array(ca.vertcat(dual_1, dual_twist_1)).reshape((14, ))
    #X_aux[:, 0] = np.array(ca.vertcat(get_trans(dual_1), get_quat(dual_1))).reshape((8, ))

    # Constraints on control actions
    F_max = L[0]*L[4] + 20
    F_min = 0
    tau_1_max = 0.1
    tau_1_min = -0.1
    tau_2_max = 0.1
    tau_2_min = -0.1
    tau_3_max = 0.1
    taux_3_min = -0.1

    # Desired Trajectory
    #hd, hd_d, qd, w_d, f_d, M_d = compute_reference(t, sample_time, v_max, a_max, n, L)
    #hd, hd_d, hd_dd, hd_ddd, hd_dddd, theta, theta_d, theta_dd = trajectory(t, 2, (initial +1)*0.5)
    #hd, hd_d, qd, w_d, f_d, M_d = compute_flatness_states(t, L,  2, (initial + 1)*0.5)
    #hd, hd_d, qd, w_d, f_d, M_d = compute_flatness_states(t, L,  2, (initial + 1)*0.5, x0[0:3], sample_time)


    # Initial condition for the desired states
    X_d = np.zeros((14, t.shape[0]+1), dtype=np.double)

    # Desired Reference Inputs
    u_d = np.zeros((4, t.shape[0]), dtype=np.double)

    for k in range(0, t.shape[0]):
        u_d[0, k] = f_d[0, k]
        u_d[1, k] = M_d[0, k]
        u_d[2, k] = M_d[1, k]
        u_d[3, k] = M_d[2, k]

        qw1_d = qd[0, k]
        qx1_d = qd[1, k]
        qy1_d = qd[2, k]
        qz1_d = qd[3, k]
        tx1_d = hd[0, k]
        ty1_d = hd[1, k]
        tz1_d = hd[2, k]

        # Initial Desired Dualquaternion
        dual_1_d = dualquat_from_pose(qw1_d, qx1_d, qy1_d,  qz1_d, tx1_d, ty1_d, tz1_d)

        hxd_d = hd_d[0, k]
        hyd_d = hd_d[1, k]
        hzd_d = hd_d[2, k]
        wx_d = w_d[0, k]
        wy_d = w_d[1, k]
        wz_d = w_d[2, k]

        angular_linear_1_d = np.array([wx_d, wy_d, wz_d, hxd_d, hyd_d, hzd_d]) # Angular Body linear Inertial

        # Update Reference
        X_d[8:14, k] = angular_linear_1_d
        X_d[0:8, k] = np.array(dual_1_d).reshape((8, ))


    # Empty vectors for the desired Dualquaernion
    Q2_trans_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q2_quat_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)

    Q2_trans_data[:, 0] = np.array(get_trans(X_d[0:8, 0])).reshape((4, ))
    Q2_quat_data[:, 0] = np.array(get_quat(X_d[0:8, 0])).reshape((4, ))

    # Odometry message
    quat_1_msg = get_odometry(quat_1_msg, X[0:8, 0], 'quat_1')
    send_odometry(quat_1_msg, odom_pub_1)

    quat_1_d_msg = get_odometry(quat_1_d_msg, X_d[0:8, 0], 'quat_1_d')
    send_odometry(quat_1_d_msg, odom_pub_2)

    marker_msg = Marker()
    marker_msg, points = init_marker(marker_msg, hd[:, 0])

    # Optimization problem
    ocp = create_ocp_solver(X[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, sample_time)
    #acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= False, generate= False)

    # Integration using Acados
    #acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= False, generate= False)

    # Dimensions of the optimization problem
    x_dim = ocp.model.x.size()[0]
    u_dim = ocp.model.u.size()[0]

    # Reset Solver
    acados_ocp_solver.reset()

    # Initial Conditions optimization problem
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", X[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", u_d[:, 0])

    # Noise
    sigma_x = 0.01
    sigma_y = 0.01
    sigma_z = 0.01
    sigma_theta_x = 0.001
    sigma_theta_y = 0.001
    sigma_theta_z = 0.001
    sigma_vx = 0.0001
    sigma_vy = 0.0001
    sigma_vz = 0.0001
    sigma_wx = 0.0001
    sigma_wy = 0.0001
    sigma_wz = 0.0001
    aux_noise = np.zeros(12)
    aux_noise[0] = sigma_x**2
    aux_noise[1] = sigma_y**2
    aux_noise[2] = sigma_z**2
    aux_noise[3] = sigma_theta_x**2
    aux_noise[4] = sigma_theta_y**2
    aux_noise[5] = sigma_theta_z**2
    aux_noise[6] = sigma_vx**2
    aux_noise[7] = sigma_vy**2
    aux_noise[8] = sigma_vz**2
    aux_noise[9] = sigma_wx**2
    aux_noise[10] = sigma_wy**2
    aux_noise[11] = sigma_wz**2
    uav_white_noise_cov = np.diag(aux_noise)

    # Cost Values
    orientation_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    translation_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    control_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    total_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    lie_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    # KKT conditions
    kkt_values = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)
    sqp_iteration = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    error_dual_no_filter = np.array(error_dual_f(X_d[0:8, 0], X[0:8, 0])).reshape((8, ))
    if error_dual_no_filter[0] > 0.0:
        X_d[0:8, :] = X_d[0:8, :]
    else:
        X_d[0:8, :] = -X_d[0:8, :]
    # Simulation loop
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        marker_msg, points = send_marker(marker_msg, points, trajec_pub, hd[0:, k])
#
        white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)

        # Compute cost
        orientation_cost[:, k] = cost_quaternion(get_quat(X_d[0:8, k]), get_quat(X[0:8, k]))
        translation_cost[:, k] = cost_translation(get_trans(X_d[0:8, k]), get_trans(X[0:8, k]))
        total_cost[:, k] = orientation_cost[:, k] + translation_cost[:, k]

        # Check properties
        real = X[0:4, k]
        dual = X[4:8, k]
        # Chekc norm of the quaternion
        quat_check = get_quat(X[0:8, k])
        #print("-----")
        #print(np.linalg.norm(quat_check))
        #print(np.dot(real, dual))
        # Control Law Acados
        acados_ocp_solver.set(0, "lbx", X[:, k])
        acados_ocp_solver.set(0, "ubx", X[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = X_d[:,k+j]
            uref = u_d[:,k+j]
            aux_ref = np.hstack((yref, uref))
            acados_ocp_solver.set(j, "p", aux_ref)

        # Desired Trayectory at the last Horizon
        yref_N = X_d[:,k+N_prediction]
        uref_N = u_d[:,k+N_prediction]
        aux_ref_N = np.hstack((yref_N, uref_N))
        acados_ocp_solver.set(N_prediction, "p", aux_ref_N)

        # Check Solution since there can be possible errors 
        #acados_ocp_solver.options_set("rti_phase", 2)
        acados_ocp_solver.solve()

        stat_fields = ['statistics', 'time_tot', 'time_lin', 'time_sim', 'time_sim_ad', 'time_sim_la', 'time_qp', 'time_qp_solver_call', 'time_reg', 'sqp_iter', 'residuals', 'qp_iter', 'alpha']

        for field in stat_fields:
            #print(f"{field} : {acados_ocp_solver.get_stats(field)}")
            None
        print(initial)
        kkt_values[:, k]  = acados_ocp_solver.get_stats('residuals')
        sqp_iteration[:, k] = acados_ocp_solver.get_stats('sqp_iter')
        #acados_ocp_solver.print_statistics()

        # Get the control Action
        aux_control = acados_ocp_solver.get(0, "u")
        F[:, k] = aux_control[0]
        M[:, k] = aux_control[1:4]
        u[0, k] = F[:, k]
        u[1:4, k] = M[:, k]

        control_cost[:, k] = np.dot(u[:, k], u[:, k])
        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic
        delta_t[:, k] = toc_solver

        # Update Data of the system
        acados_integrator.set("x", X[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral = acados_integrator.solve()
        xcurrent = acados_integrator.get("x")

        # Update Data of the system
        #X[:, k+1] = xcurrent
        X[:, k+1] = noise(xcurrent, white_noise)

        # Update Matrices of our system
        Q1_trans_data[:, k + 1] = np.array(get_trans(X[0:8, k+1])).reshape((4, ))
        Q1_quat_data[:, k + 1] = np.array(get_quat(X[0:8, k+1])).reshape((4, ))
        # Compute body angular velocity and inertial linear velocity
        velocities  = X[8:14, k +1]
        # Save body angular and inertial linear velocitu
        Q1_velocities_data[:, k + 1] = np.array(velocities).reshape((6, ))

        # Update matrices of the reference
        Q2_trans_data[:, k+1] = np.array(get_trans(X_d[0:8, k+1])).reshape((4, ))
        Q2_quat_data[:, k+1] = np.array(get_quat(X_d[0:8, k+1])).reshape((4, ))

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, X[0:8, k+1], 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_1_d_msg = get_odometry(quat_1_d_msg, X_d[0:8, k+1], 'quat_1_d')
        send_odometry(quat_1_d_msg, odom_pub_2)

        rospy.loginfo(message_ros + str(delta_t[:, k]))
    
    # Normalize cost
    orientation_cost = orientation_cost/1
    translation_cost = translation_cost/1
    control_cost = control_cost/1
    total_cost = total_cost/np.max(total_cost)

    fig11, ax11, ax12, ax13, ax14 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax12, ax13, ax14, Q1_quat_data[0:4, :], Q2_quat_data[0:4, :], t, "Quaternion Results Based On LieAlgebra Cost "+ str(initial), folder_path)

    fig21, ax21, ax22, ax23 = fancy_plots_3()
    plot_states_position(fig21, ax21, ax22, ax23, Q1_trans_data[1:4, :], Q2_trans_data[1:4, :], t, "Position Results Based On LieAlgebra Cost "+ str(initial), folder_path)
    # Control Actions
    fig51, ax51, ax52, ax53, ax54 = fancy_plots_4()
    plot_control_actions(fig51, ax51, ax52, ax53, ax54, F, M, t, "Control Actions of the System Based On LieAlgebra Cost "+ str(initial), folder_path)

    fig101, ax101  = fancy_plots_1()
    plot_cost_total(fig101, ax101, orientation_cost, t, "Cost Ori Based On LieAlgebra Cost "+ str(initial), folder_path)

    return X, X_d, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction, kkt_values, sqp_iteration

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("DualQuaternions", disable_signals=True, anonymous=True)
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        trajectory_topic = "/" + "ref"
        trajectory_publisher = rospy.Publisher(trajectory_topic, Marker, queue_size = 10)
        # Dynamics Parameters
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # empty matrices
        Data_States = []
        Data_reference = []
        Data_F = []
        Data_M = []
        Data_orientation_cost = []
        Data_translation_cost = []
        Data_control_cost = []
        Data_time = []
        Data_N_prediction = []
        Data_KKT = []
        Data_sqp = []

        print(x_0.shape[0])

        #a_max = np.array([1 ,2, 3, 4])*0.3
        #v_max = np.array([1, 1.5, 2, 2.5, 3, 3.5, 4])*1

        a_max = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])*0.3
        v_max = np.array([1])*1

        # Use itertools.product to get all possible combinations
        combinations = np.array(list(itertools.product(v_max, a_max)))
        print(combinations)
        number_experiments = combinations.shape[0]
        print(number_experiments)

        # Multiple Experiments
        for k in range(x_0.shape[0]):
            x, xref, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction, kkt_values, sqp_iteration = main(odometry_publisher_1, odometry_publisher_2, trajectory_publisher, L, x_0[k, :],  combinations[k, 0], combinations[k, 1], 1, k)
            Data_States.append(x)
            Data_reference.append(xref)
            Data_F.append(F)
            Data_M.append(M)
            Data_orientation_cost.append(orientation_cost)
            Data_translation_cost.append(translation_cost)
            Data_control_cost.append(control_cost)
            Data_time.append(t)
            Data_N_prediction.append(N_prediction)
            Data_KKT.append(kkt_values)
            Data_sqp.append(sqp_iteration)

        Data_States = np.array(Data_States)
        Data_reference = np.array(Data_reference)
        Data_F = np.array(Data_F)
        Data_M = np.array(Data_M)
        Data_orientation_cost = np.array(Data_orientation_cost)
        Data_translation_cost = np.array(Data_translation_cost)
        Data_control_cost = np.array(Data_control_cost)
        Data_time = np.array(Data_time)
        Data_N_prediction = np.array(Data_N_prediction)
        Data_KKT = np.array(Data_KKT)
        Data_sqp = np.array(Data_sqp)

        # Save Data matlab 
        mdic_x = {"x": Data_States, "xref": Data_reference, "F": Data_F, "M": Data_M, "orientation_cost": Data_orientation_cost,
                "translation_cost": Data_translation_cost, "control_cost": Data_control_cost, "t": Data_time, "N": Data_N_prediction, 'KKT': Data_KKT, 'sqp': Data_sqp}
        savemat("Dual_cost" + ".mat", mdic_x)
        #savemat("Dual_cost_without_velocities" + ".mat", mdic_x)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass