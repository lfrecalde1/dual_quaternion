#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control
from nav_msgs.msg import Odometry
from functions import dualquat_from_pose_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, rotation_casadi, rotation_inverse_casadi, dual_velocity_casadi, dual_quat_casadi, velocities_from_twist_casadi
from ode_acados import f_rk4_casadi_simple, noise, cost_quaternion_casadi, cost_translation_casadi, error_lie_norm
from nmpc_acados import create_ocp_solver
from acados_template import AcadosOcpSolver, AcadosSimSolver
import scipy.io
from scipy.io import savemat


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
error_lie = error_lie_norm()

#Identification = scipy.io.loadmat('Separed_cost.mat') 
Identification = scipy.io.loadmat('Separed_cost_without_velocities.mat') 
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

def main(odom_pub_1, odom_pub_2, L, x0, initial):
    # Split Values
    m = L[0]
    g = L[4]
    # Sample Time Defintion
    sample_time = 0.03
    t_f = 10

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)

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
    print(x0.shape)
    theta1 = x0[3]
    nx = x0[4]
    ny = x0[5]
    nz = x0[6]
    tx1 = x0[0]
    ty1 = x0[1]
    tz1 = x0[2]

    # Initial Dualquaternion
    dual_1 = dualquat_from_pose(theta1, nx, ny,  nz, tx1, ty1, tz1)
    angular_linear_1 = np.array([x0[7], x0[8], x0[9], x0[10], x0[11], x0[12]]) # Angular Body linear Inertial
    dual_twist_1 = dual_twist(angular_linear_1, dual_1)
    velocities  = dual_twist_1

    # Empty Matrices that are used to plot the results
    Q1_trans_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_quat_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_velocities_data = np.zeros((6, t.shape[0] + 1 - N_prediction), dtype=np.double)

    # Vector of the generalized states
    X = np.zeros((14, t.shape[0] + 1 - N_prediction), dtype=np.double)

    # Control Actions 
    u = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)
    F = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Update initial condition in the empty matrices
    Q1_trans_data[:, 0] = np.array(get_trans(dual_1)).reshape((4, ))
    Q1_quat_data[:, 0] = np.array(get_quat(dual_1)).reshape((4, ))
    Q1_velocities_data[:, 0] = np.array(velocities).reshape((6, ))
    X[:, 0] = np.array(ca.vertcat(dual_1, dual_twist_1)).reshape((14, ))

    # Constraints on control actions
    F_max = L[0]*L[4] + 20
    F_min = 0
    tau_1_max = 0.1
    tau_1_min = -0.1
    tau_2_max = 0.1
    tau_2_min = -0.1
    tau_3_max = 0.1
    taux_3_min = -0.1

    # Defining desired frame
    theta1_d = 0.0
    nx_d = 0.0
    ny_d = 0.0
    nz_d = 1.0
    tx1_d = 0
    ty1_d = 0
    tz1_d = 0

    # Initial Desired Dualquaternion
    dual_1_d = dualquat_from_pose(theta1_d, nx_d, ny_d,  nz_d, tx1_d, ty1_d, tz1_d)

    # Initial condition for the desired states
    X_d = np.zeros((14, t.shape[0]+1), dtype=np.double)
    X_d[0:8, 0] = np.array(dual_1_d).reshape((8, ))
    for k in range(0, t.shape[0]):
        angular_linear_1_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Angular Body linear Inertial
        dual_twist_1_d = dual_twist(angular_linear_1_d, dual_1_d)
        dual_1_d = f_rk4(dual_1_d, dual_twist_1_d, sample_time)
        # Update Reference
        X_d[0:8, k + 1] = np.array(dual_1_d).reshape((8, ))
        X_d[8:14, k] = np.array(dual_twist_1_d).reshape((6, ))

    # Desired Reference Inputs
    u_d = np.zeros((4, t.shape[0]), dtype=np.double)
    u_d[0, :] = m*g

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
    sigma_x = 0.00
    sigma_y = 0.00
    sigma_z = 0.00
    sigma_theta_x = 0.000
    sigma_theta_y = 0.000
    sigma_theta_z = 0.000
    sigma_vx = 0.0000
    sigma_vy = 0.0000
    sigma_vz = 0.0000
    sigma_wx = 0.0000
    sigma_wy = 0.0000
    sigma_wz = 0.0000
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

    lie_cost = np.zeros((2, t.shape[0] - N_prediction), dtype=np.double)
    lie_cost_real = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    lie_cost_dual = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    # Simulation loop
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)
        #acados_ocp_solver.options_set("rti_phase", 1)
        #acados_ocp_solver.solve()

        # Compute cost
        orientation_cost[:, k] = cost_quaternion(get_quat(X_d[0:8, k]), get_quat(X[0:8, k]))
        translation_cost[:, k] = cost_translation(get_trans(X_d[0:8, k]), get_trans(X[0:8, k]))
        lie_cost[:, k] = np.array(error_lie(X_d[0:8, k], X[0:8, k])).reshape((2, ))
        lie_cost_real[:, k] = lie_cost[0, k]
        lie_cost_dual[:, k] = lie_cost[1, k]

        # Check properties
        real = X[0:4, k]
        dual = X[4:8, k]
        # Chekc norm of the quaternion
        quat_check = get_quat(X[0:8, k])
        print("-----")
        print(np.linalg.norm(quat_check))
        print(np.dot(real, dual))

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

        stat_fields = ['time_tot', 'time_lin', 'time_qp', 'time_qp_solver_call', 'time_reg', 'sqp_iter']
        for field in stat_fields:
            print(f"{field} : {acados_ocp_solver.get_stats(field)}")

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

    fig11, ax11, ax12, ax13, ax14 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax12, ax13, ax14, Q1_quat_data[0:4, :], Q2_quat_data[0:4, :], t, "Quaternion Results Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    fig21, ax21, ax22, ax23 = fancy_plots_3()
    plot_states_position(fig21, ax21, ax22, ax23, Q1_trans_data[1:4, :], Q2_trans_data[1:4, :], t, "Position Results Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()
    fig31, ax31, ax32, ax33 = fancy_plots_3()
    plot_angular_velocities(fig31, ax31, ax32, ax33, Q1_velocities_data[0:3, :], t, "Body Angular velocities Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    fig41, ax41, ax42, ax43 = fancy_plots_3()
    plot_linear_velocities(fig41, ax41, ax42, ax43, Q1_velocities_data[3:6, :], t, "Inertial Linear velocities Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    # Control Actions
    fig51, ax51, ax52, ax53, ax54 = fancy_plots_4()
    plot_control_actions(fig51, ax51, ax52, ax53, ax54, F, M, t, "Control Actions of the System Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    # Sampling time
    fig61, ax61  = fancy_plots_1()
    plot_time(fig61, ax61, t_sample, delta_t, t, "Computational Time Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    fig71, ax71  = fancy_plots_1()
    plot_cost_orientation(fig71, ax71, orientation_cost, t, "Cost Orientation Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    fig81, ax81  = fancy_plots_1()
    plot_cost_translation(fig81, ax81, translation_cost, t, "Cost Translation Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()

    fig91, ax91  = fancy_plots_1()
    plot_cost_control(fig91, ax91, control_cost, t, "Cost Control Based On LieAlgebra Cost "+ str(initial), folder_path)
    plt.show()
    return X, X_d, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("DualQuaternions", disable_signals=True, anonymous=True)
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)
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

        # Multiple Experiments
        for k in range(x_0.shape[0]):
            x, xref, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction = main(odometry_publisher_1, odometry_publisher_2, L, x_0[k, :], k)
            Data_States.append(x)
            Data_reference.append(xref)
            Data_F.append(F)
            Data_M.append(M)
            Data_orientation_cost.append(orientation_cost)
            Data_translation_cost.append(translation_cost)
            Data_control_cost.append(control_cost)
            Data_time.append(t)
            Data_N_prediction.append(N_prediction)

        Data_States = np.array(Data_States)
        Data_reference = np.array(Data_reference)
        Data_F = np.array(Data_F)
        Data_M = np.array(Data_M)
        Data_orientation_cost = np.array(Data_orientation_cost)
        Data_translation_cost = np.array(Data_translation_cost)
        Data_control_cost = np.array(Data_control_cost)
        Data_time = np.array(Data_time)
        Data_N_prediction = np.array(Data_N_prediction)


        # Save Data matlab 
        mdic_x = {"x": Data_States, "xref": Data_reference, "F": Data_F, "M": Data_M, "orientation_cost": Data_orientation_cost,
                "translation_cost": Data_translation_cost, "control_cost": Data_control_cost, "t": Data_time, "N": Data_N_prediction}
        #savemat("Dual_cost" + ".mat", mdic_x)
        savemat("Dual_cost_without_velocities" + ".mat", mdic_x)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass