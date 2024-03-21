#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions
from nav_msgs.msg import Odometry
from functions import dualquat_from_pose_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, rotation_casadi, rotation_inverse_casadi, dual_velocity_casadi, dual_quat_casadi, velocities_from_twist_casadi
from ode_acados import f_rk4_casadi_simple, noise
from nmpc_acados import create_ocp_solver
from acados_template import AcadosOcpSolver, AcadosSimSolver


# Creating Funtions based on Casadi
dualquat_from_pose = dualquat_from_pose_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()
velocity_from_twist = velocities_from_twist_casadi()
rot = rotation_casadi()
inverse_rot = rotation_inverse_casadi()
f_rk4 = f_rk4_casadi_simple()


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

def main(odom_pub_1, odom_pub_2, L):
    # Split Values
    m = L[0]
    g = L[4]
    # Sample Time Defintion
    sample_time = 0.03
    t_f = 30

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    t_N = 1.0
    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + sample_time, sample_time)
    N_prediction = N.shape[0]

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")
    message_ros = "DualQuaternion Casadi NMPC "

    # Odometry Message
    quat_1_msg = Odometry()
    quat_1_d_msg = Odometry()
    
    # Defining initial condition of the system and verify properties
    theta1 = 3.8134
    nx = 0.4896
    ny = 0.2032
    nz = 0.8480
    tx1 = 2
    ty1 = 2
    tz1 = 1

    # Initial Dualquaternion
    dual_1 = dualquat_from_pose(theta1, nx, ny,  nz, tx1, ty1, tz1)

    # Empty Matrices that is used to plot the results
    Q1_trans_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_quat_data = np.zeros((4, t.shape[0] + 1 - N_prediction), dtype=np.double)
    Q1_velocities_data = np.zeros((6, t.shape[0] - N_prediction), dtype=np.double)
    X = np.zeros((8, t.shape[0] + 1 - N_prediction), dtype=np.double)
    u = np.zeros((6, t.shape[0] - N_prediction), dtype=np.double)


    # Update initial condition in the empty matrices
    Q1_trans_data[:, 0] = np.array(get_trans(dual_1)).reshape((4, ))
    Q1_quat_data[:, 0] = np.array(get_quat(dual_1)).reshape((4, ))
    X[:, 0] = np.array(dual_1).reshape((8, ))
    

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
    angular_linear_1_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Angular Body linear Inertial
    dual_twist_1_d = dual_twist(angular_linear_1_d, dual_1_d)

    # Initial condition for the desired states
    X_d = np.zeros((14, t.shape[0]+1), dtype=np.double)
    X_d[:, 0] = np.array(ca.vertcat(dual_1_d, dual_twist_1_d)).reshape((14, ))
    for k in range(0, t.shape[0]):
        angular_linear_1_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Angular Body linear Inertial
        dual_twist_1_d = dual_twist(angular_linear_1_d, dual_1_d)
        dual_1_d = f_rk4(dual_1_d, dual_twist_1_d, sample_time)
        # Update Reference
        X_d[:, k + 1] = np.array(ca.vertcat(dual_1_d, dual_twist_1_d)).reshape((14, ))

    # Empty vector
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
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)

    # Dimensions of the optimization problem
    x_dim = ocp.model.x.size()[0]
    u_dim = ocp.model.u.size()[0]

    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", X[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((u_dim,)))

    #white_noise_cov = np.diag(np.zeros(12))
    sigma_x = 0.01
    sigma_y = 0.01
    sigma_z = 0.01
    sigma_theta_x = 0.001
    sigma_theta_y = 0.001
    sigma_theta_z = 0.001
    aux_noise = np.zeros(12)
    aux_noise[0] = sigma_x**2
    aux_noise[1] = sigma_y**2
    aux_noise[2] = sigma_z**2
    aux_noise[3] = sigma_theta_x**2
    aux_noise[4] = sigma_theta_y**2
    aux_noise[5] = sigma_theta_z**2
    uav_white_noise_cov = np.diag(aux_noise)

    # Simulation loop
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)
        # Check properties
        real = X[0:4, k]
        dual = X[4:8, k]
        # Chekc norm of the quaternion
        quat_check = get_quat(X[0:8, k])
        print("-----")
        print(np.linalg.norm(quat_check))
        print(np.dot(real, dual))
        print(white_noise)

        # Control Law Acados
        acados_ocp_solver.set(0, "lbx", X[:, k])
        acados_ocp_solver.set(0, "ubx", X[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = X_d[:,k+j]
            aux_ref = np.hstack((yref))
            acados_ocp_solver.set(j, "p", aux_ref)

        yref_N = X_d[:,k+N_prediction]
        aux_ref_N = np.hstack((yref_N))
        acados_ocp_solver.set(N_prediction, "p", aux_ref_N)

        # Check Solution since there can be possible errors 
        acados_ocp_solver.solve()

        # Get the control Action
        aux_control = acados_ocp_solver.get(0, "u")
        
        # Control action body twist
        u[:, k] = aux_control[0:6]

        # Compute body angular velocity and inertial linear velocity
        velocities  = velocity_from_twist(u[:, k], dual_1)

        # Save body angular and inertial linear velocitu
        Q1_velocities_data[:, k] = np.array(velocities).reshape((6, ))

        # Update Data of the system
        acados_integrator.set("x", X[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral = acados_integrator.solve()
        xcurrent = acados_integrator.get("x")

        # Update Data of the system

        X[:, k+1] = noise(xcurrent, white_noise)
        dual_1 = X[0:8, k+1]

        # Update Matrices of our system
        Q1_trans_data[:, k + 1] = np.array(get_trans(dual_1)).reshape((4, ))
        Q1_quat_data[:, k + 1] = np.array(get_quat(dual_1)).reshape((4, ))

        # Update matrices of the reference
        Q2_trans_data[:, k+1] = np.array(get_trans(X_d[0:8, k+1])).reshape((4, ))
        Q2_quat_data[:, k+1] = np.array(get_quat(X_d[0:8, k+1])).reshape((4, ))

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, X[0:8, k+1], 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_1_d_msg = get_odometry(quat_1_d_msg, X_d[0:8, k+1], 'quat_1_d')
        send_odometry(quat_1_d_msg, odom_pub_2)

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    fig11, ax11, ax12, ax13, ax14 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax12, ax13, ax14, Q1_quat_data[0:4, :], Q2_quat_data[0:4, :], t, "Quaternion Results Based On LieAlgebra Cost")
    plt.show()

    fig21, ax21, ax22, ax23 = fancy_plots_3()
    plot_states_position(fig21, ax21, ax22, ax23, Q1_trans_data[1:4, :], Q2_trans_data[1:4, :], t, "Position Results Based On LieAlgebra Cost")
    plt.show()
    fig31, ax31, ax32, ax33 = fancy_plots_3()
    plot_angular_velocities(fig31, ax31, ax32, ax33, Q1_velocities_data[0:3, :], t, "Body Angular velocities Based On LieAlgebra Cost")
    plt.show()

    fig41, ax41, ax42, ax43 = fancy_plots_3()
    plot_linear_velocities(fig41, ax41, ax42, ax43, Q1_velocities_data[3:6, :], t, "Inertial Linear velocities Based On LieAlgebra Cost")
    plt.show()


    return None
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
        main(odometry_publisher_1, odometry_publisher_2, L)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass