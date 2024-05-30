#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from nmpc import create_ocp_solver
from acados_template import AcadosOcpSolver, AcadosSimSolver
from export_ode_model import quatTorot_c, rotation_matrix_error_c, quaternion_error_c, ln_quaternion_c, quaternion_conjugate_c, Ad_c
from scipy.linalg import logm
from fancy_plots import plot_cost_orientation_rotational, fancy_plots_1, plot_cost_orientation_quat, fancy_plots_3, plot_control_actions_angular

rotation_matrix_error = rotation_matrix_error_c()
quaternion_error = quaternion_error_c()
ln_quaternion = ln_quaternion_c()
quaternion_conjugate = quaternion_conjugate_c()
adjoint = Ad_c()


import os
script_dir = os.path.dirname(__file__)
#folder_path = os.path.join(script_dir, 'cost_with_velocities/')
folder_path = os.path.join(script_dir, 'cost_without_velocities/')

def set_odom_msg(msg, x):
    # Set the odometry of the quadrotor
    # INPUT 
    # msg                                                               - odometry msg
    # x                                                                 - states of the system
    # OUTPUT
    # msg                                                               - odometry msg
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    #msg.child_frame_id = "blue_robot_base"

    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = x[1]
    msg.pose.pose.orientation.y = x[2]
    msg.pose.pose.orientation.z = x[3]
    msg.pose.pose.orientation.w = x[0]

    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0

    msg.twist.twist.angular.x = x[4]
    msg.twist.twist.angular.y = x[5]
    msg.twist.twist.angular.z = x[6]
    return msg

def send_odom_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None

def get_random_quaternion_complete(number):
    random_rotation = R.random(number)
    
    # Convert the rotation to quaternion format
    quaternion = random_rotation.as_quat()  # Returns (x, y, z, w)
    return quaternion

def hedge(x):
    X = np.zeros((3, 3), dtype=np.double)
    X[0, 1] = -x[2]
    X[0, 2] = x[1]
    X[1, 0] = x[2]
    X[1, 2] = -x[0]
    X[2, 0] = -x[1]
    X[2, 1] = x[0]
    return X

def vee(X):
    x = np.zeros((3, 1), dtype=np.double)
    x[0, 0] = X[2, 1]
    x[1, 0] = X[0, 2]
    x[2, 0] = X[1, 0]
    return x

def compute_error_r3(x_d, x):
    error = rotation_matrix_error(x_d, x)
    so3 = logm(error)
    r3 = np.zeros((3, 1), dtype=np.double)
    r3[0, 0] = so3[2, 1]
    r3[1, 0] = so3[0, 2]
    r3[2, 0] = so3[1, 0]
    return r3

def control_law_SO3(x_d, x, L):
    # Gain matrices
    Kp = np.zeros((3, 3), dtype=np.double)
    Kv = np.zeros((3, 3), dtype=np.double)

    Kp[0, 0] = 1000
    Kp[1, 1] = 1000
    Kp[2, 2] = 1000

    Kv[0, 0] = 100
    Kv[1, 1] = 100
    Kv[2, 2] = 100

    # Inertia Matrix
    J = np.zeros((3, 3), dtype=np.double)
    J[0, 0] = L[0]
    J[1, 1] = L[1]
    J[2, 2] = L[2]

    # Quaternions
    q_d = x_d[0:4]
    q = x[0:4]

    # Angular velocities
    w_d = x_d[4:7]
    w = x[4:7]

    error = rotation_matrix_error(q_d, q)
    error_T = rotation_matrix_error(q, q_d)
    error_d = w - error@w_d
    print("Rot")
    print(error)
    print(error_d)


    aux = np.cross(w, J@w)
    aux = aux.reshape((3, 1))
    law = aux - (1/2) * J@Kp @ vee(error-error_T) - J@Kv @ error_d
    #law = aux - (1/2) * J@Kp @ vee(logm(error)) - J@Kv @ error_d
    law = np.array(law).reshape((3, ))
    return law

def control_law_S3(x_d, x, L):
    # Gain matrices
    Kp = np.zeros((3, 3), dtype=np.double)
    Kv = np.zeros((3, 3), dtype=np.double)

    Kp[0, 0] = 2000
    Kp[1, 1] = 2000
    Kp[2, 2] = 2000

    Kv[0, 0] = 100
    Kv[1, 1] = 100
    Kv[2, 2] = 100

    # Inertia Matrix
    J = np.zeros((3, 3), dtype=np.double)
    J[0, 0] = L[0]
    J[1, 1] = L[1]
    J[2, 2] = L[2]

    # Quaternions
    q_d = x_d[0:4]
    q = x[0:4]

    # Angular velocities
    w_d = x_d[4:7]
    w = x[4:7]

    error_q = quaternion_error(q_d, q)
    error_q_c = quaternion_conjugate(error_q)
    error_d = w - adjoint(error_q_c, w_d)

    aux = np.cross(w, J@w)
    aux = aux.reshape((3, 1))

    # Control LAW
    law = aux - (2)* J@Kp @ ln_quaternion(error_q) - J@Kv @ error_d
    law = np.array(law).reshape((3, ))

    print("Quat")
    print(error_q)
    print(error_d)

    return law

def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list, odom_pub_1, odom_pub_2, odom_pub_d, initial)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # OUTPUS 
    # x                         - states of the system
    # F                         - force control action
    # M                         - torques of the system
    # delta_t                   - computational time

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)

    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized states of the system
    x = np.zeros((7, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    x2 = np.zeros((7, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x2[:, 0] = x_0

    # Generalized control actions
    u = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)
    u2 = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)
    u2_d = np.zeros((3, t.shape[0]), dtype=np.double)

    # Defining Desired Orientation
    theta_d = 0.0
    n_d = np.array([0.0, 0.0, 1.0])
    q_d = np.hstack([np.cos(theta_d / 2), np.sin(theta_d / 2) * np.array(n_d)])

    x_d = np.zeros((7, t.shape[0] + 1), dtype=np.double)
    x_d2 = np.zeros((7, t.shape[0] + 1), dtype=np.double)

    for k in range(0, t.shape[0] + 1):
        x_d2[0:4, k] = q_d
        x_d[0:4, k] = q_d

    # Odometry message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()
    quat_d_msg = Odometry()

    # Set Odom Message
    quat_1_msg = set_odom_msg(quat_1_msg, x[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)

    quat_2_msg = set_odom_msg(quat_2_msg, x2[:, 0])
    send_odom_msg(quat_2_msg, odom_pub_2)

    quat_d_msg = set_odom_msg(quat_d_msg, x_d[:, 0])
    send_odom_msg(quat_d_msg, odom_pub_d)

    # ODE Solver using Acados
    tau_1_max = 300
    tau_1_min = -300
    tau_2_max = 300
    tau_2_min = -300
    tau_3_max = 300
    taux_3_min = -300

    # Optimization problem
    ocp = create_ocp_solver(x[:, 0], N_prediction, t_N, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, ts)
    ocp2 = create_ocp_solver(x2[:, 0], N_prediction, t_N, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, ts)

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)

    acados_ocp_solver_2 = AcadosOcpSolver(ocp2, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_integrator_2 = AcadosSimSolver(ocp2, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)

    acados_ocp_solver.reset()

    # Reset Solver
    x_dim = ocp2.model.x.size()[0]
    u_dim = ocp2.model.u.size()[0]

    # Initial Conditions optimization problem
    for stage in range(N_prediction + 1):
        acados_ocp_solver_2.set(stage, "x", x2[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver_2.set(stage, "u", u2_d[:, 0])

    norm_orientation_1 =  np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    norm_orientation_2 =  np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    message_ros = "Geometric Controller "

    # Simulation of the system
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        # Check shortest path
        for j in range(N_prediction):
            # check shortest path
            error_dual_no_filter = np.array(quaternion_error(x_d2[0:4, k+j], x2[0:4, k])).reshape((4, ))
            if error_dual_no_filter[0] >= 0.0:
                x_d2[0:4, k+j] = x_d2[0:4, k+j]
            else:
                x_d2[0:4, k+j] = -x_d2[0:4, k+j]
        error_dual_no_filter = np.array(quaternion_error(x_d2[0:4, k+N_prediction], x2[0:4, k])).reshape((4, ))
        if error_dual_no_filter[0] >= 0.0:
            x_d2[0:4, k+N_prediction] = x_d2[0:4, k+N_prediction]
        else:
            x_d2[0:4, k+N_prediction] = -x_d2[0:4, k+N_prediction]
        # Compute error matrix mode
        error_r3_1  = compute_error_r3(x_d[0:4, k], x[0:4, k])
        error_r3_2  = compute_error_r3(x_d[0:4, k], x2[0:4, k])
        norm_orientation_1[:, k] = np.linalg.norm(error_r3_1)
        norm_orientation_2[:, k] = np.linalg.norm(error_r3_2)

        # Control Law
        u[:, k] = control_law_SO3(x_d[:, k], x[:, k], L)
        #u2[:, k] = control_law_S3(x_d[:, k], x2[:, k], L)

        # Control Law Acados
        acados_ocp_solver_2.set(0, "lbx", x2[:, k])
        acados_ocp_solver_2.set(0, "ubx", x2[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = x_d2[:,k+j]
            uref = u2_d[:,k+j]
            aux_ref = np.hstack((yref, uref))
            acados_ocp_solver_2.set(j, "p", aux_ref)

        # Desired Trayectory at the last Horizon
        yref_N = x_d2[:,k+N_prediction]
        uref_N = u2_d[:,k+N_prediction]
        aux_ref_N = np.hstack((yref_N, uref_N))
        acados_ocp_solver_2.set(N_prediction, "p", aux_ref_N)

        # Check Solution since there can be possible errors 
        acados_ocp_solver_2.solve()

        # Get the control Action
        aux_control = acados_ocp_solver_2.get(0, "u")
        u2[:, k] = aux_control

        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic
        delta_t[:, k] = toc_solver

        # Dynamics System 1
        acados_integrator.set("x", x[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral_1 = acados_integrator.solve()
        xcurrent_1 = acados_integrator.get("x")
        x[:, k+1] = xcurrent_1

        # Dynamics system 2
        acados_integrator_2.set("x", x2[:, k])
        acados_integrator_2.set("u", u2[:, k])

        status_integral_2 = acados_integrator_2.solve()
        xcurrent_2 = acados_integrator_2.get("x")
        x2[:, k+1] = xcurrent_2

        # Send msg to Ros
        quat_1_msg = set_odom_msg(quat_1_msg, x[:, k+1])
        send_odom_msg(quat_1_msg, odom_pub_1)

        quat_2_msg = set_odom_msg(quat_2_msg, x2[:, k+1])
        send_odom_msg(quat_2_msg, odom_pub_2)

        quat_d_msg = set_odom_msg(quat_d_msg, x_d[:, k+1])
        send_odom_msg(quat_d_msg, odom_pub_d)

        rospy.loginfo(message_ros + str(toc_solver))
    
    fig11, ax11  = fancy_plots_1()
    plot_cost_orientation_rotational(fig11, ax11, norm_orientation_1, t, "Cost Total R "+ str(initial), folder_path)
    
    fig12, ax12  = fancy_plots_1()
    plot_cost_orientation_quat(fig12, ax12, norm_orientation_2, t, "Cost Total Quat Lie"+ str(initial), folder_path)

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_control_actions_angular(fig13, ax13, ax23, ax33, u, t, "Control Actions of the System R "+ str(initial), folder_path)

    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_control_actions_angular(fig14, ax14, ax24, ax34, u2, t, "Control Actions of the System Quat Lie"+ str(initial), folder_path)


if __name__ == '__main__':
    try:
        # Node Parameters
        rospy.init_node("Geometric_Controller", disable_signals=True, anonymous=True)
        odomety_topic_1 = "/" + "frame_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "frame_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        odomety_topic_d = "/" + "frame_d" + "/odom"
        odometry_publisher_d = rospy.Publisher(odomety_topic_d, Odometry, queue_size = 10)

        # Time parameters
        ts = 0.01
        t_f = 2
        t_N = 0.1

        # System Dynamic Parameters
        Jxx = 1.0
        Jyy = 3.0
        Jzz = 5.0
        L = [Jxx, Jyy, Jzz]
        number_experiments = 1
        ramdon_quaternions = get_random_quaternion_complete(number_experiments)

        # Empty Matrix for my inital States
        X_total = []
        for i_random in range(number_experiments):
            omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
            #angle_0, axis_0 = get_random_quaternion()
            quat_0 = np.array([ramdon_quaternions[i_random, 3], ramdon_quaternions[i_random, 0], ramdon_quaternions[i_random, 1], ramdon_quaternions[i_random, 2]])
            #quat_0 = np.array([1, 0.0, 0.0, 0.0])
            theta_0 = 0.99*np.pi
            n_0 = np.array([0.0, 0.0, 1.0])
            #quat_0 = np.hstack([np.cos(theta_0 / 2), np.sin(theta_0 / 2) * np.array(n_0)])
            x = np.hstack((quat_0, omega_0))
            X_total.append(x)
        
        X_total = np.array(X_total)
        for k in range(0, X_total.shape[0]):
            main(ts, t_f, t_N, X_total[k, :], L, odometry_publisher_1, odometry_publisher_2, odometry_publisher_d, k)
        #savemat("Dual_cost_without_velocities" + ".mat", mdic_x)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass