#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from acados_template import AcadosSim, AcadosSimSolver, AcadosOcp, AcadosOcpSolver
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from functions import dualquat_from_pose_casadi
from fancy_plots import fancy_plots_3, fancy_plots_4, fancy_plots_1
from fancy_plots import plot_dual_real_reference, plot_dual_dual_reference, plot_states_velocity_reference, plot_states_angular_reference, plot_states_position, plot_states_quaternion
from nmpc import create_ocp_solver
from ode_acados import compute_reference, rotation_inverse_casadi

rotation_body = rotation_inverse_casadi()
import os
script_dir = os.path.dirname(__file__)
folder_path = os.path.join(script_dir, 'results_nominal/')

def set_odom_msg(msg, x):
    # Set the odometry of the quadrotor
    # INPUT 
    # msg                                                               - odometry msg
    # x                                                                 - states of the system
    # OUTPUT
    # msg                                                               - odometry msg
    t = x[0:3]
    q = x[3:7]
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"

    msg.pose.pose.position.x = t[0]
    msg.pose.pose.position.y = t[1]
    msg.pose.pose.position.z = t[2]

    msg.pose.pose.orientation.x = q[1]
    msg.pose.pose.orientation.y = q[2]
    msg.pose.pose.orientation.z = q[3]
    msg.pose.pose.orientation.w = q[0]

    return msg

def send_odom_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None

def main(ts: float, t_f: float, t_N: float, L: list, odom_pub_1, odom_pub_2, initial)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # odom_pub_1                - Publisher system states
    # odom_pub_2                - Publisher system desired reference
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


    # Initial States of the system
    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    angle = 0.0*np.pi/2
    axis = np.array([0, 0, 1])
    quat_0 = np.hstack([np.cos(angle / 2), np.sin(angle / 2) * axis])

    # Init dual quaternions
    X1 = np.zeros((7, t.shape[0] + 1 -N_prediction), dtype=np.double)
    X1[:, 0] = np.array([x0, y0, z0, quat_0[0], quat_0[1], quat_0[2], quat_0[3]])

    # Linear velocity inertial frame and angular velocity body frame
    u = np.zeros((6, t.shape[0] - N_prediction), dtype=np.double)
    xi = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)

   # Odometry message 
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()


    # Message Ros
    message_ros = "Dual Quaternion Integrations "

    # Constrainsts
    wx_max = + 7
    wy_max = + 7
    wz_max = + 7
    vx_max = + 5
    vy_max = + 5
    vz_max = + 5

    wx_min = - 7 
    wy_min = - 7 
    wz_min = - 7 
    vx_min = - 5 
    vy_min = - 5 
    vz_min = - 5 

    # Integration set up
    ocp = create_ocp_solver(X1[:, 0], N_prediction, t_N, wx_max, wx_min, wy_max, wy_min, wz_max, wz_min, vx_max, vx_min, vy_max, vy_min,  vz_max, vz_min, L, ts)
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)

    # Integration Without Drag
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)

    # Init optimizer
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", X1[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", u[:, 0])

    hd, hd_d, qd, w_d, f_d, M_d = compute_reference(t, ts, 70.0*(initial+1), L)

    # Initial condition for the desired states
    X_d = np.zeros((13, t.shape[0]+1), dtype=np.double)

    for k in range(0, t.shape[0]):
        qw1_d = qd[0, k]
        qx1_d = qd[1, k]
        qy1_d = qd[2, k]
        qz1_d = qd[3, k]
        tx1_d = hd[0, k]
        ty1_d = hd[1, k]
        tz1_d = hd[2, k]

        velocity_body = rotation_body(qd[:, k], hd_d[:, k])
        hxd_d = velocity_body[0, 0]
        hyd_d = velocity_body[1, 0]
        hzd_d = velocity_body[2, 0]

        wx_d = w_d[0, k]
        wy_d = w_d[1, k]
        wz_d = w_d[2, k]

        angular_linear_1_d = np.array([wx_d, wy_d, wz_d, hxd_d, hyd_d, hzd_d]) # Angular Body linear Inertial
        vector = np.array([tx1_d, ty1_d, tz1_d, qw1_d, qx1_d, qy1_d, qz1_d])
        # Update Reference
        X_d[7:13, k] = angular_linear_1_d
        X_d[0:7, k] = vector

    # Set Odom Message
    quat_1_msg = set_odom_msg(quat_1_msg, X1[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)

    quat_2_msg = set_odom_msg(quat_2_msg, X_d[:, 0])
    send_odom_msg(quat_2_msg, odom_pub_2)

    # Loop simulation
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        # Control actions

        #dual_twist_1_d = dual_twist(u[:, k], D1[:, k])
        #xi[:, k] = np.array(dual_twist_1_d).reshape((6, ))

        acados_ocp_solver.set(0, "lbx", X1[:, k])
        acados_ocp_solver.set(0, "ubx", X1[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = X_d[0:7, k+j]
            uref = X_d[7:13, k+j]
            aux_ref = np.hstack((yref, uref))
            acados_ocp_solver.set(j, "p", aux_ref)

        # Desired Trayectory at the last Horizon
        yref_N = X_d[0:7,k+N_prediction]
        uref_N = X_d[7:13,k+N_prediction]
        aux_ref_N = np.hstack((yref_N, uref_N))
        acados_ocp_solver.set(N_prediction, "p", aux_ref_N)

        acados_ocp_solver.solve()

        aux_control = acados_ocp_solver.get(0, "u")
        u[:, k] = aux_control
        xi[:, k] = np.array(rotation_body(X1[3:7, k], u[3:6, k])).reshape((3, ))

        # Acados integration
        acados_integrator.set("x", X1[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral = acados_integrator.solve()
        xcurrent = acados_integrator.get("x")
        X1[:, k+1] = xcurrent

        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic

        # Send msg to Ros
        quat_1_msg = set_odom_msg(quat_1_msg, X1[:, k+1])
        send_odom_msg(quat_1_msg, odom_pub_1)

        quat_2_msg = set_odom_msg(quat_2_msg, X_d[:, k+1])
        send_odom_msg(quat_2_msg, odom_pub_2)

        rospy.loginfo(message_ros + str(toc_solver))

    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, X1[0:3, :], X_d[0:3, :], t, "Position of the System "+ str(initial), folder_path)

    # Orientation
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_states_quaternion(fig12, ax12, ax22, ax32, ax42, X1[3:7, :], X_d[3:7, :], t, "Quaternions of the System "+ str(initial), folder_path)


    # Linear Velocities Inertial frame
    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_states_velocity_reference(fig13, ax13, ax23, ax33, u[3:6, :], hd_d[0:3, :], t, "Linear Velocity of the System and Reference "+ str(initial), folder_path)


    # Angular Velocities Body Frame
    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_states_angular_reference(fig14, ax14, ax24, ax34, u[0:3, :], w_d[0:3, :], t, "Angular Velocity of the System and Reference "+ str(initial), folder_path)

    # Linear Velocities Inertial frame
    fig15, ax15, ax25, ax35 = fancy_plots_3()
    plot_states_velocity_reference(fig15, ax15, ax25, ax35, xi[0:3, :], X_d[10:13, :], t, "Linear Velocity Body of the System and Reference "+ str(initial), folder_path)
    return None

if __name__ == '__main__':
    try: #################################### Simulation  #####################################################
        # Time parameters
        ts = 0.01
        t_f = 10
        t_N = 0.5

        # Parameters of the system  (mass, inertial matrix, gravity)
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Ros Definition
        rospy.init_node("quadrotor",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        k = 0
        main(ts, t_f, t_N, L, odometry_publisher_1, odometry_publisher_2, k)

    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass