#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from ode_acados import compute_flatness_states
from fancy_plots import fancy_plots_3, plot_states_position, fancy_plots_4, plot_states_velocity_reference, plot_states_quaternion

import os
script_dir = os.path.dirname(__file__)
#folder_path = os.path.join(script_dir, 'cost_with_velocities/')
folder_path = os.path.join(script_dir, 'cost_without_velocities/')
# Check if the folder exists, and create it if it doesn't

# Init global variables
x_init = 0.0
y_init = 0.0
z_init = 0.0

# Angular velocities
qx_init = 0.0
qy_init = 0.0
qz_init = 0.0
qw_init = 1.0

def odom_callback(odom_msg):
    # Get initial States of the system in order to compute the desired trajectory
    global x_init, y_init, z_init, qx_init, qy_init, qz_init, qw_init

    x_init = odom_msg.pose.pose.position.x
    y_init = odom_msg.pose.pose.position.y
    z_init = odom_msg.pose.pose.position.z

    qx_init = odom_msg.pose.pose.orientation.x
    qy_init = odom_msg.pose.pose.orientation.y
    qz_init = odom_msg.pose.pose.orientation.z
    qw_init = odom_msg.pose.pose.orientation.w

def init_states():
    x = np.array([x_init, y_init, z_init], dtype=np.double)
    q = np.array([qw_init, qx_init, qy_init, qz_init], dtype=np.double)
    return x, q

def main(L):
    initial = 0
    # Split Values
    m = L[0]
    g = L[4]

    # Sample Time Defintion
    sample_time = 0.01

    ## Define time for each segment
    t_inital = 2
    t_trajectory = 30
    t_final = 2


    zi = 2.0
    w_c = 4


    # Time defintion aux variable
    t_stable = np.arange(0, 2 + sample_time, sample_time)

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    rospy.loginfo_once("Planning.....")
    message_ros = "PLanning "
    x, q = init_states()

    # Init system get appropiate values for the initial state of the drone
    for k in range(0, t_stable.shape[0]):
        tic = rospy.get_time()
        x, q = init_states()

        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic
        delta_t = toc_solver
        rospy.loginfo("Init System " + str(delta_t))

    h, h_d, q, w, f, M, t = compute_flatness_states(L, x, t_inital, t_trajectory, t_final, sample_time, zi, w_c)

    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, h[0:3, :], h[0:3, :], t, "Position of the System "+ str(initial), folder_path)

    fig12, ax12, ax22, ax32 = fancy_plots_3()
    plot_states_velocity_reference(fig12, ax12, ax22, ax32, h_d[0:3, :], h_d[0:3, :], t, "Linear Velocity of the System and Reference "+ str(initial), folder_path)

     # Quaternions Body Frame
    fig13, ax13, ax23, ax33, ax43 = fancy_plots_4()
    plot_states_quaternion(fig13, ax13, ax23, ax33, ax43, q, q, t, "Quaternions of the System "+ str(initial), folder_path)

    

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Planning", disable_signals=True, anonymous=True)

        # Dynamics Parameters
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Node subscription
        rospy.Subscriber('/dual_1/odom', Odometry, odom_callback)

        main(L)

        #savemat("Dual_cost_without_velocities" + ".mat", mdic_x)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass