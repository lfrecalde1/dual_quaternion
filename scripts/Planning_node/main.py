#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from ode_acados import trajectory, compute_b, position_time, velocity_time, acceleration_time, jerk_time, snap_time, A_matrix, H_matrix
from ode_acados import trajectory_3d, trajectory_3d_init_phase, minimum_snap_planning, compute_flatness_states
from fancy_plots import fancy_plots_3, fancy_plots_4, fancy_plots_1
from fancy_plots import plot_states_position, plot_states_velocity_reference, plot_states_acceleration_reference, plot_states_quaternion
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
    t_inital = np.array([2])
    t_trajectory = np.array([10])
    t_final = np.array([2])

    ##  Compute array with time
    traj_flight_time = np.array([t_inital[0], t_trajectory[0], t_final[0]], dtype=np.double)
    print(traj_flight_time)

    ## Init and final of the trajectory
    zi = 2
    w_c = 2
    r_init, r_d_init, r_dd_init, r_ddd_init, r_dddd_init, _, _, _ = trajectory(t_inital, zi, w_c)
    r_final, r_d_final, r_dd_final, r_ddd_final, r_dddd_final, _, _, _ = trajectory(t_trajectory, zi, w_c)

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

    # Compute PLanning Section Manually
    waypoints_1 = np.array([x[0], r_init[0, 0], r_final[0, 0], x[0]])
    traj_size = waypoints_1.shape[0] - 1

    # Number Point over trajectory and polynomials
    number_points = 1/sample_time
    number_polynomial = 9
    number_coeff = number_polynomial + 1

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