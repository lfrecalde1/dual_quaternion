#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, fancy_plots_1, plot_norm_quat, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_norm_real, plot_norm_dual, plot_lyapunov, plot_lyapunov_dot
from scipy.spatial.transform import Rotation as R
from scipy import sparse
from nav_msgs.msg import Odometry
from functions import dualquat_from_pose_casadi, dualquat_trans_casadi, dualquat_quat_casadi
from functions import f_rk4_casadi_simple, dual_velocity_casadi, dual_velocity_body_casadi, velocities_body_from_twist_casadi
from functions import dual_quat_casadi, dual_control_casadi, lyapunov_casadi_simple, lyapunov_dot_casadi
from functions import jacobian_casadi, system_evolution_casadi, optimization_casadi
import osqp

# Creating Funtions based on Casadi
dual_quat = dual_quat_casadi()
dualquat_from_pose = dualquat_from_pose_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
f_rk4 = f_rk4_casadi_simple()
dual_twist = dual_velocity_casadi()
velocity_body = dual_velocity_body_casadi()
velocity_body_from_twist = velocities_body_from_twist_casadi()
kinematics_control = dual_control_casadi()
lyapunov_lie = lyapunov_casadi_simple()
lyapunov_dot_lie = lyapunov_dot_casadi()
jacobian_dual = jacobian_casadi()

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

def optimization_solver(state_target, state_init, X0, u0, solver, args, n_states, n_controls, N):

    args['p'] = ca.vertcat(
            state_init,    # current state
            state_target   # target state
        )
        # optimization variable current state
    args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )

    sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

    u0 = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
    X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)
    return u0, X0
    
def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.01
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    N = 3

    # Function of the prediction behavior
    system_prediction = system_evolution_casadi(t.shape[0], sample_time)
    solver, args = optimization_casadi(N, sample_time)
    
    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")
    message_ros = "DualQuaternion Casadi NMPC "

    # Odometry Message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()
    
    kp = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) 

    # Defining initial condition of the system and verify properties
    theta1 = 3.8134
    nx = 0.4896
    ny = 0.2032
    nz = 0.8480 
    tx1 = 10
    ty1 = 10
    tz1 = 5.0


    # Initial Dualquaternion
    dual_1 = dualquat_from_pose(theta1, nx, ny,  nz, tx1, ty1, tz1)

    # Defining the desired dualquaternion
    theta1_d = 0.0
    nx_d = 0.0
    ny_d = 0.0
    nz_d = 1.0
    tx1_d = -0.0
    ty1_d = 0.0
    tz1_d = 0.0

    # Initial Dualquaternion
    dual_2 = dualquat_from_pose(theta1_d, nx_d, ny_d,  nz_d, tx1_d, ty1_d, tz1_d)

    # Empty Matrix
    Q1_trans_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q1_quat_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q1_data = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    # Initial Values
    Q1_trans_data[:, 0] = np.array(get_trans(dual_1)).reshape((4, ))
    Q1_quat_data[:, 0] = np.array(get_quat(dual_1)).reshape((4, ))
    Q1_data[:, 0] = np.array(dual_1).reshape((8, ))

    Velocities_data = np.zeros((6, t.shape[0]), dtype=np.double)
    twist_velocities_data = np.zeros((6, t.shape[0]), dtype=np.double)

    # Empty Matrix
    Q2_trans_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q2_quat_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q2_data = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    # Initial Values
    Q2_trans_data[:, 0] = np.array(get_trans(dual_2)).reshape((4, ))
    Q2_quat_data[:, 0] = np.array(get_quat(dual_2)).reshape((4, ))
    Q2_data[:, 0] = np.array(dual_2).reshape((8, ))

    # Empty Values Lyapunov
    lyapunov_values = np.zeros((1, t.shape[0]), dtype=np.double)
    lyapunov_dot_values = np.zeros((1, t.shape[0]), dtype=np.double)


    quat_1_msg = get_odometry(quat_1_msg, dual_1, 'quat_1')
    send_odometry(quat_1_msg, odom_pub_1)

    quat_2_msg = get_odometry(quat_2_msg, dual_2, 'quat_2')
    send_odometry(quat_2_msg, odom_pub_2)

    # Init Optimization variables
    n_controls = 6
    n_states = 8
    U0 = ca.DM.ones((n_controls, N))  # initial control
    X0 = ca.repmat(dual_1, 1, N+1)         # initial state full
    U0, X0 = optimization_solver(dual_2, dual_1, X0, U0, solver, args, n_states, n_controls, N)


    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Compute Lyapunov Lie 
        lyapunov_values[0, k] = lyapunov_lie(dual_2, dual_1)
        # desired Linear inertial And angular body velocities reference quaternion
        angular_linear_2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Angular Body linear Inertial
        dual_twist_2 = dual_twist(angular_linear_2, dual_2)

        # Compute control Law
        U0, X0 = optimization_solver(dual_2, dual_1, X0, U0, solver, args, n_states, n_controls, N)

        #control_values = kinematics_control(dual_2, dual_twist_2, dual_1, kp)
        #control_values_aux = ca.vertcat(control_values[1, 0], control_values[2, 0], control_values[3, 0], control_values[5, 0], control_values[6, 0], control_values[7, 0])
        control_values_aux_full = ca.vertcat(0.0, U0[0, 0], U0[1, 0], U0[2, 0], 0.0, U0[3, 0], U0[4, 0], U0[5, 0])
        control_values_aux = U0[:, 0]

        # Compute time derivative Lyapunov
        Jacobian_system = jacobian_dual(dual_1)
        lyapunov_dot_values[0, k] = lyapunov_dot_lie(dual_2, dual_1)@Jacobian_system@control_values_aux_full


        # Evolve system
        body_velocities = velocity_body_from_twist(control_values_aux, dual_1)
        # Save Control velocities
        Velocities_data[:, k] = np.array(body_velocities).reshape((6, ))
        twist_velocities_data[:, k] = np.array(control_values_aux).reshape((6, ))

        # Evolve system
        dual_1 = f_rk4(dual_1, control_values_aux, sample_time)

        # Evolve desired Dualquaternion
        dual_twist_2 = dual_twist(angular_linear_2, dual_2)
        dual_2 = f_rk4(dual_2, dual_twist_2, sample_time)

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, dual_1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, dual_2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)

        # Update Matrices
        Q1_data[:, k + 1] = np.array(dual_1).reshape((8, ))
        Q1_trans_data[:, k + 1] = np.array(get_trans(dual_1)).reshape((4, ))
        Q1_quat_data[:, k + 1] = np.array(get_quat(dual_1)).reshape((4, ))

        Q2_data[:, k + 1] = np.array(dual_2).reshape((8, ))
        Q2_trans_data[:, k + 1] = np.array(get_trans(dual_2)).reshape((4, ))
        Q2_quat_data[:, k + 1] = np.array(get_quat(dual_2)).reshape((4, ))

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    # Compute trajectories based on initial conditions 
    tic = rospy.get_time()
    aux_trajectory = system_prediction(Q1_data[:, 0], twist_velocities_data)
    aux_trajectory = np.array(aux_trajectory)
    toc = rospy.get_time()
    delta = toc - tic
    print(delta)

    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, Q1_data[0:4, :], Q2_data[0:4, :], t, "Quaternion Results")
    plt.show()


    fig12, ax12, ax22, ax32 = fancy_plots_3()
    plot_states_position(fig12, ax12, ax22, ax32, Q1_trans_data[1:4, :], Q2_trans_data[1:4, :], t, "Position of the System")
    plt.show()
    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_angular_velocities(fig13, ax13, ax23, ax33, Velocities_data[0:3, :], t, "Angular velocities Body")
    plt.show()

    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_linear_velocities(fig14, ax14, ax24, ax34, Velocities_data[3:6, :], t, "Linear velocities Body")
    plt.show()

    fig15, ax15 = fancy_plots_1()
    plot_lyapunov(fig15, ax15, lyapunov_values, t, "lyapunnov")
    plt.show()

    fig16, ax16 = fancy_plots_1()
    plot_lyapunov_dot(fig16, ax16, lyapunov_dot_values, t, "lyapunnov dot")
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
        main(odometry_publisher_1, odometry_publisher_2)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass