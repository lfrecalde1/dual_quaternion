#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion.quaternion import Quaternion
from dual_quaternion.fancy_plots import plot_states_quaternion, fancy_plots_4, fancy_plots_1, plot_norm_quat
from dual_quaternion.fancy_plots import plot_angular_velocities, fancy_plots_3

global lambda_value

def control_quat(qd, q, wd, kp):
    qd_c = qd.conjugate()
    # Calculate left error
    q_e = q.right_error(qd_c, lambda_aux = lambda_value)

    # Apply log mapping
    q_e_ln = q_e.ln_quat()
    Ad_qe = q_e.__adj__(wd)
    
    U = q_e_ln.vector_dot_product(kp)
    return -2*U.get() + Ad_qe.get()

def reference(t, ts):
    # Desired quaternion
    theta_d = -np.pi/4
    n_d = np.array([1.0, 0.0, 0.0])

    # Desired quaternion
    qd = np.hstack([np.cos(theta_d / 2), np.sin(theta_d / 2) * np.array(n_d)])

    # Object quaternion
    qd = Quaternion(q = qd, name = "quat_d")
    # Empty vector
    Qd = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Qd[:, 0] = qd.get()

    Wd = np.zeros((4, t.shape[0]), dtype=np.double)
    Wd[1, :] = 0.1
    Wd[2, :] = 1*np.cos(0.5*t)
    Wd[3, :] = 2*np.cos(1*t)

    Wd_q = np.zeros((4, t.shape[0]), dtype=np.double)


    for k in range(0, t.shape[0]):
        Wd_qaux = qd.__adj__(Wd[:,k])
        Wd_q[:, k] = Wd_qaux.get()
        qd.__ode__(Wd[:, k], ts)
        Qd[:, k+1] = qd.get()
    return Qd, Wd

def main():
    # Sample Time Defintion
    sample_time = 0.05
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternion Axis angle
    theta = np.pi/4
    n = np.array([0.0, 0.0, 1.0])

    # Initial quaternion
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])

    # Object quaternion
    q1 = Quaternion(q = q1, name = "quat_1")

    # Control Gains
    kp = Quaternion(q = [0.0, 1.5, 1.5, 1.5])

    # Empty vector where the data is going to be stored
    Q1 = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Q1[:, 0] = q1.get()

    # Message 
    message_ros = "Quaternion "

    # Empty vector Norm of the error between quaternions
    Q_error_norm = np.zeros((1, t.shape[0]+1), dtype=np.double)

    # Empty vector of control actions
    U = np.zeros((3, t.shape[0]), dtype=np.double)

    # Send Initial Odometry 
    q1.send_odometry()

    # get Reference Signal
    Qd, Wd = reference(t, sample_time)
    qd = Quaternion(q = Qd[:, 0], name = "quat_d")
    wd = Quaternion(q = Wd[:, 0])

    # Simulation loop
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Update Quaternions
        qd.set(q = Qd[:, k])
        wd.set(q = Wd[:, k])

        # Calculate the error and the norm
        qd_c = qd.conjugate()
        q_e = q1.right_error(qd_c, lambda_aux = lambda_value)
        q_e_ln = q_e.ln_quat()
        Q_error_norm[:, k] = q_e_ln.norm()

        # Control Action
        u = control_quat(qd = qd, q = q1, wd = wd, kp = kp)
        U[:, k] = u[1:4]

        # System evolution
        q1.__ode__(u, sample_time)

        # Send Data
        qd.send_odometry()
        q1.send_odometry()

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

        # Save Data
        Q1[:, k+1] = q1.get()

    # Reshape Data
    # Orientation
    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, Q1[:, :], Qd[:, :], t, "Quaternions Results")
    plt.show()

    fig12, ax12 = fancy_plots_1()
    plot_norm_quat(fig12, ax12, Q_error_norm, t, "Quaternion Error Norm")
    plt.show()

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_angular_velocities(fig13, ax13, ax23, ax33, U, t, "Angular velocities")
    plt.show()
    return None

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Quaternions", disable_signals=True, anonymous=True)
        lambda_value = rospy.get_param('~lambda_value', -1.0)  # Default to None if not provided
        if lambda_value == -1.0:
            lambda_value = None
        else:
            lambda_value = lambda_value
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass