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
    # Calculate left error
    q_e = q.left_error(qd, lambda_aux = lambda_value)

    # Apply log mapping
    q_e_ln = q_e.ln_quat()

    # Conjugate left error
    q_e_c = q_e.conjugate()

    Ad_qec = q_e_c.__adj__(wd)
    
    U = q_e_ln.vector_dot_product(kp)
    return -2*U.get()


def main():

    # Sample Time Defintion
    sample_time = 0.05
    t_f = 10

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternion Axis angle
    theta = 3.81
    n = np.array([0.4896, 0.2032, 0.8480])

    # Desired quaternion
    theta_d = np.pi/2
    n_d = np.array([0.0, 0.0, 1.0])

    # Initial quaternion
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    # Desired quaternion
    qd = np.hstack([np.cos(theta_d / 2), np.sin(theta_d / 2) * np.array(n_d)])

    # Object quaternion
    q1 = Quaternion(q = q1, name = "quat_1")
    qd = Quaternion(q = qd, name = "quat_d")
    kp = Quaternion(q = [0.0, 1, 1, 1])

    # Empty vector where the data is going to be stored
    Q1 = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Q1[:, 0] = q1.get()

    Qd = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Qd[:, 0] = qd.get()
    # Message 
    message_ros = "Quaternion "
    # Empty vector Norm of the error between quaternions
    Q_error_norm = np.zeros((1, t.shape[0]+1), dtype=np.double)

    U = np.zeros((3, t.shape[0]), dtype=np.double)
    
    # Simulation loop
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Calculate the error and the norm
        q_e = q1.left_error(qd, lambda_aux = lambda_value)
        q_e_ln = q_e.ln_quat()
        Q_error_norm[:, k] = q_e_ln.norm()

        # Control Action
        u = control_quat(qd = qd, q = q1, wd = [0.0, 0.0, 0.0, 0.0], kp = kp)
        U[:, k] = u[1:4]

        
        # Time restriction Correct
        loop_rate.sleep()

        # System evolution
        q1.__ode__(u, sample_time)

        # Send Data
        q1.send_odometry()
        qd.send_odometry()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

        # Save Data
        Q1[:, k+1] = q1.get()
        Qd[:, k+1] = qd.get()

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