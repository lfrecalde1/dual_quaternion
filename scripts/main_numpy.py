#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import plot_states_quaternion, fancy_plots_4, fancy_plots_1, plot_norm_quat, plot_angular_velocities, fancy_plots_3 
from nav_msgs.msg import Odometry

def get_odometry(odom_msg, dqd, name):
    # Function to send the Oritentation of the Quaternion
    q_d = dqd.get
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = name
    odom_msg.pose.pose.position.x = 0
    odom_msg.pose.pose.position.y = 0
    odom_msg.pose.pose.position.z = 0

    odom_msg.pose.pose.orientation.x = q_d[1]
    odom_msg.pose.pose.orientation.y = q_d[2]
    odom_msg.pose.pose.orientation.z = q_d[3]
    odom_msg.pose.pose.orientation.w = q_d[0]
    return odom_msg

def send_odometry(odom_msg, odom_pub):
    # Function to send the orientation of the Quaternion
    odom_pub.publish(odom_msg)
    return None

def quatdot(quat, omega):
    # Quaternion evolution guaranteeing norm 1 (Improve this section)
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    quat_data = quat.get
    qw = quat_data[0, 0]
    qx = quat_data[1, 0]
    qy = quat_data[2, 0]
    qz = quat_data[3, 0]

    # Auxiliary variable in order to avoid numerical issues
    K_quat = 2
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    q_dot = (1/2)*(quat*omega) + K_quat*quat_error*quat
    return q_dot

def f_rk4(quat, omega, ts):
    # Function that computes 
    k1 = quatdot(quat, omega)
    k2 = quatdot(quat + (1/2)*ts*k1, omega)
    k3 = quatdot(quat + (1/2)*ts*k2, omega)
    k4 = quatdot(quat + (1)*ts*k3, omega)
    # Compute forward Euler method
    quat = quat + (1/6)*ts*(k1 + 2*k2 + 2*k3 + k4)
    return quat

def reference(t, ts):
    # Desired Quaternion
    theta_2 = -np.pi/4
    n_2 = np.array([1.0, 0.0, 0.0])
    q2 = np.hstack([np.cos(theta_2 / 2), np.sin(theta_2 / 2) * np.array(n_2)])
    quat_2 = Quaternion(q = q2)

    Qd = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Qd[:, 0] = quat_2.get[:, 0]

    Wd = np.zeros((4, t.shape[0]), dtype=np.double)
    Wd[1, :] = 0.1
    Wd[2, :] = 1*np.cos(0.5*t)
    Wd[3, :] = 2*np.cos(1*t)

    qw = Quaternion(q = Wd[:, 0])
    for k in range(0, t.shape[0]):
        # Save Control Actions
        qw.set(q = Wd[:, k])

        # System Evolution
        quat_2 = f_rk4(quat_2, qw, ts)

        # Save States of the system
        Qd[:, k + 1] = quat_2.get[:,0]
    return Qd, Wd

def control_law(qd, q, kp, w):
    qd_c = qd.conjugate()
    # Calculate left error
    q_e =  qd_c * q

    # Shortest path
    q_e_data = q_e.get
    if q_e_data[0, 0] >= 0.0:
        q_e = -1*q_e
    else:
        q_e = 1*q_e

    # Conjugate
    q_e_c = q_e.conjugate()
    # Apply log mapping
    q_e_ln = q_e.ln()
    
    U = q_e_ln.vector_dot_product(kp) + q_e_c * w * q_e
    return U, q_e_ln

def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.01
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternions
    theta = np.pi/4
    n = np.array([0.0, 0.0, 1.0])
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    q2, w2 = reference(t, sample_time)

    # Create Quaternions objects
    quat_1 = Quaternion(q = q1)
    quat_2 = Quaternion(q = q2[:, 0])
    quat_w = Quaternion(q = w2[:, 0])

    # Empty Messages
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()

    # Angular Velocities  quaternion 1
    w1 = np.zeros((4, t.shape[0]), dtype=np.double)

    # Empty matrices
    Q1 = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q1[:, 0] = quat_1.get[:,0]
    Q2 = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    Q2[:, 0] = quat_2.get[:, 0]
    Qnorm = np.zeros((1, t.shape[0] + 1), dtype=np.double)

    # Control Gain
    kp = Quaternion(q = np.array([0.0, 1.5, 1.5, 1.5]))

    # Message 
    message_ros = "Quaternion Casadi "

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Update Reference
        quat_2.set(q = q2[:, k])
        quat_w.set(q = w2[:, k])

        # Control Law
        U, q_e_ln = control_law(quat_2, quat_1, kp, quat_w)

        # Save Control Actions
        w1[:, k] = U.get[:, 0]
        Qnorm[:, k] = q_e_ln.norm


        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, quat_1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, quat_2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)


        # System Evolution
        quat_1 = f_rk4(quat_1, U, sample_time)

        # Save States of the syst
        Q1[:, k + 1] = quat_1.get[:,0]
        Q2[:, k + 1] = quat_2.get[:,0]

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    # Get SX information
    #Q1 = ca.DM(Q1)
    #Q1 = np.array(Q1)

    #Q2 = ca.DM(Q2)
    #Q2 = np.array(Q2)

    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, Q1[:, :], Q2[:, :], t, "Quaternions Results")
    plt.show()

    fig12, ax12 = fancy_plots_1()
    plot_norm_quat(fig12, ax12, Qnorm, t, "Quaternion Error Norm")
    plt.show()

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_angular_velocities(fig13, ax13, ax23, ax33, w1[1:4, :], t, "Angular velocities")
    plt.show()

    # 
    # Symbolic quaternion using Casadi
    #theta_c = ca.MX.sym('theta_c', 1)
    #n_c = ca.MX.sym('n_c', 3, 1)
    #theta1_c = ca.SX([3.8134])
    #n1_c = ca.SX([0.4896, 0.2032, 0.8480])
    #q1_c = ca.vertcat(ca.cos(theta1_c/2), ca.sin(theta1_c/2)@n1_c)

    #theta2_c = ca.SX([ca.pi/2])
    #n2_c = ca.SX([0.0, 0.0, 1.0])
    #q2_c = ca.vertcat(ca.cos(theta2_c/2), ca.sin(theta2_c/2)@n2_c)
    #q2_c = ca.vertcat(ca.cos(theta2_c/2), ca.sin(theta2_c/2)@n2_c)
    return None
if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Quaternions", disable_signals=True, anonymous=True)
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