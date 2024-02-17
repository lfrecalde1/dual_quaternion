#!/usr/bin/env python
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, fancy_plots_1, plot_norm_quat, plot_angular_velocities, plot_linear_velocities, fancy_plots_3 
from nav_msgs.msg import Odometry

def get_odometry(odom_msg, dqd, name):
    # Function to send the Oritentation of the Quaternion
    # Get Information from the DualQuaternion
    t_d = dqd.get_trans
    t_d_data = t_d.get

    q_d = dqd.get_quat
    q_d_data = q_d.get

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = name
    odom_msg.pose.pose.position.x = t_d_data[1]
    odom_msg.pose.pose.position.y = t_d_data[2]
    odom_msg.pose.pose.position.z = t_d_data[3]

    odom_msg.pose.pose.orientation.x = q_d_data[1]
    odom_msg.pose.pose.orientation.y = q_d_data[2]
    odom_msg.pose.pose.orientation.z = q_d_data[3]
    odom_msg.pose.pose.orientation.w = q_d_data[0]
    return odom_msg

def send_odometry(odom_msg, odom_pub):
    # Function to send the orientation of the Quaternion
    odom_pub.publish(odom_msg)
    return None


def quatdot(quat, omega):
    quat_data = quat.get_real
    dual_data =  quat.get_dual

    # Auxiliary variable in order to avoid numerical issues
    norm_r, norm_d = quat.norm
    K_quat = 2
    quat_error = 1 - norm_r
    dual_error = norm_d

    aux_1 = quat_data * (K_quat*quat_error)
    aux_2 = dual_data * (K_quat*dual_error)

    aux_dual = DualQuaternion(q_real = aux_1, q_dual = aux_2)

    q_dot = (1/2)*(quat*omega) + aux_dual
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

def velocity_dual(w, v, dual):
    w = Quaternion(q = w) 
    v = Quaternion(q = v) 
    p = dual.get_trans
    real = w
    dual = v + w.cross(p)
    dual_velocity = DualQuaternion(q_real = real, q_dual = dual)
    return dual_velocity


def control_law(qd, q, kp):
    #  Control Error
    qd_quat = qd.get_quat
    qd_quat_c = qd_quat.conjugate()
    q_quat = q.get_quat
    qe_quat = qd_quat_c * q_quat
    qe_quat_c = qe_quat.conjugate()
    p_e = q.get_trans - qe_quat_c * qd.get_trans * qe_quat
    q_e = DualQuaternion.from_pose(quat = qe_quat.get, trans = p_e.get)

    # Control error complete
    qd_c = qd.conjugate()
    # Calculate left error
    #q_e =  qd_c * q

    # Shortest path
    q_e_data = q_e.get
    if q_e_data[0, 0] >= 0.0:
        q_e = 1*q_e
    else:
        q_e = -1*q_e

    # Apply log mapping
    q_e_ln = q_e.ln()
    
    U = -2*q_e_ln.vector_dot_product(kp)
    return U

def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.05
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")


    # Defining of the vectors using casadi
    theta1 = ca.SX([3.8134])
    n1 = ca.SX([0.4896, 0.2032, 0.8480])
    q1 = ca.vertcat(ca.cos(theta1/2), ca.sin(theta1/2)@n1)
    t1 = ca.SX([0.0, 2.0, 2.0, 1.0])

    theta2 = ca.SX([0.0])
    n2 = ca.SX([0.0, 0.0, 1.0])
    q2 = ca.vertcat(ca.cos(theta2/2), ca.sin(theta2/2)@n2)
    t2 = ca.SX([0.0, 0.0, 0.0, 0.0])

    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)
    Q2 = DualQuaternion.from_pose(quat = q2, trans = t2)

    # Odometry Message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()
    # Message 
    message_ros = "DualQuaternion Casadi "

    # Angular y linear Velocity of the system
    w1 = ca.SX.zeros(4, t.shape[0])
    v1 = ca.SX.zeros(4, t.shape[0])

    # Control gains
    angular_gain = ca.SX([0.0, 1.0, 1.0, 1.0])
    trans_gain = ca.SX([0.0, 1.0, 1.0, 1.0])

    K = DualQuaternion(q_real = Quaternion(q = angular_gain), q_dual = Quaternion(q = trans_gain))

    # Empty matrices
    Q1_data = ca.SX.zeros(8, t.shape[0] + 1)
    Q1_data[0:4, 0] = Q1.get_quat.get[:, 0]
    Q1_data[4:8, 0] = Q1.get_trans.get[:, 0]

    Q2_data = ca.SX.zeros(8, t.shape[0] + 1)
    Q2_data[0:4, 0] = Q2.get_quat.get[:, 0]
    Q2_data[4:8, 0] = Q2.get_trans.get[:, 0]

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()

        # COntrol Law
        U = control_law(Q2, Q1, K)

        # Save Values Control Law
        w1[1:4, k] = U.get_real.get[1:4,0] 
        v1[1:4, k] = U.get_dual.get[1:4,0] 

        # Compute dual velocity
        dual_velocity = velocity_dual(w1[:, k], v1[:, k], Q1)

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, Q1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, Q2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)


        Q1 = f_rk4(Q1, U, sample_time)

        # Save information
        Q1_data[0:4, k +1] = Q1.get_quat.get[:, 0]
        Q1_data[4:8, k+1] = Q1.get_trans.get[:, 0]

        Q2_data[0:4, k+1] = Q2.get_quat.get[:, 0]
        Q2_data[4:8, k+1] = Q2.get_trans.get[:, 0]
        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    Q1_data = ca.DM(Q1_data)
    Q1_data = np.array(Q1_data)

    Q2_data = ca.DM(Q2_data)
    Q2_data = np.array(Q2_data)

    v1 = ca.DM(v1)
    v1 = np.array(v1)

    w1 = ca.DM(w1)
    w1 = np.array(w1)

    # PLot Results
    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, Q1_data[0:4, :], Q2_data[0:4, :], t, "Quaternions Results")
    plt.show()

    fig12, ax12, ax22, ax32 = fancy_plots_3()
    plot_states_position(fig12, ax12, ax22, ax32, Q1_data[5:8, :], Q2_data[5:8, :], t, "Position of the System")
    plt.show()

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_angular_velocities(fig13, ax13, ax23, ax33, w1[1:4, :], t, "Angular velocities")
    plt.show()

    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_linear_velocities(fig14, ax14, ax24, ax34, v1[1:4, :], t, "Linear velocities")
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