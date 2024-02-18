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
from scipy.spatial.transform import Rotation as R

def reference(t, ts):

    # Angular y linear Velocity of the system
    w1 = np.zeros((4, t.shape[0]))
    v1 = np.zeros((4, t.shape[0]))
    v1p = np.zeros((4, t.shape[0]))

    v1[1, :] = -4*0.5*np.sin(0.5*t)
    v1[2, :] = 4*0.5*np.cos(0.5*t)
    v1[3, :] = 0.0
    
    v1p[1, :] = -4*0.5*0.5*np.cos(0.5*t)
    v1p[2, :] = -4*0.5*0.5*np.sin(0.5*t)

    #v1[1, :] = -4*0.5*np.sin(0.5*t)
    #v1[2, :] = 0.5
    #v1[3, :] = 0.0

    #v1p[1, :] = -4*0.5*0.5*np.cos(0.5*t)
    #v1p[2, :] = 0


    # Compute angular displacement
    theta = np.arctan2(v1[2,:], v1[1, :])

    # Compute angular velocity
    theta_p = (1. / ((v1[2, :] / v1[1, :]) ** 2 + 1)) * ((v1p[2, :] * v1[1, :] - v1[2, :] * v1p[1, :]) / v1[1, :] ** 2)
    theta_p[0] = 0

    w1[1, :] = 0.0
    w1[2, :] = 0.0
    w1[3, :] = theta_p

    r = R.from_euler('zyx',[theta[0], 0, 0], degrees=False)
    r_q = r.as_quat()

    # Init Quaternions
    q1 = np.hstack([r_q[3], r_q[0], r_q[1], r_q[2]])
    t1 = np.array([0.0, 4.0, 0.0, 0.0])

    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)

    # Empty matrices
    Q1_data = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    Q1_data[0:4, 0] = Q1.get_quat.get[:, 0]
    Q1_data[4:8, 0] = Q1.get_trans.get[:, 0]

    w1_dual_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)
    v1_dual_data = np.zeros((4, t.shape[0] + 1), dtype=np.double)


    for k in range(0, t.shape[0]):

        # Compute dual velocity
        dual_velocity_values = dual_velocity(w1[:, k], v1[:, k], Q1)
        w1_dual_data[:, k] = dual_velocity_values.get_real.get[:, 0]
        v1_dual_data[:, k] = dual_velocity_values.get_dual.get[:, 0]


        Q1 = f_rk4(Q1, dual_velocity_values, ts)

        # Save information
        Q1_data[0:4, k +1] = Q1.get_quat.get[:, 0]
        Q1_data[4:8, k+1] = Q1.get_trans.get[:, 0]

    return Q1_data, w1_dual_data, v1_dual_data

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

def dual_velocity(w, v, dual):
    w = Quaternion(q = w) 
    v = Quaternion(q = v) 
    p = dual.get_trans
    real = w
    dual = v + w.cross(p)
    dual_velocity = DualQuaternion(q_real = real, q_dual = dual)
    return dual_velocity

def linear_velocity_body(dual_velocity, Q_current):
    # Get Real and dual values 
    real = dual_velocity.get_real
    dual = dual_velocity.get_dual

    # Compute the Linear Velocity
    p = Q_current.get_trans
    quat = Q_current.get_quat
    quat_c = quat.conjugate()
    v = dual - real.cross(p)

    # Transformation to the body frame
    v_body = quat_c * v * quat

    return v_body.get[1:4, 0]

def control_law(qd, q, kp, wd, vd):
    #  Control Error
    qd_quat = qd.get_quat
    qd_quat_c = qd_quat.conjugate()
    q_quat = q.get_quat
    qe_quat = qd_quat_c * q_quat
    qe_quat_c = qe_quat.conjugate()
    p_e = q.get_trans - qe_quat_c * qd.get_trans * qe_quat
    #q_e = DualQuaternion.from_pose(quat = qe_quat.get, trans = p_e.get)

    # Control error complete
    qd_c = qd.conjugate()
    # Calculate left error
    q_e =  qd_c * q

    # Shortest path
    q_e_data = q_e.get
    if q_e_data[0, 0] >= 0.0:
        q_e = 1*q_e
    else:
        q_e = -1*q_e

    # Apply log mapping
    q_e_ln = q_e.ln()

    # Conjugate
    q_e_c = q_e.conjugate()

    #dual_velocity_d = velocity_dual(wd, vd, q)
    dual_velocity_d = DualQuaternion(q_real = Quaternion(q = wd), q_dual = Quaternion(q = vd))
    
    U = -2*q_e_ln.vector_dot_product(kp) + q_e_c * dual_velocity_d * q_e
    return U

def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.03
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")

    # Init Quaternions
    theta1 = 0
    n1 = np.array([0, 0, 1])
    q1 = np.hstack([np.cos(theta1 / 2), np.sin(theta1 / 2) * np.array(n1)])
    t1 = np.array([0.0, 0, 0, 0.0])

    Q2_data, wd, vd = reference(t, sample_time)


    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)

    # Odometry Message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()

    # Message 
    message_ros = "DualQuaternion Casadi "

    # Angular y linear Velocity of the system
    w1 = np.zeros((4, t.shape[0]))
    v1 = np.zeros((4, t.shape[0]))

    # Control gains
    angular_gain = np.hstack([0.0, 1, 1, 1])
    trans_gain = np.array([0, 1.5, 1.5, 1.5])


    K = DualQuaternion(q_real = Quaternion(q = angular_gain), q_dual = Quaternion(q = trans_gain))

    # Empty matrices
    Q1_data = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    Q1_data[0:4, 0] = Q1.get_quat.get[:, 0]
    Q1_data[4:8, 0] = Q1.get_trans.get[:, 0]

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        Q2 = DualQuaternion.from_pose(quat = Q2_data[0:4, k], trans = Q2_data[4:8, k])

        # COntrol Law
        U = control_law(Q2, Q1, K, wd[:, k], vd[:, k])

        # Save Values Control Law
        w1[1:4, k] = U.get_real.get[1:4,0] 
        v1[1:4, k] =  linear_velocity_body(U, Q1)

        # Compute dual velocity
        dual_velocity_values = dual_velocity(w1[:, k], v1[:, k], Q1)

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, Q1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, Q2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)


        Q1 = f_rk4(Q1, U, sample_time)

        # Save information
        Q1_data[0:4, k +1] = Q1.get_quat.get[:, 0]
        Q1_data[4:8, k+1] = Q1.get_trans.get[:, 0]

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

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