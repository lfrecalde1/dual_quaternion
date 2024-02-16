#!/usr/bin/env python
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from dual_quaternion import plot_states_quaternion, fancy_plots_4, fancy_plots_1, plot_norm_quat, plot_angular_velocities, fancy_plots_3 
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

    # TODo There is s bug with the multiplication with quaternions and scalar
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


def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.05
    t_f = 10

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")

    # Init Quaternions
    theta1 = np.pi/2
    n1 = np.array([0.0, 1.0, 0.0])
    q1 = np.hstack([np.cos(theta1 / 2), np.sin(theta1 / 2) * np.array(n1)])
    t1 = np.array([0.0, 1, 1, 0.0])

    theta2 = np.pi/2
    n2 = np.array([1.0, 0.0, 0.0])
    q2 = np.hstack([np.cos(theta2 / 2), np.sin(theta2 / 2) * np.array(n2)])
    t2 = np.array([0, 0, 1, 0])

    theta3 = np.pi/16
    n3 = np.array([0.0, 0.0, 1.0])
    q3 = np.hstack([np.cos(theta3 / 2), np.sin(theta3 / 2) * np.array(n3)])
    t3 = np.array([0, 2.0, 0.0, -1])

    # Defining of the vectors using casadi
    #theta1 = ca.SX([ca.pi/2])
    #n1 = ca.SX([0.0, 1.0, 0.0])
    #q1 = ca.vertcat(ca.cos(theta1/2), ca.sin(theta1/2)@n1)
    #t1 = ca.SX([0.0, 1.0, 1.0, 0.0])

    #theta2 = ca.SX([ca.pi/2])
    #n2 = ca.SX([1.0, 0.0, 0.0])
    #q2 = ca.vertcat(ca.cos(theta2/2), ca.sin(theta2/2)@n2)
    #t2 = ca.SX([0.0, 0.0, 1.0, 0.0])

    #theta3 = ca.SX([ca.pi/16])
    #n3 = ca.SX([0.0, 0.0, 1.0])
    #q3 = ca.vertcat(ca.cos(theta3/2), ca.sin(theta3/2)@n3)
    #t3 = ca.SX([0.0, 2.0, 0.0, -1.0])

    # Create Quaternions objects
    #quat_1_r = Quaternion(q = q1)
    #quat_1_t = Quaternion(q = t1)

    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)
    Q2 = DualQuaternion.from_pose(quat = q2, trans = t2)
    Q3 = DualQuaternion.from_pose(quat = q3, trans = t3)
    #Q1 = DualQuaternion(q_real = quat_1_r, q_dual = quat_1_t)
    #alpha = ca.SX([0.2])
    alpha = 0.2

    # Odometry Message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()
    # Message 
    message_ros = "DualQuaternion Casadi "

    # Angular y linear Velocity of the system
    w1 = np.zeros((4, t.shape[0]))
    w1[2, :] = 0.1
    w1[1, :] = 0.5
    v1 = np.zeros((4, t.shape[0]))
    v1[2, :] = 0.1

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        dual_velocity = velocity_dual(w1[:, k], v1[:, k], Q1)

        # Update Reference
        Q4 =  Q1 * Q2
        
        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, Q1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        #quat_2_msg = get_odometry(quat_2_msg, Q2, 'quat_2')
        #send_odometry(quat_2_msg, odom_pub_2)


        Q1 = f_rk4(Q1, dual_velocity, sample_time)
        print(Q1.norm)
        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))
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