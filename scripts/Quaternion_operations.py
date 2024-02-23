#!/usr/bin/env python
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

    theta1 = ca.SX([ca.pi/4])
    n1 = ca.SX([0.0, 0.0, 1.0])
    q1 = ca.vertcat(ca.cos(theta1/2), ca.sin(theta1/2)@n1)
    q1 = ca.SX([5, 28, 86, 99])

    theta2 = ca.SX([ca.pi/4])
    n2 = ca.SX([0.0, 0.0, 1.0])
    q2 = ca.vertcat(ca.cos(theta2/2), ca.sin(theta2/2)@n2)
    q2 = ca.SX([-2, 0, 0, 25])

    # Create Quaternions objects
    quat_1 = Quaternion(q = q1)
    quat_2 = Quaternion(q = q2)

    # Empty Messages
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()


    # Empty matrices
    Q1 = ca.SX.zeros(4, t.shape[0] + 1)
    Q1[:, 0] = quat_1.get[:,0]

    Q2 = ca.SX.zeros(4, t.shape[0] + 1)
    Q2[:, 0] = quat_2.get[:, 0]


    # Message 
    message_ros = "Quaternion Casadi "

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        quat_3 = quat_2 * quat_1
        print(quat_1)
        print("-------------------")
        print(quat_3)


        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, quat_1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, quat_2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)



        # Save States of the syst
        Q1[:, k + 1] = quat_1.get[:,0]
        Q2[:, k + 1] = quat_2.get[:,0]

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