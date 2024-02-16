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


def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.1
    t_f = 5

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternions
    theta1 = 0.0
    n1 = np.array([0.0, 0.0, 1.0])
    q1 = np.hstack([np.cos(theta1 / 2), np.sin(theta1 / 2) * np.array(n1)])
    t1 = np.array([0.0, 1, 2, 3])

    theta2 = np.pi/2
    n2 = np.array([1.0, 0.0, 0.0])
    q2 = np.hstack([np.cos(theta2 / 2), np.sin(theta2 / 2) * np.array(n2)])
    t2 = np.array([0, 0, 1, 0])

    theta3 = np.pi/16
    n3 = np.array([0.0, 0.0, 1.0])
    q3 = np.hstack([np.cos(theta3 / 2), np.sin(theta3 / 2) * np.array(n3)])
    t3 = np.array([0, 2.0, 0.0, -1])

    # Defining of the vectors using casadi
    #theta1 = ca.SX([0.0])
    #n1 = ca.SX([0.0, 0.0, 1.0])
    #q1 = ca.vertcat(ca.cos(theta1/2), ca.sin(theta1/2)@n1)
    #t1 = ca.SX([0.0, 1.0, 2.0, 3.0])

    #theta2 = ca.SX([ca.pi/2])
    #n2 = ca.SX([1.0, 0.0, 0.0])
    #q2 = ca.vertcat(ca.cos(theta2/2), ca.sin(theta2/2)@n2)
    #t2 = ca.SX([0.0, 0.0, 1.0, 0.0])

    # Create Quaternions objects
    #quat_1_r = Quaternion(q = q1)
    #quat_1_t = Quaternion(q = t1)

    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)
    Q2 = DualQuaternion.from_pose(quat = q2, trans = t2)
    Q3 = DualQuaternion.from_pose(quat = q3, trans = t3)
    #Q1 = DualQuaternion(q_real = quat_1_r, q_dual = quat_1_t)
    alpha = 0.2

    # Message 
    message_ros = "DualQuaternion Casadi "

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Update Reference
        Q4 = Q1 * Q2
        print(Q4*2 + Q2 + alpha)

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