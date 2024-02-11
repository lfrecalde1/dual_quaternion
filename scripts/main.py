#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from nav_msgs.msg import Odometry
def get_odometry(odom_msg, dqd, name):
    q_d = dqd.get
    # Function to send the Oritentation of the Quaternion
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

def quatdot_c(quat, omega):
    # Quaternion evolution guaranteeing norm 1 (Improve this section)
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    # Split values quaternion
    quat_data = quat.get
    qw = quat_data[0, 0]
    qx = quat_data[1, 0]
    qy = quat_data[2, 0]
    qz = quat_data[3, 0]

    # Auxiliary variable in order to avoid numerical issues
    K_quat = 2
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    q_dot = (1/2)*(omega*quat) + K_quat*quat_error*quat
    return q_dot
def main(odom_pub_1, odom_pub_2):
    # Sample Time Defintion
    sample_time = 0.01
    t_f = 50

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternion Axis angle
    theta = 3.8134
    n = np.array([0.4896, 0.2032, 0.8480])

    theta_2 = 0.0
    n_2 = np.array([0.0, 0.0, 1.0])

    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    q2 = np.hstack([np.cos(theta_2 / 2), np.sin(theta_2 / 2) * np.array(n_2)])

    quat_1 = Quaternion(q = q1)
    quat_2 = Quaternion(q = q2)

    quat_1_msg = Odometry()
    quat_2_msg = Odometry()

    # Angular Velocities 
    w = np.zeros((4, t.shape[0]), dtype=np.double)
    qw = Quaternion(q = w[:, 0])
    

    # Message 
    message_ros = "Quaternion "

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Set Velocities
        qw.set(w[:, k])
        # Update Quaternions
        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, quat_1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, quat_2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)

        print(quat_1)
        print(quat_2)


        # System Evolution
        qp = quatdot_c(quat_1, qw)


        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    return None

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