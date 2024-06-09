#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from functions import dualquat_from_pose_casadi
from fancy_plots import fancy_plots_3, fancy_plots_4, fancy_plots_1
from fancy_plots import plot_dual_real_reference, plot_dual_dual_reference
dualquat_from_pose = dualquat_from_pose_casadi()


import os
script_dir = os.path.dirname(__file__)
folder_path = os.path.join(script_dir, 'results/')

def set_odom_msg(msg, x):
    # Set the odometry of the quadrotor
    # INPUT 
    # msg                                                               - odometry msg
    # x                                                                 - states of the system
    # OUTPUT
    # msg                                                               - odometry msg
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    #msg.child_frame_id = "blue_robot_base"

    msg.pose.pose.position.x = x[0]
    msg.pose.pose.position.y = x[1]
    msg.pose.pose.position.z = x[2]

    msg.pose.pose.orientation.x = x[4]
    msg.pose.pose.orientation.y = x[5]
    msg.pose.pose.orientation.z = x[6]
    msg.pose.pose.orientation.w = x[3]

    return msg

def send_odom_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None

def main(ts: float, t_f: float, t_N: float, L: list, odom_pub_1, odom_pub_2, initial)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # odom_pub_1                - Publisher system states
    # odom_pub_2                - Publisher system desired reference
    # OUTPUS 
    # x                         - states of the system
    # F                         - force control action
    # M                         - torques of the system
    # delta_t                   - computational time

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)


    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]


    # Generalized states of the system
    x = np.zeros((7, t.shape[0]), dtype=np.double)
    x[0, :] = np.cos(t)
    x[1, :] = np.sin(t)
    x[2, :] = t
    angle = np.zeros((1, t.shape[0]), dtype=np.double)
    angle[0, :] = 2*t
    axis =np.zeros((3, t.shape[0]), dtype=np.double) 
    axis[0, :] = np.cos(t)/np.sqrt(2)
    axis[1, :] = np.sin(t)
    axis[2, :] = np.cos(t)/np.sqrt(2)

    quat_0 = np.hstack([np.cos(angle[0, 0] / 2), np.sin(angle[0, 0] / 2) * axis[:, 0]])
    x[3:7, 0] = quat_0


    D1 = np.zeros((8, t.shape[0]), dtype=np.double)
    dual_1 = dualquat_from_pose(x[3, 0], x[4, 0], x[5, 0],  x[6, 0], x[0, 0], x[1, 0], x[2, 0])
    D1[:, 0] = np.array(dual_1).reshape((8, ))


    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)
    
    quat_1_msg = Odometry()

    # Set Odom Message
    quat_1_msg = set_odom_msg(quat_1_msg, x[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)

    # Message Ros
    message_ros = "Dual Quaternion Integrations "

    # Send msg to Ros
    quat_1_msg = set_odom_msg(quat_1_msg, x[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)
    # Loop simulation
    for k in range(0, t.shape[0] - 1):
        tic = rospy.get_time()
        # Update states
        quat_0 = np.hstack([np.cos(angle[0, k + 1] / 2), np.sin(angle[0, k] / 2) * axis[:, k + 1]])
        x[3:7, k+1] = quat_0

        # Update Dual Quaternion
        dual_1 = dualquat_from_pose(x[3, k+1], x[4, k+1], x[5, k+1],  x[6, k+1], x[0, k+1], x[1, k+1], x[2, k+1])
        D1[:,  k +1] = np.array(dual_1).reshape((8, ))
        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic

        # Send msg to Ros
        quat_1_msg = set_odom_msg(quat_1_msg, x[:, k+1])
        send_odom_msg(quat_1_msg, odom_pub_1)


        rospy.loginfo(message_ros + str(toc_solver))

    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_dual_real_reference(fig11, ax11, ax21, ax31, ax41, D1[0, :], D1[1:4, :], D1[0, :], D1[1:4, :], t, "Dual-Quaternion Real Integration "+ str(initial), folder_path)
    plt.show()
    
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_dual_dual_reference(fig12, ax12, ax22, ax32, ax42, D1[4, :], D1[5:8, :], D1[4, :], D1[5:8, :], t, "Dual-Quaternion Dual Integration "+ str(initial), folder_path)
    plt.show()
    return None

if __name__ == '__main__':
    try: #################################### Simulation  #####################################################
        # Time parameters
        ts = 0.01
        t_f = 5
        t_N = 0.5

        # Parameters of the system  (mass, inertial matrix, gravity)
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Ros Definition
        rospy.init_node("quadrotor",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        k = 0
        main(ts, t_f, t_N, L, odometry_publisher_1, odometry_publisher_2, k)

    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass