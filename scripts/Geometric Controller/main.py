#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control, plot_cost_total
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

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

    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = x[1]
    msg.pose.pose.orientation.y = x[2]
    msg.pose.pose.orientation.z = x[3]
    msg.pose.pose.orientation.w = x[0]

    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0

    msg.twist.twist.angular.x = x[4]
    msg.twist.twist.angular.y = x[5]
    msg.twist.twist.angular.z = x[6]
    return msg

def send_odom_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None

def get_random_quaternion_complete(number):
    random_rotation = R.random(number)
    
    # Convert the rotation to quaternion format
    quaternion = random_rotation.as_quat()  # Returns (x, y, z, w)
    return quaternion

def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list, odom_pub_1, odom_pub_2, initial)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # OUTPUS 
    # x                         - states of the system
    # F                         - force control action
    # M                         - torques of the system
    # delta_t                   - computational time

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)
    N_planning = t.shape[0]-1

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)

    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized states of the system
    x = np.zeros((7, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    # Control actions
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized control actions
    u = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Defining Desired Orientation
    theta_d = -np.pi/2
    n_d = np.array([0.0, 0.0, 1.0])
    q_d = np.hstack([np.cos(theta_d / 2), np.sin(theta_d / 2) * np.array(n_d)])

    x_d = np.zeros((7, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x_d[0:4, 0] = q_d

    # Odometry message
    quat_1_msg = Odometry()
    quat_1_d_msg = Odometry()

    # Set Odom Message
    quat_1_msg = set_odom_msg(quat_1_msg, x[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)

    quat_1_d_msg = set_odom_msg(quat_1_d_msg, x_d[:, 0])
    send_odom_msg(quat_1_d_msg, odom_pub_2)


    message_ros = "Geometric Controller "
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic
        delta_t[:, k] = toc_solver

        # Dynamics
        x[:, k + 1 ] = x[:, k]
        x_d[:, k+1] = x_d[:, k]
        # Send msg to Ros
        quat_1_msg = set_odom_msg(quat_1_msg, x[:, k+1])
        send_odom_msg(quat_1_msg, odom_pub_1)

        quat_1_d_msg = set_odom_msg(quat_1_d_msg, x_d[:, k+1])
        send_odom_msg(quat_1_d_msg, odom_pub_2)

        rospy.loginfo(message_ros + str(toc_solver))


if __name__ == '__main__':
    try:
        # Node Parameters
        rospy.init_node("Geometric_Controller", disable_signals=True, anonymous=True)
        odomety_topic_1 = "/" + "frame_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "frame_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        # Time parameters
        ts = 0.03
        t_f = 20
        t_N = 0.5

        # System Dynamic Parameters
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        L = [Jxx, Jyy, Jzz]
        number_experiments = 1
        ramdon_quaternions = get_random_quaternion_complete(number_experiments)

        # Empty Matrix for my inital States
        X_total = []
        for i_random in range(number_experiments):
            omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
            #angle_0, axis_0 = get_random_quaternion()
            #quat_0 = np.array([ramdon_quaternions[i_random, 3], ramdon_quaternions[i_random, 0], ramdon_quaternions[i_random, 1], ramdon_quaternions[i_random, 2]])
            quat_0 = np.array([1, 0.0, 0.0, 0.0])
            x = np.hstack((quat_0, omega_0))
            X_total.append(x)
        
        X_total = np.array(X_total)
        for k in range(0, X_total.shape[0]):
            main(ts, t_f, t_N, X_total[k, :], L, odometry_publisher_1, odometry_publisher_2, k)
        #savemat("Dual_cost_without_velocities" + ".mat", mdic_x)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass