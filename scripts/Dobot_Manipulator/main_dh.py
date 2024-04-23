#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control, plot_cost_total
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from functions import dualquat_from_pose_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, rotation_casadi, rotation_inverse_casadi, dual_velocity_casadi, dual_quat_casadi, velocities_from_twist_casadi

# Creating Funtions based on Casadi
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()
velocity_from_twist = velocities_from_twist_casadi()
dualquat_from_pose = dualquat_from_pose_casadi()

def tranform_data(origin_frame, child_frame, dqd, tf_pub):
    # Compute quaternion and translations
    t_d = get_trans(dqd)
    q_d = get_quat(dqd)

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = origin_frame
    t.child_frame_id = child_frame


    t.transform.translation.x = t_d[1]
    t.transform.translation.y = t_d[2]
    t.transform.translation.z = t_d[3]
    t.transform.rotation.x = q_d[1]
    t.transform.rotation.y = q_d[2]
    t.transform.rotation.z = q_d[3]
    t.transform.rotation.w = q_d[0]
    tf_pub.sendTransform(t)
    return None

def main():
    # Sample Time Defintion
    sample_time = 0.03
    t_f = 30

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    delta_t = np.zeros((1, t.shape[0]), dtype=np.double)

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")

    # Initial States of the robot manipulator
    qw1 = 1.0
    qx1 = 0.0
    qy1 = 0.0
    qz1 = 0.0
    tx1 = 0.0
    ty1 = 0.0
    tz1 = 0.147

    link_1 = dualquat_from_pose(qw1, qx1, qy1,  qz1, tx1, ty1, tz1)
    X_1 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    qw2 = -0.5
    qx2 = -0.5
    qy2 = -0.5
    qz2 = 0.5
    tx2 = 0.0
    ty2 = 0.0
    tz2 = 0.0

    link_2 = dualquat_from_pose(qw2, qx2, qy2,  qz2, tx2, ty2, tz2)
    X_2 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    
    qw3 = 1.0
    qx3 = 0.0
    qy3 = 0.0
    qz3 = 0.0
    tx3 = -0.427
    ty3 = 0.0
    tz3 = 0.0

    link_3 = dualquat_from_pose(qw3, qx3, qy3,  qz3, tx3, ty3, tz3)
    X_3 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    qw4 = 0.707105
    qx4 = 0.0
    qy4 = 0.0
    qz4 = -0.707108
    tx4 = -0.357
    ty4 = 0.0
    tz4 = 0.141

    link_4 = dualquat_from_pose(qw4, qx4, qy4,  qz4, tx4, ty4, tz4)
    X_4 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    qw5 = 0.707105
    qx5 = 0.707108
    qy5 = 0.0
    qz5 = 0.0
    tx5 = 0.0
    ty5 = -0.116
    tz5 = 0.0

    link_5 = dualquat_from_pose(qw5, qx5, qy5,  qz5, tx5, ty5, tz5)
    X_5 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    qw6 = 0.707105
    qx6 = -0.707108
    qy6 = 0.0
    qz6 = 0.0
    tx6 = 0.0
    ty6 = 0.105
    tz6 = 0.0

    link_6 = dualquat_from_pose(qw6, qx6, qy6,  qz6, tx6, ty6, tz6)
    X_6 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    # Set up initial conditions in the empty matrices
    X_1[:, 0] = np.array(link_1).reshape((8, ))
    X_2[:, 0] = np.array(link_2).reshape((8, ))
    X_3[:, 0] = np.array(link_3).reshape((8, ))
    X_4[:, 0] = np.array(link_4).reshape((8, ))
    X_5[:, 0] = np.array(link_5).reshape((8, ))
    X_6[:, 0] = np.array(link_6).reshape((8, ))

    message_ros = "Dobot Forward Kinematics "

    manipulador_frames = TransformBroadcaster()

    # Simulation loop
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()

        # Update Matrices
        X_1[:, k+1] = np.array(link_1).reshape((8, ))
        X_2[:, k+1] = np.array(link_2).reshape((8, ))
        X_3[:, k+1] = np.array(link_3).reshape((8, ))
        X_4[:, k+1] = np.array(link_4).reshape((8, ))
        X_5[:, k+1] = np.array(link_5).reshape((8, ))
        X_6[:, k+1] = np.array(link_6).reshape((8, ))

        # Send Data throught Ros
        tranform_data("world", "link_1", X_1[:, k+1], manipulador_frames)
        tranform_data("link_1", "link_2", X_2[:, k+1], manipulador_frames)
        tranform_data("link_2", "link_3", X_3[:, k+1], manipulador_frames)
        tranform_data("link_3", "link_4", X_4[:, k+1], manipulador_frames)
        tranform_data("link_4", "link_5", X_5[:, k+1], manipulador_frames)
        tranform_data("link_5", "link_6", X_6[:, k+1], manipulador_frames)

        # Sample time restriction
        loop_rate.sleep()

        # Save the appropriate sample time
        toc_solver = rospy.get_time() - tic
        delta_t[:, k] = toc_solver

        rospy.loginfo(message_ros + str(delta_t[:, k]))

    
if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("DualQuaternions", disable_signals=True, anonymous=True)
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass