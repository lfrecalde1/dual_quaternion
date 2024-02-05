#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion.quaternion import Quaternion

def control_quat(qd, q, kp):
    q_e = q.left_error(qd)
    q_e_ln = q_e.ln_quat()
    U = q_e_ln.vector_dot_product(kp)
    return -2*U.get()


def main():
    # Sample Time Defintion
    sample_time = 0.01
    t_f = 5

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternion Axis angle
    theta = 3.81
    n = np.array([0.4896, 0.2032, 0.8480])

    # Desired quaternion
    theta_d = 0.45
    n_d = np.array([0.4896, 0.2032, 0.8480])

    # Initial quaternion
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    # Desired quaternion
    qd = np.hstack([np.cos(theta_d / 2), np.sin(theta_d / 2) * np.array(n_d)])

    # Object quaternion
    q1 = Quaternion(q = q1, name = "quat_1")
    qd = Quaternion(q = qd, name = "quat_d")
    kp = Quaternion(q = [0.0, 2, 2, 2])

    # Empty vector where the data is going to be stored
    Q1 = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Q1[:, 0] = q1.get()

    Qd = np.zeros((4, t.shape[0]+1), dtype=np.double)
    Qd[:, 0] = qd.get()
    # Message 
    message_ros = "Quaternion "
    
    # Simulation loop
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()

        u = control_quat(qd, q1, kp)

        
        # Time restriction Correct
        loop_rate.sleep()

        # System evolution
        q1.__ode__(u, sample_time)

        # Send Data
        q1.send_odometry()
        qd.send_odometry()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

        # Save Data
        Q1[:, k+1] = q1.get()
        Qd[:, k+1] = qd.get()

    # Reshape Data
    return None

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Quaternions",disable_signals=True, anonymous=True)
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass