#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion.quaternion import Quaternion
from dual_quaternion.quaternion import DualQuaternion


def main():
    # Sample Time Defintion
    sample_time = 0.05
    t_f = 30

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Dual_Quaternion.....")

    # Init Quaternion Axis angle
    theta = np.pi/2
    n = np.array([0.0, 1.0, 0.0])

    theta_2 = np.pi/2
    n_2 = np.array([0.0, 1.0, 0.0])

    # Initial quaternion
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    t1 = np.array([0.0, 1.0, 2.0, 3.0])

    q2 = np.hstack([np.cos(theta_2 / 2), np.sin(theta_2 / 2) * np.array(n_2)])
    t2 = np.array([0.0, 0, 2.0, 0.0])
    

    # Object quaternion
    Q1 = DualQuaternion(q = q1, t = t1, name = "dual_1")

    Q2 = DualQuaternion(q = q2, t = t2, name = "dual_2")

    w1 = [0.0, 0.0, 0.0, 0.0]
    v1 = [0.0, 0.0, 0.0, 0.0]
    U1 = np.hstack((w1, v1))

    w2 = [0.0, 0.0, 0.0, 0.1]
    v2 = [0.0, 0.0, 0.0, 0.0]
    U2 = np.hstack((w2, v2))
    # Message 
    message_ros = "Quaternion "

    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Update Quaternions
        Q2.__adj__(U2)
        # ODE systems
        Q1.__ode__(Q1.dual_velocity_body(U1), sample_time)
        Q2.__ode__(Q2.dual_velocity_body(U2), sample_time)
        # Send Data
        Q1.send_odometry()
        Q2.send_odometry()

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
        rospy.init_node("Dual_quaternions", disable_signals=True, anonymous=True)
        lambda_value = rospy.get_param('~lambda_value', -1.0)  # Default to None if not provided
        if lambda_value == -1.0:
            lambda_value = None
        else:
            lambda_value = lambda_value
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass