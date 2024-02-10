#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion

def main():
    # Sample Time Defintion
    sample_time = 0.05
    t_f = 20

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

    theta_2 = np.pi/2
    n_2 = np.array([0.0, 0.0, 1.0])

    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    q2 = np.hstack([np.cos(theta_2 / 2), np.sin(theta_2 / 2) * np.array(n_2)])

    # Symbolic quaternion using Casadi
    #theta_c = ca.MX.sym('theta_c', 1)
    #n_c = ca.MX.sym('n_c', 3, 1)
    theta1_c = ca.SX([3.8134])
    n1_c = ca.SX([0.4896, 0.2032, 0.8480])
    q1_c = ca.vertcat(ca.cos(theta1_c/2), ca.sin(theta1_c/2)@n1_c)

    theta2_c = ca.SX([ca.pi/2])
    n2_c = ca.SX([0.0, 0.0, 1.0])
    q2_c = ca.vertcat(ca.cos(theta2_c/2), ca.sin(theta2_c/2)@n2_c)


    quat_1 = Quaternion(q = q1_c)
    quat_1_c = quat_1.conjugate()
    quat_2 = Quaternion(q = q2_c)


    quat_3 = quat_1 - quat_2
    quat_4 = quat_2 - quat_1

    aux1 = q1 - q2
    aux2 =  q2 - q1

    aux1_c = q1_c - q2_c
    aux2_c = q2_c - q1_c

    aux_c = quat_1*quat_1_c

    print(q1)
    print(aux1)
    print(aux2)
    print("--------------------------")
    print(q1_c)
    print(aux1_c)
    print(aux2_c)
    print("--------------------------")
    print(quat_1)
    print(quat_3)
    print(quat_4)
    print("--------------------------")
    print(quat_1)
    print(quat_1_c)
    print(aux_c)







    return None

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("Quaternions", disable_signals=True, anonymous=True)
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass