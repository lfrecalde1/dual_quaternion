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
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Dual_Quaternion.....")

    # Init Quaternion Axis angle
    theta = 3.81
    n = np.array([0.4896, 0.2032, 0.8480])

    # Initial quaternion
    q1 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])
    t1 = np.array([0.0, -1.0, 10.0, -6.0])

    # Object quaternion
    Q1 = DualQuaternion(q = q1, t = t1, name = "dual_1")
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