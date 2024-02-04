#!/usr/bin/env python3
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion.quaternion import Quaternion

def main():
    # Sample Time Defintion
    sample_time = 0.05

    # Time defintion aux variable
    t = 0

    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Quaternion.....")

    # Init Quaternion Axis angle
    theta = np.pi/4
    n = np.array([0.0, 0.0, 1.0])
    q0 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])

    # Object quaternion
    q = Quaternion(q = q0)

    # Message 
    message_ros = "Quaternion "
    while not rospy.is_shutdown():
        tic = rospy.get_time()


        # Time restriction Correct
        loop_rate.sleep()

        # Send Odometry
        q.send_odometry()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))
        t = t + delta
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