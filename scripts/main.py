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
    sample_time = 0.02

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

    # Velocities




    # Object quaternion
    q1 = Quaternion(q = q0, name = "quat_1")

    # Message 
    message_ros = "Quaternion "
    
    # Simulation loop
    while not rospy.is_shutdown():
        tic = rospy.get_time()
        
        # Time restriction Correct
        loop_rate.sleep()

        # Send Odometry
        q1.send_odometry()

        # System evolution
        q1.__ode__([0.0, 0.0, 0.0, 0.5], sample_time)

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