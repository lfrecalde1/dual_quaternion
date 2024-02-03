#!/usr/bin/env python3
import time
import rospy
import numpy as np

import numpy as np
import matplotlib.pyplot as plt

from dual_quaternion.quaternion import Quaternion

def main():
    # Init Quaternion Axis angle
    theta = 3.8134
    n = np.array([0.4896, 0.2032, 0.8480])
    q0 = np.hstack([np.cos(theta / 2), np.sin(theta / 2) * np.array(n)])

    # Object quaternion
    q = Quaternion(q = q0)
    aux = q.__repr__()

    # Check elements
    print(aux)

    return None
if __name__ == '__main__':
    try:
        main()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass