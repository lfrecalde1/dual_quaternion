from numbers import Number
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import random
from scipy.linalg import expm


class Quaternion():
    # Properties of the class
    q = np.array([1.0, 0.0, 0.0, 0.0])
    def __init__(self, qw=0.0, qx=0.0, qy=0.0, qz=0.0, q=None, name = "quat"):
        # Class initialization
        # INPUT:
        #  qw: Scalar part of the quaternion.
        #  qx: First element of the vector part of the quaternion.
        #  qy: Second element of the vector part of the quaternion.
        #  qz: Third element of the vector part of the quaternion.
        #  q: Quaternion as an array, list, tuple, or numpy array [qw, qx, qy, qz].
        # Example:
        #  q = Quaternion(q=[qw, qx, qy, qz])
        #  q = Quaternion(q=(qw, qx, qy, qz))
        #  q = Quaternion(q=np.array([qw, qx, qy, qz]))
        #  q = Quaternion(qw, qx, qy, qz)
        if q is not None:
            if isinstance(q, (np.ndarray, list, tuple)):
                if len(q) != 4:
                    raise ValueError("Array q must have exactly 4 elements.")
                self.q = np.array(q, dtype=np.double)
                self.q = self.q.reshape((4, 1))
            else:
                raise TypeError("q must be an ndarray, list, or tuple.")
        else:
            if not all(isinstance(i, Number) for i in [qw, qx, qy, qz]):
                raise TypeError("qw, qx, qy, qz should be scalars.")
            self.q = np.array([qw, qx, qy, qz], dtype=np.double)
            self.q = self.q.reshape((4, 1))
        # Odometry message
        self.odom_msg = Odometry()

        # Name
        self.name = name

        # Publisher Odometry
        odomety_topic = "/" + self.name + "/odom"
        self.odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 10)

    def __str__(self):
        return np.str(self.q)

    def __repr__(self):
        return "<Quaternion w:{} x:{} y:{} z:{}>".format(self.q[0, 0], self.q[1, 0], self.q[2, 0], self.q[3, 0])

    def get_odometry(self):
        # Function to send the Oritentation of the Quaternion
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = self.name

        self.odom_msg.pose.pose.position.x = 0
        self.odom_msg.pose.pose.position.y = 0
        self.odom_msg.pose.pose.position.z = 0

        self.odom_msg.pose.pose.orientation.x = self.q[1, 0]
        self.odom_msg.pose.pose.orientation.y = self.q[2, 0]
        self.odom_msg.pose.pose.orientation.z = self.q[3, 0]
        self.odom_msg.pose.pose.orientation.w = self.q[0, 0]
        return None

    def send_odometry(self):
        # Function to send the orientation of the Quaternion
        self.get_odometry()
        # Send Odometry
        self.odometry_publisher.publish(self.odom_msg)
        return None

    
    def matrix_quaternion(qw=0.0, qx=0.0, qy=0.0, qz=0.0, q=None):
        # Funtion that transforms a quaternion (4,1) to a matrix (4, 4)
        # INPUT
        #  qw: Scalar part of the quaternion.
        #  qx: First element of the vector part of the quaternion.
        #  qy: Second element of the vector part of the quaternion.
        #  qz: Third element of the vector part of the quaternion.
        #  q: Quaternion as an array, list, tuple, or numpy array [qw, qx, qy, qz].

        # OUTPUT
        # qm: Quaternion in a matrix form

        if q is not None:
            if isinstance(q, (np.ndarray, list, tuple)):
                if len(q) != 4:
                    raise ValueError("Array q must have exactly 4 elements.")
                qm = np.array([[q[0], -q[1], -q[2], -q[3]],
                               [q[1], q[0], -q[3], q[2]],
                               [q[2], q[3], q[0], -q[1]],
                               [q[3], -q[2], q[1], q[0]]], dtype=np.double)
            else:
                raise TypeError("q must be an ndarray, list, or tuple.")
        else:
            if not all(isinstance(i, Number) for i in [qw, qx, qy, qz]):
                raise TypeError("qw, qx, qy, qz should be scalars.")
            qm = np.array([[qw, -qx, -qy, -qz],
                            [qx, qw, -qz, qy],
                            [qy, qz, qw, -qx],
                            [qz, -qy, qx, qw]], dtype=np.double)
        return qm

    def __mul__(self, q1):
        # Function that multiples two quaternions
        # q = q1 x q2
        # INPUT                                    
        # q1                                      - Quaternion 
        # OUTPUT
        # q_mul                                   - Results of the multiplication of two Quaternions
        q1m = self.matrix_quaternion(q = q1)
        q2 = self.q
        q_mul = q1m@q2
        return q_mul
    
    def __ode__(self, w, ts):
        # Funtion that evolve the quaternion states
        # INPUTS
        # w                                                       - Angular velocities
        # ts                                                      - Sample time
        # Output                                                  
        # q_k                                                     - Solution ODE
        wm = self.matrix_quaternion(q = w)
        wm_aux = wm*(ts/2)
        wm_exp = expm(wm_aux)

        # ODE
        q_k = wm_exp@self.q
        self.q = q_k
        return None

    @property
    def get(self):
        return self.q[:, 0]