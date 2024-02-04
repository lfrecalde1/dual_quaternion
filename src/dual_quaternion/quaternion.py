from numbers import Number
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import random
from scipy.linalg import expm


class Quaternion():
    # Properties of the class
    q = np.array([1.0, 0.0, 0.0, 0.0])
    def __init__(self, qw=0.0, qx=0.0, qy=0.0, qz=0.0, q=None, name = None):
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
        if name is None:
            self.name = name
        else:
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
        print(self.q)
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
        if self.name is not None:
            self.get_odometry()
            # Send Odometry
            self.odometry_publisher.publish(self.odom_msg)
        else:
            raise ValueError("This is not Quaternion with publishe")
        return None

    
    def matrix_quaternion(self):
        # Funtion that transforms a quaternion (4,1) to a matrix (4, 4)
        # INPUT
        q_aux = self.q[:, 0]

        # OUTPUT
        # qm: Quaternion in a matrix form

        qm = np.array([[q_aux[0], -q_aux[1], -q_aux[2], -q_aux[3]],
                    [q_aux[1], q_aux[0], -q_aux[3], q_aux[2]],
                    [q_aux[2], q_aux[3], q_aux[0], -q_aux[1]],
                    [q_aux[3], -q_aux[2], q_aux[1], q_aux[0]]], dtype=np.double)
        return qm

    def matrix_identity(self):
        # Funtion that transforms a quaternion (4,1) to a matrix (4, 4)
        # INPUT
        q_aux = self.q[:, 0]

        # OUTPUT
        # qm: Quaternion in a matrix form

        qm = np.array([[q_aux[0], 0.0, 0.0, 0.0],
                    [0.0, q_aux[1], 0.0, 0.0],
                    [0.0, 0.0, q_aux[2], 0.0],
                    [0.0, 0.0, 0.0, q_aux[3]]], dtype=np.double)
        return qm

    def __mul__(self, q1):
        # Check if the input is a quaternion
        if isinstance(q1, (Quaternion)):
            q1 = q1
        else:
            q1 = Quaternion(q = q1)
        # Function that multiples two quaternions
        # q = q1 x q2
        # INPUT                                    
        # q1                                      - Quaternion 
        # OUTPUT
        # q_mul                                   - Results of the multiplication of two Quaternions
        q1m = q1.matrix_quaternion()
        q2 = self.q
        q_mul = q1m@q2
        return Quaternion(q = q_mul[:, 0])
    
    def __ode__(self, w, ts):
        # Funtion that evolve the quaternion states
        # INPUTS
        # w                                                       - Angular velocities
        # ts                                                      - Sample time
        # Output                                                  
        # q_k                                                     - Solution ODE
        # Check if the input is a quaternion
        if isinstance(w, (Quaternion)):
            w = w
        else:
            w = Quaternion(q = w)

        wm = w.matrix_quaternion()
        wm_aux = wm*(ts/2)
        wm_exp = expm(wm_aux)

        # ODE
        q_k = wm_exp@self.q
        self.q = q_k
        return None

    def conjugate(self):
        # Compute the Conjugate of a quaternion
        return Quaternion(self.q[0,0], -self.q[1,0], -self.q[2,0], -self.q[3,0])

    def left_error(self, q1):
        # Funtion that evolve the quaternion states
        # error = q1_c x q2 
        # INPUTS
        # q1                                                       - Quaternion
        # Output                                                  
        # error                                                    - Error quaternion
        # Check if the input is a quaternion
        if isinstance(q1, (Quaternion)):
            q1 = q1
        else:
            q1 = Quaternion(q = q1)

        # get the conjugate
        q1_c = q1.conjugate()

        # Compute left error
        error = self.__mul__(q1 = q1_c)
        return error

    def right_error(self, q1):
        # Funtion that evolve the quaternion states
        # error = q1 x q2_c
        # INPUTS
        # q1                                                       - Quaternion
        # Output                                                  
        # error                                                    - Error quaternion
        # Check if the input is a quaternion
        if isinstance(q1, (Quaternion)):
            q1 = q1
        else:
            q1 = Quaternion(q = q1)

        # get the conjugate

        # Compute left error
        q2_c = self.conjugate()
        error = q2_c.__mul__(q1 = q1)
        return error

    def angle_axis(self):

        if np.isclose(self.q[0,0], 1., atol=1.e-20):
            return (np.array([[0.0], [0.0], [0.0], [1.0]]))

        angle = 2. * np.arccos(self.q[0, 0])
        x = self.q[1, 0] / np.sqrt(1. - self.q[0, 0]**2)
        y = self.q[2, 0] / np.sqrt(1. - self.q[0, 0]**2)
        z = self.q[3, 0] / np.sqrt(1. - self.q[0, 0]**2)

        return np.array([[angle], [x], [y], [z]], dtype=np.double)

    def ln_quat(self):
        # Log mapping
        angle_axis_aux = self.angle_axis()
        angle_axis_aux = angle_axis_aux[:, 0]
        return Quaternion(qw= 0.0, qx= angle_axis_aux[0]*angle_axis_aux[1], qy = angle_axis_aux[0]*angle_axis_aux[2], qz= angle_axis_aux[0]*angle_axis_aux[3])


    def vector_dot_product(self, q1):
        # Check if the input is a quaternion
        if isinstance(q1, (Quaternion)):
            q1 = q1
        else:
            q1 = Quaternion(q = q1)
        # Function that multiples two quaternions
        # q = q1 x q2
        # INPUT                                    
        # q1                                      - Quaternion 
        # OUTPUT
        # q_mul                                   - Results of the multiplication of two Quaternions
        q1m = q1.matrix_identity()
        q2 = self.q
        q_mul = q1m@q2
        return Quaternion(q = q_mul[:, 0])

    def get(self):
        return self.q[:, 0]