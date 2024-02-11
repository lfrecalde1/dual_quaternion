from numbers import Number
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import random
from scipy.linalg import expm
class DualQuaternion():
    # Properties of the class
    q = None
    t = None
    def __init__(self,tw = 0.0, tx = 0.0, ty = 0.0, tz = 0.0, t = None, qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, q = None, dual=None, name = None):
        # Check Values for the quaternion elements
        if dual is not None:
            if isinstance(dual, (np.ndarray, list, tuple)):
                if len(dual) != 8:
                    raise ValueError("Array q must have exactly 4 elements.")
                self.p = Quaternion(q = dual[0:4])
                self.d = Quaternion(q = dual[4:8])
            else:
                raise TypeError("dual must be an ndarray, list, or tuple.")
        else:
            if q is not None:
                if isinstance(q, (np.ndarray, list, tuple)):
                    if len(q) != 4:
                        raise ValueError("Array q must have exactly 4 elements.")
                    q = Quaternion(q = q)
                elif isinstance(q, (Quaternion)):
                    q = q
                else:
                    raise TypeError("q must be an ndarray, list, or tuple.")
            else:
                if not all(isinstance(i, Number) for i in [qw, qx, qy, qz]):
                    raise TypeError("qw, qx, qy, qz should be scalars.")
                q = Quaternion(qw = qw, qx = qx, qy = qy, qz = qz)

            # Check Values for the position elements
            if t is not None:
                if isinstance(t, (np.ndarray, list, tuple)):
                    if len(t) != 4:
                        raise ValueError("Array t must have exactly 4 elements with the first values equal to zero.")
                    t = Quaternion(q = t)
                elif isinstance(t, (Quaternion)):
                    t = t
                else:
                    raise TypeError("t must be an ndarray, list, or tuple.")
            else:
                if not all(isinstance(i, Number) for i in [tw, tx, ty, tz]):
                    raise TypeError("tw, tx, ty, tz should be scalars.")
                t = Quaternion(qw = tw, qx = tx, qy = ty, qz = tz)
                
            self.p = q
            dual = q.__mul__(q1 = t)
            dual_data = (1/2)*dual.get()
            self.d = Quaternion(q = dual_data)

        
        # Set name for the Odometry topic
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

    def get_p(self):
        # Funtion that gets the dualquaternion primal part as np array
        return self.p.get()

    def get_q(self):
        # Function thar returns the quaternion of the dualquaternion
        return self.p.get()

    def get_d(self):
        # Funtion that gets the quaternion dual part as np array
        return self.d.get()

    def get_t(self):
        # Funtion that returns the translation of the dualquaternion
        primal_m = self.p.matrix_quaternion_aux()
        primal_m_t = primal_m.T
        dual_v = self.d.get()
        dual_v = dual_v.reshape((4, 1))
        trans = 2*primal_m_t@dual_v
        return trans[:, 0]

    def get(self):
        # Funtion that gets the dualquaternion as np array
        return np.hstack([self.p.get(), self.d.get()])

    def matrix_dualquaternion(self):
        # Funtion that transforms a dualquaternion (8,1) to a matrix (8, 8)
        # INPUT  
        # None                                                             
        # OUTPUT
        # H                                                                 - Dualquaternion matrix
        h1p = self.p.matrix_quaternion()
        h1d = self.d.matrix_quaternion()
        h1_zero = np.zeros((4, 4), dtype=np.double)
        H = np.vstack((np.hstack((h1p, h1_zero)),np.hstack((h1d, h1p))))
        return H

    def __mul__(self, q1, name=None):
        # Check if the input is a quaternion
        if isinstance(q1, (DualQuaternion)):
            D1 = q1
        else:
            D1 = DualQuaternion(dual = q1)

        # Function that multiples two quaternions
        # q = q1 x q2
        # INPUT                                    
        # q1                                      - Quaternion 
        # OUTPUT
        # q_mul                                   - Results of the multiplication of two Quaternions
        q1m = D1.matrix_dualquaternion()
        q2 = self.get()
        q2 = q2.reshape((8, 1))
        q_mul = q1m@q2
        return DualQuaternion(dual = q_mul[:, 0], name= name)

    def get_odometry(self):
        # Function to send the Oritentation of the Quaternion
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "world"
        self.odom_msg.child_frame_id = self.name
        # Get Data 
        t = self.get_t()
        q = self.get_q()
        self.odom_msg.pose.pose.position.x = t[1]
        self.odom_msg.pose.pose.position.y = t[2]
        self.odom_msg.pose.pose.position.z = t[3]

        self.odom_msg.pose.pose.orientation.x = q[1]
        self.odom_msg.pose.pose.orientation.y = q[2]
        self.odom_msg.pose.pose.orientation.z = q[3]
        self.odom_msg.pose.pose.orientation.w = q[0]
        return None

    def send_odometry(self):
        # Function to send the orientation of the Quaternion
        if self.name is not None:
            self.get_odometry()
            # Send Odometry
            self.odometry_publisher.publish(self.odom_msg)
        else:
            raise ValueError("This is not a Quaternion with publisher")
        return None

    def dual_velocity_body(self, w):
        # Function that maps the linear and angular velocities to the dual quaternion form
        if isinstance(w, (DualQuaternion)):
            aux = w.get()
            w1 = DualQuaternion(dual = aux)

        else:
            aux = w
            w1 = DualQuaternion(dual = aux)

        # Transform velocity to the body
        #w1 = self.__adj__(w1)

        # Split values
        w1_data = w1.get()
        angular = w1_data[1:4]
        linear = w1_data[5:8]

        # Position dual
        p_data = self.get_t()
        p = p_data[1:4]

        # Create Primal and dual
        primal = np.hstack((0, angular))
        dual_aux = linear + np.cross(angular, p)
        dual = np.hstack((0, dual_aux))

        dual_velocity = np.hstack((primal, dual))

        return DualQuaternion(dual=dual_velocity)

    def __ode__(self, w, ts):
        # Funtion that evolves the Dualquaternion states
        # INPUTS
        # w                                                       - Angular velocities
        # ts                                                      - Sample time
        # Output                                                  
        # q_k                                                     - Solution ODE
        # Check if the input is a quaternion

        if isinstance(w, (DualQuaternion)):
            aux = w.get()
            w1 = DualQuaternion(dual = aux*(ts/2))

        else:
            aux = w*(ts/2)
            w1 = DualQuaternion(dual = aux)

        wm = w1.matrix_dualquaternion()
        wm_aux = wm
        wm_exp = expm(wm_aux)

        # ODE
        q2 = self.get()
        q2 = q2.reshape((8, 1))
        q_k = wm_exp@q2

        self.p = Quaternion(q = q_k[0:4, 0])
        self.d = Quaternion(q = q_k[4:8, 0])
        return None

    def __adj__(self, q1):
        # Check if the input is a DualQuaternion
        if isinstance(q1, (DualQuaternion)):
            q1 = q1
        else:
            q1 = DualQuaternion(dual = q1)

        q2_c = self.conjugate()
        q2 = self.get()

        aux_1 = q2_c.__mul__(q1)
        aux_2 = aux_1.__mul__(q2)
        q_mul = aux_2.get()

        return DualQuaternion(dual = q_mul)

    def conjugate(self):
        # Compute the Conjugate of a quaternion
        primal_c = self.p.conjugate()
        dual_c = self.d.conjugate()
        primal_c_data = primal_c.get()
        dual_c_data = dual_c.get()
        aux = np.hstack((primal_c_data, dual_c_data))

        return DualQuaternion(dual = aux)

class Quaternion():
    # Properties of the class
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.double)
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

    def __show__(self):
        print(self.q)
        return None

    def get_odometry(self):
        # Function to send the Oritentation of the Quaternion
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "world"
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
            raise ValueError("This is not a Quaternion with publisher")
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

    def matrix_quaternion_aux(self):
        # Funtion that transforms a quaternion (4,1) to a matrix (4, 4)
        # INPUT
        q_aux = self.q[:, 0]

        # OUTPUT
        # qm: Quaternion in a matrix form

        qm = np.array([[q_aux[0], -q_aux[1], -q_aux[2], -q_aux[3]],
                    [q_aux[1], q_aux[0], q_aux[3], -q_aux[2]],
                    [q_aux[2], -q_aux[3], q_aux[0], q_aux[1]],
                    [q_aux[3], q_aux[2], -q_aux[1], q_aux[0]]], dtype=np.double)
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
    
    def __adj__(self, q1):
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
        q2_c = self.conjugate()
        q2 = self.get()

        aux_1 = q2_c.__mul__(q1)
        aux_2 = aux_1.__mul__(q2)
        q_mul = aux_2.get()
        return Quaternion(q = q_mul)

    def __ode__(self, w, ts):
        # Funtion that evolve the quaternion states
        # INPUTS
        # w                                                       - Angular velocities
        # ts                                                      - Sample time
        # Output                                                  
        # q_k                                                     - Solution ODE
        # Check if the input is a quaternion

        if isinstance(w, (Quaternion)):
            aux = w.get()
            w1 = Quaternion(q = aux*(ts/2))

        else:
            aux = w*(ts/2)
            w1 = Quaternion(q = aux)

        wm = w1.matrix_quaternion()
        wm_aux = wm
        wm_exp = expm(wm_aux)

        # ODE
        q_k = wm_exp@self.q
        self.q = q_k
        return None

    def conjugate(self):
        # Compute the Conjugate of a quaternion
        return Quaternion(self.q[0,0], -self.q[1,0], -self.q[2,0], -self.q[3,0])


    def right_error(self, q1, lambda_aux = None):
        # Funtion that evolve the quaternion states
        # error = q1 x q2_c
        # INPUTS
        # q1                                                       - Quaternion
        # Output                                                  
        # error                                                    - Error quaternion
        # Check if the input is a quaternion
        if isinstance(q1, (Quaternion)):
            None
        else:
            q1 = Quaternion(q = q1)

        # get the conjugate

        # Compute right error
        q2 = self.get()

        error = q1.__mul__(q1 = q2)

        error_data = error.get()

        # Check shortest path
        if lambda_aux is not None:
            if isinstance(lambda_aux, Number):
                # Check for shortest path
                if error_data[0]>=0:
                    aux = 1.0
                else:
                    aux = -1.0
                error_aux = aux*error_data
            else:
                raise TypeError("Lambda must be a number.")
        else:
            error_aux = error_data

        return Quaternion(q = error_aux)

    def angle_axis(self):

        #if np.isclose(self.q[0,0], 1., atol=1.e-10):
            #return (np.array([[0.0], [0.0], [0.0], [1.0]]))

        #angle = 2. * np.arccos(self.q[0, 0])
        #x = self.q[1, 0] / np.sqrt(1. - self.q[0, 0]**2)
        #y = self.q[2, 0] / np.sqrt(1. - self.q[0, 0]**2)
        #z = self.q[3, 0] / np.sqrt(1. - self.q[0, 0]**2)

        vector = Quaternion(q=[0.0, self.q[1, 0], self.q[2, 0], self.q[3, 0]])
        norm = vector.norm()
        #angle = 2. * np.arccos(self.q[0, 0])
        angle = np.arctan2(norm, self.q[0])
        if  np.abs(angle[0]) > 2.22e-10:
            x = self.q[1, 0] / norm
            y = self.q[2, 0] / norm
            z = self.q[3, 0] / norm
        else:
            angle = [0.0]
            x = 0.0
            y = 0.0
            z = 1.0
        result = np.array([[angle[0]], [x], [y], [z]], dtype = np.double)
        #result = np.array([[angle], [x], [y], [z]], dtype = np.double)
        return result

    def ln_quat(self):
        # Log mapping
        angle_axis_aux = self.angle_axis()
        angle_axis_aux = angle_axis_aux[:, 0]
        return Quaternion(qw= 0.0, qx= (1/2)*(angle_axis_aux[0])*angle_axis_aux[1], qy = (1/2)*(angle_axis_aux[0])*angle_axis_aux[2], qz= (1/2)*(angle_axis_aux[0])*angle_axis_aux[3])


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

    def norm(self):
        # Function that calculates the norm of a quaternion
        q1 = self.get()
        q1 = q1.reshape((4, 1))
        norm_2 = np.sqrt(np.dot(q1.T, q1))
        return norm_2[0, 0]

    def norm_square(self):
        # Function that calculates the square norma of a quaternion
        q1 = self.get()
        q1 = q1.reshape((4, 1))
        norm_2 = np.dot(q1.T, q1)
        return norm_2[0, 0]

    def inverse(self):
        # Function that calculates the inverse of a quaternion
        q1_c = self.conjugate()
        q1_c_data = q1_c.get()
        inverse = q1_c_data/self.norm_square()
        return Quaternion(q= inverse)

    def get(self):
        # Funtion that gets the quaternion as np array
        return self.q[:, 0]

    def set(self, qw=0.0, qx=0.0, qy=0.0, qz=0.0, q=None):
        # Funtion that sets the quaternion as np array
        if q is not None:
            if isinstance(q, (np.ndarray, list, tuple)):
                if len(q) != 4:
                    raise ValueError("Array q must have exactly 4 elements.")
                aux = np.array(q, dtype=np.double)
                self.q = aux.reshape((4, 1))
            else:
                raise TypeError("q must be an ndarray, list, or tuple.")
        else:
            if not all(isinstance(i, Number) for i in [qw, qx, qy, qz]):
                raise TypeError("qw, qx, qy, qz should be scalars.")
            aux= np.array([qw, qx, qy, qz], dtype=np.double)
            self.q = aux.reshape((4, 1))
        return None