import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
def quatTorot(quat: np.ndarray)->np.ndarray:
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # R                                                          - rotational matrix
    # Normalized quaternion
    q = quat
    q = q/np.linalg.norm(q, 2)

    # Auxiliar variable
    q_hat = np.zeros((3, 3), dtype=np.double)
    q_hat[0, 1] = -q[3]
    q_hat[0, 2] = q[2]
    q_hat[1, 2] = -q[1]
    q_hat[1, 0] = q[3]
    q_hat[2, 0] = -q[2]
    q_hat[2, 1] = q[1]

    R = np.eye(3) + 2 * (q_hat@q_hat) + 2 * q[0] * q_hat
    return R
def quatdot(quat: np.ndarray, omega: np.ndarray)-> np.ndarray:
    # Quaternion evolution guaranteeing norm 1.
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    # Split values quaternion
    quat = quat.reshape((4, 1))
    qw = quat[0, 0]
    qx = quat[1, 0]
    qy = quat[2, 0]
    qz = quat[3, 0]

    # Split angular values
    p = omega[0]
    q = omega[1]
    r = omega[2]

    # Auxiliary variable in order to avoid numerical issues
    K_quat = 2
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    # Create skew matrix
    S = np.array([[0.0, -p, -q, -r], [p, 0.0, r, -q], [q, -r, 0.0, p], [r, q, -p, 0.0]], dtype=np.double)
    q_dot = (1/2)*(S@quat) + K_quat*quat_error*quat
    return q_dot

def systemdynamics(x: np.ndarray, F: np.ndarray, M: np.ndarray, L: list)-> np.ndarray:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # x                                                          - states of the system (13, )
    # F                                                          - Force (1, )
    # M                                                          - Torques(3, )
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # xdot                                                       - The rate of change of flow in the system. (13, )

    # Split system parameters
    m = L[0]
    Jxx = L[1]
    Jyy = L[2]
    Jzz = L[3]
    gravity = L[4]

    # Inertial Matrix
    J = np.array([[Jxx, 0.0, 0.0], [0.0, Jyy, 0.0], [0.0, 0.0, Jzz]], dtype=np.double)

    # Axiliar variable
    e3 = np.array([[0.0], [0.0], [1.0]], dtype=np.double)
    g = gravity*e3

    # Split states values
    vel = x[3:6]
    quat = x[6:10]
    omega = x[10:13]
    R = quatTorot(quat)

    # Control actions
    M = M.reshape((3, 1))
    F = F

    # Rate of change
    # Linear section
    vel = vel.reshape((3, 1))
    acc = ((F*(R@e3))/m) - g

    # Angular
    qdot = quatdot(quat, omega)
    aux = J@omega
    aux_cross = np.cross(omega, aux)
    aux_cross = aux_cross.reshape((3, 1))
    omega_dot = np.linalg.inv(J)@(M - aux_cross)
    
    # Stack Vectors
    xdot = np.vstack((vel, acc, qdot, omega_dot))
    return xdot[:, 0]

    
def axisToquaternion(angle: float, axis: list)-> np.ndarray:
    # DESCRIPTION
    # transform axis-angle to quaternion
    # INPUTS
    # anlge                                       - angle of rotation
    # axis                                        - axis where the rotation is executed
    # OUTPUT
    # quat                                        - unit quaternion
    axis_aux = np.array(axis)

    real_part = np.cos((angle)/2)
    imaginary_part = axis_aux*np.sin((angle)/2)

    quat = [real_part, imaginary_part[0], imaginary_part[1], imaginary_part[2]]
    quat = np.array(quat)
    return quat

def f_d(x: np.ndarray, F: np.ndarray, M: np.ndarray, ts: float, L: list):
    # Compute Runge Kutta
    # INPUT
    # x_k                                                          - states of the system (13, )
    # F_k                                                          - Force (1, )
    # M_k                                                          - Torques(3, )
    # L                                                            - system parameters(mass, Inertias and gravity)
    # ts                                                           - sample time
    # OUTPUT                           
    # x_k+1                                                        - The rate of change of flow in the system. (13, )

    k1 = systemdynamics(x, F, M, L)
    k2 = systemdynamics(x+(ts/2)*k1, F, M, L)
    k3 = systemdynamics(x+(ts/2)*k2, F, M, L)
    k4 = systemdynamics(x+(ts)*k3, F, M, L)
    x_k = x + (ts/6)*(k1 +2*k2 +2*k3 +k4)
    return x_k

def f_d_casadi(x: np.ndarray, F: np.ndarray, M: np.ndarray, ts: float, f_s):
    # Compute Runge Kutta
    # INPUT
    # x_k                                                          - states of the system (13, )
    # F_k                                                          - Force (1, )
    # M_k                                                          - Torques(3, )
    # ts                                                           - sample time
    # f_s                                                          - casadi dynamics
    # OUTPUT                           
    # x_k+1                                                        - The rate of change of flow in the system. (13, )

    k1 = f_s(x, F, M)
    k2 = f_s(x+(ts/2)*k1, F, M)
    k3 = f_s(x+(ts/2)*k2, F, M)
    k4 = f_s(x+(ts)*k3, F, M)
    x_k = x + (ts/6)*(k1 +2*k2 +2*k3 +k4)
    aux_x = np.array(x_k[:, 0]).reshape(13, )
    return aux_x

def ref_trajectory(t):
    # Compute the desired Trajecotry of the system
    # INPUT 
    # t                                                - time
    # OUTPUT
    # xd, yd, zd                                       - desired position
    # theta                                            - desired orientation
    # theta_p                                          - desired angular velocity

    # Desired period by the user
    period = 10.0  
    radius = 1.0  
    omega = (2 * np.pi) / period  # angular frequency

    # Compute desired reference x y z
    xd = radius * np.cos(omega * t)
    yd = radius * np.sin(omega * t)
    zd = 0.0*t

    # Compute velocities
    xd_p = -radius * omega * np.sin(omega * t)
    yd_p = radius * omega * np.cos(omega * t)
    zd_p = 0.0*t

    # Compute acceleration
    xd_pp = -radius * omega * omega * np.cos(omega * t)
    yd_pp = -radius * omega * omega * np.sin(omega * t)

    # Compute angular displacement
    theta = np.arctan2(yd_p, xd_p)

    # Compute angular velocity
    theta_p = (1. / ((yd_p / xd_p) ** 2 + 1)) * ((yd_pp * xd_p - yd_p * xd_pp) / xd_p ** 2)
    theta_p[0] = 0

    return xd, yd, zd, theta, theta_p

def ref_trajectory_agresive(t, mul):
    # Compute the desired Trajecotry of the system
    # INPUT 
    # t                                                - time
    # OUTPUT
    # xd, yd, zd                                       - desired position
    # theta                                            - desired orientation
    # theta_p                                          - desired angular velocity

    Q = 0.01
    # Compute desired reference x y z
    xd = 4 * np.sin(mul * 0.04* t)
    yd = 4 * np.sin(mul * 0.08 * t)
    zd = 0 * np.sin(Q*t) + 0

    # Compute velocities
    xd_p = 4*mul*0.04*np.cos(mul*0.04*t)
    yd_p = 4*mul*0.08*np.cos(mul*0.08*t)
    zd_p = 0*Q*np.cos(Q*t)

    # Compute acceleration
    xd_pp = -4*mul*mul*0.04*0.04*np.sin(mul*0.04*t)
    yd_pp = -4*mul*mul*0.08*0.08*np.sin(mul*0.08*t);  

    # Compute angular displacement
    theta = np.arctan2(yd_p, xd_p)

    # Compute angular velocity
    theta_p = (1. / ((yd_p / xd_p) ** 2 + 1)) * ((yd_pp * xd_p - yd_p * xd_pp) / xd_p ** 2)
    theta_p[0] = 0

    return xd, yd, zd, theta, theta_p

def desired_quaternion(q, omega, ts):
    # Compute the the rate of change of the quaternion
    # INPUT 
    # q                                                                                       - quaternion
    # omega                                                                                   - angular velocity
    k1 = quatdot(q, omega)
    k2 = quatdot(q+(ts/2)*k1.reshape((4,)), omega)
    k3 = quatdot(q+(ts/2)*k2.reshape((4,)), omega)
    k4 = quatdot(q+(ts)*k3.reshape((4,)), omega)
    q_k = q + (ts/6)*(k1.reshape((4,)) +2*k2.reshape((4, )) +2*k3.reshape((4, )) +k4.reshape((4, )))
    return q_k

def compute_desired_quaternion(theta, theta_p, t, ts):
    # Compute the desired quaternion over time with respect to the angular velocity omega.
    # INPUT
    # theta                                                                              - desired inital angle 
    # theta_p                                                                            - angular velocity wz
    # t                                                                                  - time
    # ts                                                                                 - sample time
    # OUTPUT
    # q                                                                                  - desired quaternion 
    # Empty vector
    q = np.zeros((4, t.shape[0]), dtype = np.double)
    omega = np.zeros((3, t.shape[0]), dtype = np.double)

    # Euler angles to quaternion
    r = R.from_euler('zyx',[theta[0], 0, 0], degrees=False)
    r_q = r.as_quat()

    # Initial conditions
    q[0, 0] = r_q[3]
    q[1, 0] = r_q[0]
    q[2, 0] = r_q[1]
    q[3, 0] = r_q[2]

    # Angular velocity only z axis
    omega[2, :] = theta_p

    # Compute desired quaternion
    for k in range(0, t.shape[0]-1):
        q[:, k+1] = desired_quaternion(q[:, k], omega[:, k], ts)
    return  q
def get_euler_angles(q):
    # Compute euler angles
    # INPUTS
    # q                                           - quaternion
    # OUTPUT
    # euler                                       - euler angles
    x = np.array([q[1], q[2], q[3], q[0]], dtype=np.double)
    r = R.from_quat(x)
    euler = r.as_euler('zyx', degrees=False)
    return euler

def set_odom_msg(msg, x):
    # Set the odometry of the quadrotor
    # INPUT 
    # msg                                                               - odometry msg
    # x                                                                 - states of the system
    # OUTPUT
    # msg                                                               - odometry msg
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    #msg.child_frame_id = "blue_robot_base"

    msg.pose.pose.position.x = x[0]
    msg.pose.pose.position.y = x[1]
    msg.pose.pose.position.z = x[2]

    msg.pose.pose.orientation.x = x[7]
    msg.pose.pose.orientation.y = x[8]
    msg.pose.pose.orientation.z = x[9]
    msg.pose.pose.orientation.w = x[6]

    msg.twist.twist.linear.x = x[3]
    msg.twist.twist.linear.y = x[4]
    msg.twist.twist.linear.z = x[5]

    msg.twist.twist.angular.x = x[10]
    msg.twist.twist.angular.y = x[11]
    msg.twist.twist.angular.z = x[12]
    return msg

def send_odom_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None

def init_marker(marker, x):
    marker.header.frame_id = "map"  # Set the frame ID
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # Scale of the mesh
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.g = 1.0  # Red color
    aux_point = [Point(x[0], x[1], x[2])]
    marker.points = aux_point  # Position of the 
    return marker, aux_point

def set_marker(marker, x, aux_point):
    marker.header.frame_id = "map"  # Set the frame ID
    marker.header.stamp = rospy.Time.now()
    aux_point.append(Point(x[0], x[1], x[2]))
    marker.points = aux_point  # Position of the 
    return marker, aux_point

def init_marker_ref(marker, x):
    marker.header.frame_id = "map"  # Set the frame ID
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # Scale of the mesh
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.b = 1.0  # Red color
    aux_point = [Point(x[0], x[1], x[2])]
    marker.points = aux_point  # Position of the 
    return marker, aux_point

def set_marker_ref(marker, x, aux_point):
    marker.header.frame_id = "map"  # Set the frame ID
    marker.header.stamp = rospy.Time.now()
    aux_point.append(Point(x[0], x[1], x[2]))
    marker.points = aux_point  # Position of the 
    return marker, aux_point
def send_marker_msg(msg, pub):
    # Send Odometry to ROS framework
    # INPUT 
    # msg                                                                            - odometry msg
    # pub                                                                            - odometry publisher
    pub.publish(msg)
    return None