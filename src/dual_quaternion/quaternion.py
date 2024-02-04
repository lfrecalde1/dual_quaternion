from numbers import Number
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import random


class Quaternion():
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
            else:
                raise TypeError("q must be an ndarray, list, or tuple.")
        else:
            if not all(isinstance(i, Number) for i in [qw, qx, qy, qz]):
                raise TypeError("qw, qx, qy, qz should be scalars.")
            self.q = np.array([qw, qx, qy, qz], dtype=np.double)
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
        return "<Quaternion w:{} x:{} y:{} z:{}>".format(self.q[0], self.q[1], self.q[2], self.q[3])

    def get_odometry(self):
        # Function to send the Oritentation of the Quaternion
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = self.name

        self.odom_msg.pose.pose.position.x = 0
        self.odom_msg.pose.pose.position.y = 0
        self.odom_msg.pose.pose.position.z = 0

        self.odom_msg.pose.pose.orientation.x = self.q[1]
        self.odom_msg.pose.pose.orientation.y = self.q[2]
        self.odom_msg.pose.pose.orientation.z = self.q[3]
        self.odom_msg.pose.pose.orientation.w = self.q[0]
        return None

    def send_odometry(self):
        # Function to send the orientation of the Quaternion
        self.get_odometry()
        # Send Odometry
        self.odometry_publisher.publish(self.odom_msg)
        return None