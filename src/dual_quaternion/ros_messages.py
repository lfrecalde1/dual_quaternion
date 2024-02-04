from nav_msgs.msg import Odometry
import rospy

def get_odometry(odom_msg, name, quat):
    # Function to send the Oritentation of the Quaternion
    q = quat.get

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "map"
    odom_msg.child_frame_id = name

    odom_msg.pose.pose.position.x = 0
    odom_msg.pose.pose.position.y = 0
    odom_msg.pose.pose.position.z = 0

    odom_msg.pose.pose.orientation.x = q[1, 0]
    odom_msg.pose.pose.orientation.y = q[2, 0]
    odom_msg.pose.pose.orientation.z = q[3, 0]
    odom_msg.pose.pose.orientation.w = q[0, 0]
    return odom_msg

def send_odometry(odom_msg, odometry_publisher):
    # Function to send the orientation of the Quaternion
    # Send Odometry
    odometry_publisher.publish(odom_msg)
    return None