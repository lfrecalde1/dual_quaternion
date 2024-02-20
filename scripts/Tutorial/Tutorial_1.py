
#!/usr/bin/env python
import time
import rospy
import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dual_quaternion import Quaternion
from dual_quaternion import DualQuaternion
from nav_msgs.msg import Odometry
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, fancy_plots_1, plot_norm_quat, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_norm_real, plot_norm_dual

def get_odometry(odom_msg, dqd, name):
    # Function to send the Oritentation of the Quaternion
    # Get Information from the DualQuaternion
    t_d = dqd.get_trans
    t_d_data = t_d.get

    q_d = dqd.get_quat
    q_d_data = q_d.get

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = name
    odom_msg.pose.pose.position.x = t_d_data[1]
    odom_msg.pose.pose.position.y = t_d_data[2]
    odom_msg.pose.pose.position.z = t_d_data[3]

    odom_msg.pose.pose.orientation.x = q_d_data[1]
    odom_msg.pose.pose.orientation.y = q_d_data[2]
    odom_msg.pose.pose.orientation.z = q_d_data[3]
    odom_msg.pose.pose.orientation.w = q_d_data[0]
    return odom_msg

def send_odometry(odom_msg, odom_pub):
    # Function to send the orientation of the Quaternion
    odom_pub.publish(odom_msg)
    return None


def main(odom_pub_1, odom_pub_2, odom_pub_3):
    # Sample Time Defintion
    sample_time = 0.01
    t_f = 20

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    
    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")

    # Defining of the vectors using casadi
    theta1 = ca.SX([np.pi/2])
    n1 = ca.SX([0.0, 0.0, 1.0])
    q1 = ca.vertcat(ca.cos(theta1/2), ca.sin(theta1/2)@n1)
    t1 = ca.SX([0.0, 1.0, 1.0, 0.0])

    # Defining of the vectors using casadi
    theta2 = ca.SX([ca.pi/4])
    n2 = ca.SX([1.0, 0.0, 0.0])
    q2 = ca.vertcat(ca.cos(theta2/2), ca.sin(theta2/2)@n2)
    t2 = ca.SX([0.0, 0.0, 2.0, 0.0])


    # Init Dualquaternion
    Q1 = DualQuaternion.from_pose(quat = q1, trans = t1)
    Q2 = DualQuaternion.from_pose(quat = q2, trans = t2)

    # Odometry Message
    quat_1_msg = Odometry()
    quat_2_msg = Odometry()
    quat_3_msg = Odometry()
    # Message 
    message_ros = "DualQuaternion Casadi "

    # Empty matrices
    Q1_data = ca.SX.zeros(8, t.shape[0] + 1)
    Q1_data[0:4, 0] = Q1.get_quat.get[:, 0]
    Q1_data[4:8, 0] = Q1.get_trans.get[:, 0]

    # Empty matrices
    Q2_data = ca.SX.zeros(8, t.shape[0] + 1)
    Q2_data[0:4, 0] = Q2.get_quat.get[:, 0]
    Q2_data[4:8, 0] = Q2.get_trans.get[:, 0]


    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Computing a sequential transformation
        Q3 = Q1 * Q2

        # Obtaining the norm ot the DualQuaternion elements
        Q1_norm = Q1.norm
        Q2_norm = Q2.norm
        Q3_norm = Q3.norm
        print(Q1_norm)
        print(Q2_norm)
        print(Q3_norm)

        # Send Data throught Ros
        quat_1_msg = get_odometry(quat_1_msg, Q1, 'quat_1')
        send_odometry(quat_1_msg, odom_pub_1)

        quat_2_msg = get_odometry(quat_2_msg, Q2, 'quat_2')
        send_odometry(quat_2_msg, odom_pub_2)

        quat_3_msg = get_odometry(quat_3_msg, Q3, 'quat_3')
        send_odometry(quat_3_msg, odom_pub_3)

        # Save information
        Q1_data[0:4, k +1] = Q1.get_quat.get[:, 0]
        Q1_data[4:8, k+1] = Q1.get_trans.get[:, 0]

        Q2_data[0:4, k +1] = Q2.get_quat.get[:, 0]
        Q2_data[4:8, k+1] = Q2.get_trans.get[:, 0]

        # Time restriction Correct
        loop_rate.sleep()

        # Print Time Verification
        toc = rospy.get_time()
        delta = toc - tic
        rospy.loginfo(message_ros + str(delta))

    Q1_data = ca.DM(Q1_data)
    Q1_data = np.array(Q1_data)

    Q2_data = ca.DM(Q2_data)
    Q2_data = np.array(Q2_data)


    # PLot Results
    fig11, ax11, ax21, ax31, ax41 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax21, ax31, ax41, Q1_data[0:4, :], Q2_data[0:4, :], t, "Quaternions Results")
    plt.show()
    return None
if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("DualQuaternions", disable_signals=True, anonymous=True)
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        odomety_topic_3 = "/" + "dual_3" + "/odom"
        odometry_publisher_3 = rospy.Publisher(odomety_topic_3, Odometry, queue_size = 10)
        main(odometry_publisher_1, odometry_publisher_2, odometry_publisher_3)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass