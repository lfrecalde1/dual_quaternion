import numpy as np
import time
import matplotlib.pyplot as plt
from fancy_plots import fancy_plots_3, fancy_plots_4, fancy_plots_1
from fancy_plots import plot_states_position, plot_states_quaternion, plot_control_actions, plot_states_euler, plot_time
from system_functions import f_d, axisToquaternion, f_d_casadi, ref_trajectory, compute_desired_quaternion, get_euler_angles
from system_functions import send_odom_msg, set_odom_msg, init_marker, set_marker, send_marker_msg, ref_trajectory_agresive
from system_functions import init_marker_ref, set_marker_ref
from export_ode_model import quadrotorModel
from acados_template import AcadosOcpSolver, AcadosSimSolver
from nmpc import create_ocp_solver
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker



def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list, pub, pub_planning, pub_ref)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # OUTPUS 
    # x                         - states of the system
    # F                         - force control action
    # M                         - torques of the system
    # delta_t                   - computational time

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)
    N_planning = t.shape[0]-1

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)


    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized states of the system
    x = np.zeros((13, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    x_planning = np.zeros((13, t.shape[0]), dtype=np.double)
    x_planning[:, 0] = x_0
    u_planning = np.zeros((4, t.shape[0]), dtype=np.double)
    u_planning[0, :] = L[0] * L[4]

    # Euler angles of the system
    euler = np.zeros((3, t.shape[0] + 1 - N_prediction), dtype=np.double)
    euler[:, 0] = get_euler_angles(x[6:10, 0])

    # Control actions
    F = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized control actions
    u = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)

    # Desired states
    xref = np.zeros((13, t.shape[0]), dtype = np.double)
    # Desired position of the system
    #xd, yd, zd, theta, theta_p = ref_trajectory(t)
    xd, yd, zd, theta, theta_p = ref_trajectory_agresive(t, 20)

    xref[0, :] = 0.0
    xref[1, :] = 0.0
    xref[2, :] = 0.0

    # Compute desired quaternion
    q_d = compute_desired_quaternion(theta, theta_p, t, ts)

    # Desired quaternion
    xref[6, :] = 1.0
    xref[7, :] = 0.0
    xref[8, :] = 0.0
    xref[9, :] = 0.0

    # Desired Euler angles
    euler_d = np.zeros((3, t.shape[0]), dtype=np.double)
    euler_d[2, :] = theta

    # Constraints on control actions
    F_max = L[0]*L[4] + 20
    F_min = 0
    tau_1_max = 0.1
    tau_1_min = -0.1
    tau_2_max = 0.1
    tau_2_min = -0.1
    tau_3_max = 0.1
    taux_3_min = -0.1

    # Optimization problem
    ocp = create_ocp_solver(x[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, ts)


    # No Cython
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)

    
    # Integration using Acados
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json")
    # Auxiliary variables and control
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # Reset Solver
    acados_ocp_solver.reset()


    # Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", x[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", u_planning[:, 0])
    # Ros message
    rospy.loginfo_once("Quadrotor Simulation")
    message_ros = "Quadrotor Simulation "

    # Odometry message
    odom_msg = Odometry()

    # Set Odom Message
    odom_msg = set_odom_msg(odom_msg, x[:, 0])
    send_odom_msg(odom_msg, pub)

    # Marker Message Planning trajectory and reference
    mesh_marker_msg = Marker()
    mesh_marker_msg, aux_trajectory = init_marker(mesh_marker_msg, xref[:, 0])
    ref_marker_msg = Marker()
    ref_marker_msg, aux_trajectory_ref = init_marker_ref(ref_marker_msg, xref[:, 0])

    # Loop simulation
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        acados_ocp_solver.options_set("rti_phase", 1)
        acados_ocp_solver.solve()
        # Control Law Acados
        acados_ocp_solver.set(0, "lbx", x[:, k])
        acados_ocp_solver.set(0, "ubx", x[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = xref[:,k+j]
            uref = u_planning[:,k+j]
            aux_ref = np.hstack((yref, uref))
            acados_ocp_solver.set(j, "p", aux_ref)

        yref_N = xref[:,k+N_prediction]
        uref_N = u_planning[:,k+N_prediction]
        aux_ref_N = np.hstack((yref_N, uref_N))
        acados_ocp_solver.set(N_prediction, "p", aux_ref_N)

        # Check Solution since there can be possible errors 
        acados_ocp_solver.options_set("rti_phase", 2)
        acados_ocp_solver.solve()

        # Check Solution since there can be possible errors 
        #acados_ocp_solver.solve()
        stat_fields = ['time_tot', 'time_lin', 'time_qp', 'time_qp_solver_call', 'time_reg', 'sqp_iter']
        for field in stat_fields:
            print(f"{field} : {acados_ocp_solver.get_stats(field)}")

        # Get the control Action
        aux_control = acados_ocp_solver.get(0, "u")
        F[:, k] = aux_control[0]
        M[:, k] = aux_control[1:4]
        u[0, k] = F[:, k]
        u[1:4, k] = M[:, k]

        # run time
        loop_rate.sleep()
        toc_solver = rospy.get_time() - tic
        delta_t[:, k] = toc_solver

        # Get states of the system using acados (ERK)
        acados_integrator.set("x", x[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral = acados_integrator.solve()
        xcurrent = acados_integrator.get("x")

        # System evolution
        x[:, k+1] = xcurrent
        euler[:, k+1] = get_euler_angles(x[6:10, k+1])

        # Send msg to Ros
        odom_msg = set_odom_msg(odom_msg, x[:, k+1])
        mesh_marker_msg, aux_trajectory = set_marker(mesh_marker_msg, xref[:, k+1], aux_trajectory)
        ref_marker_msg, aux_trajectory_ref = set_marker_ref(ref_marker_msg, xref[:, k+1], aux_trajectory_ref)
        send_odom_msg(odom_msg, pub)
        send_marker_msg(mesh_marker_msg, pub_planning)
        send_marker_msg(ref_marker_msg, pub_ref)
        rospy.loginfo(message_ros + str(toc_solver))

    # Results
    # Position
    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, x[0:3, :], xref[0:3, :], t, "Position of the System")
    plt.show()

    # Orientation
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_states_quaternion(fig12, ax12, ax22, ax32, ax42, x[6:10, :], xref[6:10, :], t, "Quaternions of the System")
    plt.show()

    # Control Actions
    fig13, ax13, ax23, ax33, ax43 = fancy_plots_4()
    plot_control_actions(fig13, ax13, ax23, ax33, ax43, F, M, t, "Control Actions of the System")
    plt.show()

    # Sampling time
    fig14, ax14  = fancy_plots_1()
    plot_time(fig14, ax14, t_sample, delta_t, t, "Computational Time")
    plt.show()

    #fig14, ax14, ax24, ax34 = fancy_plots_3()
    #plot_states_euler(fig14, ax14, ax24, ax34, euler[0:3, :], euler_d[0:3, :], t, "Euler Angles of the System")
    #plt.show()
    #None

if __name__ == '__main__':
    try: #################################### Simulation  #####################################################
        # Time parameters
        ts = 0.03
        t_f = 30
        t_N = 0.3

        # Parameters of the system  (mass, inertial matrix, gravity)
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Initial conditions of the system
        pos_0 = np.array([-4.0, -4.0, 4.0], dtype=np.double)
        vel_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
        angle_0 = 3.8134
        axis_0 = [0.4896, 0.2032, 0.8480]
        quat_0 = axisToquaternion(angle_0, axis_0)
        omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)

        # Generalized vector of initial conditions
        x_0 = np.hstack((pos_0, vel_0, quat_0, omega_0))

        # Ros Definition
        rospy.init_node("quadrotor",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/quadrotor/odom"
        odometry_publisher = rospy.Publisher(odomety_topic, Odometry, queue_size = 1, tcp_nodelay=True)

        planning_topic = "/quadrotor/planning"
        planning_publisher = rospy.Publisher(planning_topic, Marker, queue_size=10, tcp_nodelay=True)

        ref_topic = "/quadrotor/ref"
        ref_publisher = rospy.Publisher(ref_topic, Marker, queue_size=10, tcp_nodelay=True)

        # Simulation
        main(ts, t_f, t_N, x_0, L, odometry_publisher, planning_publisher, ref_publisher)

    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass