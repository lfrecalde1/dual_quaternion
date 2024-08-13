import numpy as np
import time
import matplotlib.pyplot as plt
import itertools
from fancy_plots import fancy_plots_3, fancy_plots_4, fancy_plots_1
from fancy_plots import plot_states_position, plot_states_quaternion, plot_control_actions, plot_cost_total, plot_control_actions_reference, plot_states_velocity_reference, plot_states_angular_reference
from system_functions import get_euler_angles
from system_functions import send_odom_msg, set_odom_msg, init_marker, set_marker, send_marker_msg
from system_functions import init_marker_ref, set_marker_ref
from acados_template import AcadosOcpSolver, AcadosSimSolver
from nmpc import create_ocp_solver, create_simulation_solver
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from functions import dualquat_from_pose_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, dual_velocity_casadi
from ode_acados import f_rk4_casadi_simple, noise, cost_quaternion_casadi, cost_translation_casadi, error_quat_aux_casadi
from ode_acados import compute_flatness_states
from scipy.io import savemat
from scipy.spatial.transform import Rotation as R



dualquat_from_pose = dualquat_from_pose_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()
f_rk4 = f_rk4_casadi_simple()
cost_quaternion = cost_quaternion_casadi()
cost_translation = cost_translation_casadi()
error_dual_f = error_quat_aux_casadi()

import os
script_dir = os.path.dirname(__file__)
#folder_path = os.path.join(script_dir, 'cost_with_velocities/')
folder_path = os.path.join(script_dir, 'cost_without_velocities/')
# Check if the folder exists, and create it if it doesn't

def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list, odom_pub_1, odom_pub_2, pub_planning, pub_ref,  v_max, a_max, n, initial)-> None:
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

    # Ros time definitions
    hz = int(1/ts)
    loop_rate = rospy.Rate(hz)

    ts_nmpc = 0.01
    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts_nmpc, ts_nmpc)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized states of the system
    x = np.zeros((13, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    # Reference States
    hd, hd_d, qd, w_d, f_d, M_d = compute_flatness_states(t, L,  2, (initial + 1)*0.5)

    # Control actions
    u_planning = np.zeros((4, t.shape[0]), dtype=np.double)
    u_planning[0, :] = f_d
    u_planning[1:4, :] = M_d[0:3, :]


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

    xref[0:3, :] = hd[0:3, :]
    xref[3:6, :] = hd_d[0:3, :]
    xref[6:10, :] = qd[0:4, :]
    xref[10:13, :] = w_d[0:4, :]

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
    ocp = create_ocp_solver(x[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, ts_nmpc)


    # No Cython
    #acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= False, generate= False)

    ocp_simulation = create_simulation_solver(x[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, ts)

    # Integration Drag Model
    #acados_integrator = AcadosSimSolver(ocp_simulation, json_file="acados_simulation_" + ocp_simulation.model.name + ".json", build= True, generate= True)
    acados_integrator = AcadosSimSolver(ocp_simulation, json_file="acados_simumlation_" + ocp_simulation.model.name + ".json", build= False, generate= False)

    # Integration Without Drag
    #acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= True, generate= True)
    #acados_integrator = AcadosSimSolver(ocp, json_file="acados_sim_" + ocp.model.name + ".json", build= False, generate= False)
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
    quat_1_msg = Odometry()
    quat_1_d_msg = Odometry()

    # Set Odom Message
    quat_1_msg = set_odom_msg(quat_1_msg, x[:, 0])
    send_odom_msg(quat_1_msg, odom_pub_1)

    quat_1_d_msg = set_odom_msg(quat_1_d_msg, xref[:, 0])
    send_odom_msg(quat_1_d_msg, odom_pub_2)

    # Marker Message Planning trajectory and reference
    mesh_marker_msg = Marker()
    mesh_marker_msg, aux_trajectory = init_marker(mesh_marker_msg, xref[:, 0])
    ref_marker_msg = Marker()
    ref_marker_msg, aux_trajectory_ref = init_marker_ref(ref_marker_msg, xref[:, 0])

    # Noise
    sigma_x = 0.01
    sigma_y = 0.01
    sigma_z = 0.01
    sigma_theta_x = 0.001
    sigma_theta_y = 0.001
    sigma_theta_z = 0.001
    sigma_vx = 0.0001
    sigma_vy = 0.0001
    sigma_vz = 0.0001
    sigma_wx = 0.0001
    sigma_wy = 0.0001
    sigma_wz = 0.0001
    aux_noise = np.zeros(12)
    aux_noise[0] = sigma_x**2
    aux_noise[1] = sigma_y**2
    aux_noise[2] = sigma_z**2
    aux_noise[3] = sigma_theta_x**2
    aux_noise[4] = sigma_theta_y**2
    aux_noise[5] = sigma_theta_z**2
    aux_noise[6] = sigma_vx**2
    aux_noise[7] = sigma_vy**2
    aux_noise[8] = sigma_vz**2
    aux_noise[9] = sigma_wx**2
    aux_noise[10] = sigma_wy**2
    aux_noise[11] = sigma_wz**2
    uav_white_noise_cov = np.diag(aux_noise)

    # Cost Values
    orientation_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    translation_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    control_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    total_cost = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)

    # KKT conditions
    kkt_values = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)
    sqp_iteration = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    error_quat_no_filter = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)

    error_dual_no_filter = np.array(error_dual_f(xref[6:10, 0], x[6:10, 0])).reshape((4, ))
    if error_dual_no_filter[0] > 0.0:
        xref[6:10, :] = xref[6:10, :]
    else:
        xref[6:10, :] = -xref[6:10, :]
    
    # Loop simulation
    for k in range(0, t.shape[0] - N_prediction):
        tic = rospy.get_time()
        white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)
        print("v_max")
        print(v_max)
        print("a_max")
        print(a_max)

        # Compute cost
        orientation_cost[:, k] = cost_quaternion(xref[6:10, k], x[6:10, k])
        translation_cost[:, k] = cost_translation(xref[0:3, k], x[0:3, k])
        total_cost[:, k] = orientation_cost[:, k] + translation_cost[:, k]
        error_quat_no_filter[:, k] = np.array(error_dual_f(xref[6:10, k], x[6:10, k])).reshape((4, ))
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
        #acados_ocp_solver.options_set("rti_phase", 2)
        acados_ocp_solver.solve()

        # Check Solution since there can be possible errors 
        #acados_ocp_solver.solve()
        stat_fields = ['statistics', 'time_tot', 'time_lin', 'time_sim', 'time_sim_ad', 'time_sim_la', 'time_qp', 'time_qp_solver_call', 'time_reg', 'sqp_iter', 'residuals', 'qp_iter', 'alpha']
        #for field in stat_fields:
        #    print(f"{field} : {acados_ocp_solver.get_stats(field)}")
        print(initial)
        kkt_values[:, k]  = acados_ocp_solver.get_stats('residuals')
        sqp_iteration[:, k] = acados_ocp_solver.get_stats('sqp_iter')
        #print(error_quat_no_filter[:, k])
        # compute gradient

        # Get the control Action
        aux_control = acados_ocp_solver.get(0, "u")
        F[:, k] = aux_control[0]
        M[:, k] = aux_control[1:4]
        u[0, k] = F[:, k]
        u[1:4, k] = M[:, k]

        control_cost[:, k] = np.dot(u[:, k], u[:, k])

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
        #x[:, k+1] = xcurrent
        x[:, k+1] = noise(xcurrent, white_noise)
        #euler[:, k+1] = get_euler_angles(x[6:10, k+1])

        # Send msg to Ros
        quat_1_msg = set_odom_msg(quat_1_msg, x[:, k+1])
        send_odom_msg(quat_1_msg, odom_pub_1)

        quat_1_d_msg = set_odom_msg(quat_1_d_msg, xref[:, k+1])
        send_odom_msg(quat_1_d_msg, odom_pub_2)

        mesh_marker_msg, aux_trajectory = set_marker(mesh_marker_msg, xref[:, k+1], aux_trajectory)
        ref_marker_msg, aux_trajectory_ref = set_marker_ref(ref_marker_msg, xref[:, k+1], aux_trajectory_ref)
        send_marker_msg(mesh_marker_msg, pub_planning)
        send_marker_msg(ref_marker_msg, pub_ref)
        rospy.loginfo(message_ros + str(toc_solver))

    # Normalize cost
    orientation_cost = orientation_cost/1
    translation_cost = translation_cost/1
    control_cost = control_cost/1
    total_cost = total_cost/np.max(total_cost)

    # Results
    # Position
    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, x[0:3, :], xref[0:3, :], t, "Position of the System "+ str(initial), folder_path)

    # Orientation
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_states_quaternion(fig12, ax12, ax22, ax32, ax42, x[6:10, :], xref[6:10, :], t, "Quaternions of the System "+ str(initial), folder_path)

    # Linear Velocities Inertial frame
    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_states_velocity_reference(fig13, ax13, ax23, ax33, x[3:6, :], hd_d[0:3, :], t, "Linear Velocity of the System and Reference "+ str(initial), folder_path)

    # Angular Velocities Body Frame
    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_states_angular_reference(fig14, ax14, ax24, ax34, x[10:13, :], w_d[0:3, :], t, "Angular Velocity of the System and Reference "+ str(initial), folder_path)

    # Control Actions Opti and Flatness
    fig15, ax15, ax25, ax35, ax45 = fancy_plots_4()
    plot_control_actions_reference(fig15, ax15, ax25, ax35, ax45, F, M, f_d, M_d, t, "Control Actions of the System and Reference "+ str(initial), folder_path)

    # Cost Funtion
    fig18, ax18  = fancy_plots_1()
    plot_cost_total(fig18, ax18, orientation_cost, t, "Cost Ori "+ str(initial), folder_path)

    return x, xref, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction, kkt_values, sqp_iteration

def get_random_quaternion(min_angle=0.0, max_angle=np.math.pi):
    axis = np.random.uniform(-1.0, 1.0, 3)
    axis /= np.linalg.norm(axis)
    angle = np.random.uniform(min_angle, max_angle)
    return angle, axis
def get_random_quaternion_complete(number):
    random_rotation = R.random(number)
    
    # Convert the rotation to quaternion format
    quaternion = random_rotation.as_quat()  # Returns (x, y, z, w)
    return quaternion

def get_random_position(min_trans, max_trans, number):
    x = np.random.uniform(min_trans, max_trans, number)
    y = np.random.uniform(min_trans, max_trans, number)
    z = np.random.uniform(min_trans, max_trans, number)
    trans = np.array([x, y, z])
    trans = trans.reshape(trans.shape[1], trans.shape[0])
    return trans

if __name__ == '__main__':
    try: #################################### Simulation  #####################################################
        # Time parameters
        ts = 0.01
        t_f = 30
        t_N = 0.5

        # Parameters of the system  (mass, inertial matrix, gravity)
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        dx = 0.4
        dy = 0.4
        dz = 0.8
        kh = 0.05
        L = [m, Jxx, Jyy, Jzz, g, dx, dy, dz, kh]

        # Initial conditions of the system
        X_total = []
        X_total_aux = []

        #a_max = np.array([1 ,2, 3, 4])*0.3
        #v_max = np.array([1, 1.5, 2, 2.5, 3, 3.5, 4])*1

        a_max = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])*0.3
        v_max = np.array([1])*1

        max_position = 4
        min_position = -4

        # Use itertools.product to get all possible combinations
        combinations = np.array(list(itertools.product(v_max, a_max)))
        print(combinations)
        number_experiments = combinations.shape[0]
        print(number_experiments)

        # Random initial Conditions
        ramdon_quaternions = get_random_quaternion_complete(number_experiments)
        ramdon_positions = get_random_position(min_position, max_position, number_experiments)

        for i_random in range(number_experiments):
            # Reference States
            #pos_0 = np.array([0, 0, 0])
            #pos_0 = np.array([hd[0,0], hd[1, 0], hd[2, 0]])

            # Random Initial positions
            pos_0 = np.array([ramdon_positions[i_random, 0], ramdon_positions[i_random, 1], ramdon_positions[i_random, 2]])
            vel_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
            omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
            #vel_0 = np.array([hd_d[0, 0], hd_d[1, 0], hd_d[2, 0]], dtype=np.double)
            #omega_0 = np.array([w_d[0, 0], w_d[1, 0], w_d[2, 0]], dtype=np.double)

            # Fixed Initial Conditions
            theta_0 = 0.0
            n_0 = np.array([0.0, 0.0, 1.0])
            #quat_0 = np.hstack([np.cos(theta_0 / 2), np.sin(theta_0 / 2) * np.array(n_0)])
            #quat_0 = np.hstack([qd[0,0], qd[1, 0], qd[2, 0], qd[3, 0]])

            # Random initial conditions
            quat_0 = np.array([ramdon_quaternions[i_random, 3], ramdon_quaternions[i_random, 0], ramdon_quaternions[i_random, 1], ramdon_quaternions[i_random, 2]])
            x = np.hstack((pos_0, vel_0, quat_0, omega_0))
            x_aux = np.hstack((pos_0, quat_0, omega_0, vel_0))
            X_total.append(x)
            X_total_aux.append(x_aux)

        # Matrix of initial conditions
        X_total = np.array(X_total)
        X_total_aux = np.array(X_total_aux)

        # Ros Definition
        rospy.init_node("quadrotor",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic_1 = "/" + "dual_1" + "/odom"
        odometry_publisher_1 = rospy.Publisher(odomety_topic_1, Odometry, queue_size = 10)

        odomety_topic_2 = "/" + "dual_2" + "/odom"
        odometry_publisher_2 = rospy.Publisher(odomety_topic_2, Odometry, queue_size = 10)

        planning_topic = "/quadrotor/planning"
        planning_publisher = rospy.Publisher(planning_topic, Marker, queue_size=10, tcp_nodelay=True)

        ref_topic = "/quadrotor/ref"
        ref_publisher = rospy.Publisher(ref_topic, Marker, queue_size=10, tcp_nodelay=True)

        Data_States = []
        Data_reference = []
        Data_F = []
        Data_M = []
        Data_orientation_cost = []
        Data_translation_cost = []
        Data_control_cost = []
        Data_time = []
        Data_N_prediction = []
        Data_KKT = []
        Data_sqp = []

        for k in range(0, X_total.shape[0]):
            # Simulation
            x, xref, F, M, orientation_cost, translation_cost, control_cost, t, N_prediction, kkt_values, sqp_iteration = main(ts, t_f, t_N, X_total[k, :], L, odometry_publisher_1, odometry_publisher_2, planning_publisher, ref_publisher, combinations[k, 0], combinations[k, 1], 1, k)
            Data_States.append(x)
            Data_reference.append(xref)
            Data_F.append(F)
            Data_M.append(M)
            Data_orientation_cost.append(orientation_cost)
            Data_translation_cost.append(translation_cost)
            Data_control_cost.append(control_cost)
            Data_time.append(t)
            Data_N_prediction.append(N_prediction)
            Data_KKT.append(kkt_values)
            Data_sqp.append(sqp_iteration)

        Data_States = np.array(Data_States)
        Data_reference = np.array(Data_reference)
        Data_F = np.array(Data_F)
        Data_M = np.array(Data_M)
        Data_orientation_cost = np.array(Data_orientation_cost)
        Data_translation_cost = np.array(Data_translation_cost)
        Data_control_cost = np.array(Data_control_cost)
        Data_time = np.array(Data_time)
        Data_N_prediction = np.array(Data_N_prediction)
        Data_KKT = np.array(Data_KKT)
        Data_sqp = np.array(Data_sqp)


        # Save Data matlab 
        mdic_x = {"x": Data_States, "xref": Data_reference, "F": Data_F, "M": Data_M, "orientation_cost": Data_orientation_cost,
                "translation_cost": Data_translation_cost, "control_cost": Data_control_cost, "t": Data_time, "N": Data_N_prediction,
                 "x_init": X_total_aux, "KKT": Data_KKT, 'sqp': Data_sqp}
        savemat("Separed_cost" + ".mat", mdic_x)
        #savemat("Separed_cost_without_velocities" + ".mat", mdic_x)
    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass