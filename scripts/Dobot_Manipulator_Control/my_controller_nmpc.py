
# You may need to import some classes of the controller module
# For instance From controller import Robot, Motor and more
from controller import Robot, PositionSensor, Supervisor
import sys
import time
import numpy as np
from forward_kinematics import forward_kinematics_casadi_link1, forward_kinematics_casadi_link2, forward_kinematics_casadi_link3, forward_kinematics_casadi_link4, forward_kinematics_casadi_link5, forward_kinematics_casadi_link6, forward_kinematics_casadi
from forward_kinematics import jacobian_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, ln_dual_aux_casadi, error_dual_aux_casadi, dual_multiply_aux_casadi, Ad_aux_casadi, conjugate_dual_aux_casadi, rotation_casadi
from functions import dualquat_from_pose_casadi
import casadi as ca
from scipy.spatial.transform import Rotation as R
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control, plot_cost_total, plot_control_actions_joints
from nmpc_acados import create_ocp_solver
from nmpc_acados import create_ocp_solver
from acados_template import AcadosOcpSolver, AcadosSimSolver


# Creating Funtions based on Casadi
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dualquat_from_pose = dualquat_from_pose_casadi()
error_dual = error_dual_aux_casadi()
ln_dual = ln_dual_aux_casadi()
dual_multiply = dual_multiply_aux_casadi()
Ad = Ad_aux_casadi()
conjugate = conjugate_dual_aux_casadi()
rotation_i = rotation_casadi()

forward_kinematics_link1 = forward_kinematics_casadi_link1()
forward_kinematics_link2 = forward_kinematics_casadi_link2()
forward_kinematics_link3 = forward_kinematics_casadi_link3()
forward_kinematics_link4 = forward_kinematics_casadi_link4()
forward_kinematics_link5 = forward_kinematics_casadi_link5()
forward_kinematics_link6 = forward_kinematics_casadi_link6()
forward_kinematics_f = forward_kinematics_casadi()
jacobian = jacobian_casadi()

import os
script_dir = os.path.dirname(__file__)
#folder_path = os.path.join(script_dir, 'cost_with_velocities/')
folder_path = os.path.join(script_dir, 'results/')

def dual_error(dual_d, dual):
    dual_error = error_dual(dual_d, dual)
    ln_e = ln_dual(dual_error)
    rot_e = ln_e[0:4, 0]
    trans_e = ln_e[4:8, 0]
    return trans_e, rot_e

def controller(dual_d, dual, q):
    # Create a diagonal matrix
    J = jacobian(q)
    I = np.eye(8, 8)
    aux = (J@J.T + 0.001*I)
    J_1 = J.T@ca.pinv(aux)
    # Compute error
    dual_error = error_dual(dual_d, dual)
    ln = ln_dual(dual_error)

    wb = ln[1:4, 0]
    vi = ln[5:8, 0]

    wi = rotation_i(dual[0:4], wb)

    w_i_twist = ca.vertcat(0.0, wi, 0.0, vi)

    # Error manifold
    #unit = ca.DM.zeros(8, 1)
    #unit[0, 0] = 1.0

    # Control Actions
    #control = 2*J_1@(unit - dual_error)
    control = 2*J_1@(w_i_twist)
    return control

def get_desired_frame(translation, rotation):

    # Get system Positions and quaternions
    t = translation.getSFVec3f()

    angles_rotation = rotation.getSFRotation()

    # Get axix representation
    r = R.from_rotvec(angles_rotation[3] * np.array([angles_rotation[0], angles_rotation[1], angles_rotation[2]]))
    quaternion = r.as_quat()
    r = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

    # Initial Dualquaternion
    dual_d = dualquat_from_pose(r[0], r[1], r[2],  r[3], t[0], t[1], t[2])
    return dual_d, t, r

def main(robot):
    # A function that executes the algorithm to obtain sensor data and actuate motors
    # An object instante that contains all possible information about the robot
    initial = 0
    # Get time step of the current world
    time_step = int(robot.getBasicTimeStep())

    # Time definition
    t_final = 20

    # Sample time
    t_s = 0.05

    # Definition of the time vector
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    t_N = 0.5
    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + t_s, t_s)
    N_prediction = N.shape[0]

    # Definition of the names of rotational motors
    names_m = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Set and activate motors
    motors = get_motor(robot, names_m)

    # Definition of the names of rotational sensors
    names_s = ['joint1_sensor', 'joint2_sensor', 'joint3_sensor', 'joint4_sensor', 'joint5_sensor', 'joint6_sensor']

    # Set and activate rotational sensors
    sensors = get_sensor_pos(names_s, time_step)

    # Definition of the desired angles for each joint in the manipulator
    q_c = np.zeros((6, t.shape[0] - N_prediction), dtype = np.double)

    # Definition of the desired angular velocities for each joint in the manipulator
    qp_c = np.zeros((6, t.shape[0] - N_prediction), dtype = np.double)

    # Definition of the desired angles for each joint in the manipulator
    q = np.zeros((6, t.shape[0] + 1 - N_prediction), dtype = np.double)

    # Init System
    init_system(robot, motors, [0.0, -0.5, -0.5, 0.5, -0.5, 0.0], time_step, t_s)
    # get initial conditions of the system
    q[:, 0] = get_angular_position(robot, sensors, time_step)

    # Init dual quaternion
    d = np.zeros((8, t.shape[0] + 1 - N_prediction), dtype = np.double)
    d[:, 0] = np.array(forward_kinematics_f(q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0])).reshape((8, ))

    # Init Pose Positions and quaternions
    x =  np.zeros((8, t.shape[0] + 1 - N_prediction), dtype = np.double)
    x[0:4, 0] = np.array(get_trans(d[:, 0])).reshape((4, ))
    x[4:8, 0] = np.array(get_quat(d[:, 0])).reshape((4, ))

    X =  np.zeros((14, t.shape[0] + 1 - N_prediction), dtype = np.double)
    X[0:8, 0] = d[:, 0]
    X[8:14, 0] = q[:, 0]

    u_d = np.zeros((6, t.shape[0]), dtype = np.double)

    # Desired Frame
    desired_node = robot.getFromDef('frame_d')
    translation_field = desired_node.getField('translation')
    rotation_field = desired_node.getField('rotation')

    xd =  np.zeros((7, t.shape[0] +1 - N_prediction), dtype = np.double)
    dual_d, xd[0:3, 0], xd[3:7, 0] = get_desired_frame(translation_field, rotation_field)

    translation_cost = np.zeros((1, t.shape[0]), dtype = np.double)
    orientation_cost = np.zeros((1, t.shape[0]), dtype = np.double)

    # Constraints
    theta_1_max = 10 
    theta_1_min = -10 
    theta_2_max = 10 
    theta_2_min = -10
    theta_3_max = 10 
    theta_3_min = -10 
    theta_4_max = 10 
    theta_4_min = -10 
    theta_5_max = 10 
    theta_5_min = -10 
    theta_6_max = 10 
    theta_6_min = -10

    ## Optimization problem definitions
    ocp = create_ocp_solver(X[:, 0], N_prediction, t_N, theta_1_max, theta_1_min, theta_2_max, theta_2_min, theta_3_max, theta_3_min, theta_4_max, theta_4_min, theta_5_max, theta_5_min, theta_6_max, theta_6_min, t_s)
    #ocp = create_ocp_solver(X[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L, sample_time)
    #acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= False, generate= False)

    # Dimensions of the optimization problem
    x_dim = ocp.model.x.size()[0]
    u_dim = ocp.model.u.size()[0]

    # Reset Solver
    acados_ocp_solver.reset()

    # Initial Conditions optimization problem
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", X[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", u_d[:, 0])

    # Simulation loop
    for k in range(0, t.shape[0] - N_prediction):
         if robot.step(time_step) != -1:
            tic = time.time()
            # Check for shortest path
            error_dual_no_filter = error_dual(dual_d, d[:, k])
            if error_dual_no_filter[0] >= 0.0:
                dual_d = dual_d
            else:
                dual_d = -dual_d


            # Compute the norm of the error
            translation_e, rotation_e = dual_error(dual_d, d[:, k])
            # Compute norm of the error
            orientation_cost[:, k] = rotation_e.T@rotation_e
            translation_cost[:, k] = translation_e.T@translation_e

            # Control Law Acados
            acados_ocp_solver.set(0, "lbx", X[:, k])
            acados_ocp_solver.set(0, "ubx", X[:, k])

            # Desired Trajectory of the system
            for j in range(N_prediction):
                yref = np.array(dual_d).reshape((8, ))
                uref = u_d[:,k+j]
                aux_ref = np.hstack((yref, uref))
                acados_ocp_solver.set(j, "p", aux_ref)

            # Desired Trayectory at the last Horizon
            yref_N = np.array(dual_d).reshape((8, ))
            uref_N = u_d[:,k+N_prediction]
            aux_ref_N = np.hstack((yref_N, uref_N))
            acados_ocp_solver.set(N_prediction, "p", aux_ref_N)

            # Check Solution since there can be possible errors 
            #acados_ocp_solver.options_set("rti_phase", 2)
            acados_ocp_solver.solve()

            # Get the control Action
            aux_control = acados_ocp_solver.get(0, "u")
            print(aux_control.shape)

            # Control law
            #u = controller(dual_d, d[:, k], q[:, k])
            # Compute desired values
            #qp_c[:, k] = [u[0,0], u[1, 0], u[2, 0], u[3, 0], u[4, 0], u[5, 0]]
            qp_c[:, k] = aux_control

            # actuate the rotational motors of the system
            set_motor_vel(motors, qp_c[:, k])

            # Get current states of the system
            q[:, k + 1] = get_angular_position(robot, sensors, time_step)
            d[:, k + 1] = np.array(forward_kinematics_f(q[0, k+1], q[1, k+1], q[2, k+1], q[3, k+1], q[4, k+1], q[5, k+1])).reshape((8, ))
            x[0:4, k + 1] = np.array(get_trans(d[:, k + 1])).reshape((4, ))
            x[4:8, k + 1] = np.array(get_quat(d[:, k + 1])).reshape((4, ))
            X[0:8, k+1] = d[:, k+1]
            X[8:14, k+1] = q[:, k+1]

            # Desired Frame
            dual_d, xd[0:3, k + 1], xd[3:7, k + 1] = get_desired_frame(translation_field, rotation_field)
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
            print("Sample Time")
            print(toc)
    set_motor_vel(motors, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    fig11, ax11, ax12, ax13, ax14 = fancy_plots_4()
    plot_states_quaternion(fig11, ax11, ax12, ax13, ax14, x[4:8, :], xd[3:7, :], t, "Quaternion Results Based On LieAlgebra Cost NMPC "+ str(initial), folder_path)

    fig21, ax21, ax22, ax23 = fancy_plots_3()
    plot_states_position(fig21, ax21, ax22, ax23, x[1:4, :], xd[0:3, :], t, "Position Results Based On LieAlgebra Cost NMPC "+ str(initial), folder_path)

    fig31, ax31  = fancy_plots_1()
    plot_cost_orientation(fig31, ax31, orientation_cost, t, "Cost Ori Based On LieAlgebra Cost NMPC "+ str(initial), folder_path)

    fig41, ax42  = fancy_plots_1()
    plot_cost_translation(fig41, ax42, translation_cost, t, "Cost Translation Based On LieAlgebra Cost NMPC "+ str(initial), folder_path)

    # Control Actions
    fig51, ax51, ax52, ax53, ax54 = fancy_plots_4()
    plot_control_actions_joints(fig51, ax51, ax52, ax53, ax54, qp_c, t, "Control Actions of the System Based On LieAlgebra Cost NMPC "+ str(initial), folder_path)

    return None

def get_motor(robot, name):
    # Motor Configuration
    # INPUTS
    # robot                                             - robot class
    # name                                              - device name  
    # OUTPUT
    # motors                                             - motor devices        
    motor = []
    for k in name:
        motor.append(robot.getDevice(k))
    return motor

def set_motor_pos(motor, pos):
    # Set motor with position control
    # INPUT
    # motor                                             - device motor
    # pos                                               - desired position for each motor
    # OUTPUT
    # None
    size = len(motor)
    for k in range(0, size):
        motor[k].setPosition(pos[k])
    return None

def set_motor_vel(motor, vel):
    # Set motor with velocity control
    # INPUT
    # motor                                             - device motor
    # vel                                               - desired velocity for each motor
    # OUTPUT
    # None
    size = len(motor)
    for k in range(0, size):
        motor[k].setPosition(float('inf'))
        motor[k].setVelocity(vel[k])
    return None

def get_sensor_pos(names, time_step):
    # Configuration of the rotational motor sensors
    # INPUT 
    # names                                              - list of the names for each sensor
    # time_step                                          - sample time simulation
    # OUTPUTS 
    # sensors                                            - A list with different objects for positional sensing
    sensors = []
    for k in names:
        instance = PositionSensor(k)
        instance.enable(time_step)
        sensors.append(instance)
    return sensors

def get_angular_position(robot, sensors, time_step):
    # A Function that enables the acquisition of the angular displacement from each rotational sensor
    # INPUT 
    # robot                                                                 - object instance 
    # sensors                                                               - A list with sensor objects            
    # OUTPUT                                                                
    # q                                                                     - A vector array with the respective information        
    q = np.zeros((6, ), dtype=np.double)
    size = len(sensors)
    if robot.step(time_step) != -1:
        for k in range(0, size):
            data = sensors[k].getValue()
            q[k] = data
    return q
def init_system(robot, motors, q_c, time_step, t_s):
    # Function that moves the robot to an initial configuration
    # INPUT 
    # robot                                        - A robot object that contains all the required information
    # motors                                       - A list with the required motor objects
    # q_c                                          - A vector of desired initial angular values
    # time_step                                    - sample time of the simulation webots
    # ts                                           - sample time of the simulation
    # OUTPUT
    # None
    for k in range(0, 100):
         if robot.step(time_step) != -1:
            tic = time.time()
            print("Init System")
            set_motor_pos(motors, q_c)
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
    return None


if __name__ == '__main__':
    try:
        robot = Supervisor()
        main(robot)
        pass
    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass