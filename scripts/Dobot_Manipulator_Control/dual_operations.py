
# You may need to import some classes of the controller module
# For instance From controller import Robot, Motor and more
from controller import Robot, PositionSensor, Supervisor
import sys
import time
import numpy as np
from forward_kinematics import forward_kinematics_casadi_link1, forward_kinematics_casadi_link2, forward_kinematics_casadi_link3, forward_kinematics_casadi_link4, forward_kinematics_casadi_link5, forward_kinematics_casadi_link6, forward_kinematics_casadi
from forward_kinematics import jacobian_casadi
from ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, ln_dual_aux_casadi, error_dual_aux_casadi, dual_multiply_aux_casadi, Ad_aux_casadi, conjugate_dual_aux_casadi
from functions import dualquat_from_pose_casadi
import casadi as ca
from scipy.spatial.transform import Rotation as R
from dual_quaternion import plot_states_quaternion, plot_states_position, fancy_plots_4, plot_angular_velocities, plot_linear_velocities, fancy_plots_3, plot_control_actions, fancy_plots_1, plot_time, plot_cost_orientation, plot_cost_translation, plot_cost_control, plot_cost_total

# Creating Funtions based on Casadi
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dualquat_from_pose = dualquat_from_pose_casadi()
error_dual = error_dual_aux_casadi()
ln_dual = ln_dual_aux_casadi()
dual_multiply = dual_multiply_aux_casadi()
Ad = Ad_aux_casadi()
conjugate = conjugate_dual_aux_casadi()


# Robot Kinematics
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

def controller(dual_d, dual, q):
    # Create a diagonal matrix
    J = jacobian(q)
    I = np.eye(8, 8)
    aux = (J@J.T + 0.0005*I)
    J_1 = J.T@ca.pinv(aux)
    # Compute error
    dual_error = error_dual(dual_d, dual)
    unit  = ca.DM.zeros(8, 1)
    unit[0,0] = 1.0

    # Euclidean Space
    ln_e = unit - dual_error

    # Control Actions
    control = J_1@ln_e
    return control
def dual_error(dual_d, dual):
    dual_error = error_dual(dual_d, dual)
    ln_e = ln_dual(dual_error)
    rot_e = ln_e[0:4, 0]
    trans_e = ln_e[4:8, 0]
    return trans_e, rot_e

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
    return dual_d

def main(robot):
    # A function that executes the algorithm to obtain sensor data and actuate motors
    # An object instante that contains all possible information about the robot
    # Get time step of the current world
    time_step = int(robot.getBasicTimeStep())

    # Time definition
    t_final = 40

    # Sample time
    t_s = 0.05

    initial = 0

    # Definition of the time vector
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # Desired Frame
    desired_node = robot.getFromDef('frame_d')
    translation_desired_field = desired_node.getField('translation')
    rotation_desired_field = desired_node.getField('rotation')

    # Current frame
    desired_node = robot.getFromDef('frame_1')
    translation_field = desired_node.getField('translation')
    rotation_field = desired_node.getField('rotation')

    dual_d = get_desired_frame(translation_desired_field, rotation_desired_field)
    dual_1 = get_desired_frame(translation_field, rotation_field)

    translation_cost = np.zeros((1, t.shape[0]), dtype = np.double)
    orientation_cost = np.zeros((1, t.shape[0]), dtype = np.double)

    # Simulation loop
    for k in range(0, t.shape[0]):
         if robot.step(time_step) != -1:
            tic = time.time()
            dual_d = get_desired_frame(translation_desired_field, rotation_desired_field)
            dual_1 = get_desired_frame(translation_field, rotation_field)
            translation_e, rotation_e = dual_error(dual_d, dual_1)
            print("Translation error")
            print(translation_e.T@translation_e)
            print(translation_e)

            print("Rotation error")
            print(rotation_e.T@rotation_e)
            print(rotation_e)

            # Compute norm
            orientation_cost[:, k] = rotation_e.T@rotation_e
            translation_cost[:, k] = translation_e.T@translation_e
            
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
            print("Sample Time")
            print(toc)

    fig11, ax11  = fancy_plots_1()
    plot_cost_orientation(fig11, ax11, orientation_cost, t, "Cost Ori Based On LieAlgebra Cost "+ str(initial), folder_path)

    fig12, ax12  = fancy_plots_1()
    plot_cost_translation(fig12, ax12, translation_cost, t, "Cost Translation Based On LieAlgebra Cost "+ str(initial), folder_path)
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