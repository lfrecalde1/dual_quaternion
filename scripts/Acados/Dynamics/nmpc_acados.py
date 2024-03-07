from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from ode_acados import quadrotorModel
from casadi import Function, MX, vertcat, sin, cos, fabs
import numpy as np

def create_ocp_solver(N_horizon, t_horizon, L, ts)->AcadosOcp:
    # Creation of the optimal control problem
    # INPUTS
    # x0                                                                                 - Initial condition
    # N_horizon                                                                          - Prediction horizon in steps
    # t_horizon                                                                          - Prediction horizon in seconds
    # F_max, F_min                                                                       - maximum and minimum force
    # tau_1_max, tau_1_min                                                               - maximum and minimum torque 1
    # tau_2_max, tau_2_min                                                               - maximum and minimum torque 2
    # tau_3_max, tau_3_min                                                               - maximum and minimum torque 3
    # L                                                                                  - Parameters of the system (mass, inertial matrix, gravity)
    # OUTPUT
    # ocp                                                                                - optimal control problem (acados)

    # Optimal control problem class
    ocp = AcadosOcp()

    # Model of the system
    model = quadrotorModel(L)

    # Constructing the optimal control problem
    ocp.model = model
    ocp.dims.N = N_horizon

    # Auxiliary variable for the reference states of the system

    # Dimensions of the problem

    # Set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM" 
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.Tsim = ts
    ocp.solver_options.levenberg_marquardt = 0.01
    ocp.solver_options.sim_method_num_stages = 3
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.sim_method_newton_iter = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4


    ocp.solver_options.tf = t_horizon


    return ocp