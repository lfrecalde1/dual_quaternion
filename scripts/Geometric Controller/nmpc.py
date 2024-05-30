
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from export_ode_model import quadrotorModel
from casadi import Function, MX, vertcat, sin, cos, fabs
import numpy as np

def create_ocp_solver(x0, N_horizon, t_horizon, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min, L, ts)->AcadosOcp:
    # Creation of the optimal control problem
    # INPUTS
    # x0                                                                                 - Initial condition
    # N_horizon                                                                          - Prediction horizon in steps
    # t_horizon                                                                          - Prediction horizon in seconds
    # tau_1_max, tau_1_min                                                               - maximum and minimum torque 1
    # tau_2_max, tau_2_min                                                               - maximum and minimum torque 2
    # tau_3_max, tau_3_min                                                               - maximum and minimum torque 3
    # L                                                                                  - Parameters of the system (mass, inertial matrix, gravity)
    # OUTPUT
    # ocp                                                                                - optimal control problem (acados)

    # Optimal control problem class
    ocp = AcadosOcp()

    # Model of the system
    model, f_d, constraint, error_quaternion, adjoint, quaternion_conjugate, quaternion_error, ln = quadrotorModel(L)

    # Constructing the optimal control problem
    ocp.model = model

    # Auxiliary variable for the reference states of the system
    ocp.p = model.p

    # Dimensions of the problem
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # Set the dimension of the problem
    ocp.dims.N = N_horizon

    # Gain matrices position error
    Q = MX.zeros(3, 3)
    Q[0, 0] = 2
    Q[2, 2] = 2
    #Q[3, 3] = 2
    Q[1, 1] = 2

    J = MX.zeros(3, 3)
    J[0, 0] = L[0]
    J[1, 1] = L[1]
    J[2, 2] = L[2]

    # Control effort using gain matrices
    R = MX.zeros(3, 3)
    R[0, 0] = 0.05/tau_1_max
    R[1, 1] = 0.05/tau_2_max
    R[2, 2] = 0.05/tau_3_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Position error
    error_nominal_input = ocp.p[7:10] - model.u[0:3]

    # Quaternion error
    q_d = ocp.p[0:4]
    q = model.x[0:4]

    w_d = ocp.p[4:7]
    w = model.x[4:7]

    error_ori = quaternion_error(q_d, q)
    error_ln = ln(error_ori)
    error_ori_T = quaternion_error(q, q_d)
    aux_error = error_ori - error_ori_T


    error_q_c = quaternion_conjugate(error_ori)
    error_dot = w - adjoint(error_q_c, w_d)


    ocp.model.cost_expr_ext_cost = 1000*(error_ln.T @ Q @ error_ln) + 2*(error_dot.T @ error_dot) + (error_nominal_input.T @ R @ error_nominal_input)
    ocp.model.cost_expr_ext_cost_e = 1000*(error_ln.T @ Q @error_ln)+ 2*(error_dot.T @ error_dot)
    #ocp.model.cost_expr_ext_cost = 1000*(aux_error.T @ Q @ aux_error) + 15*(error_dot.T @ error_dot) + (error_nominal_input.T @ R @ error_nominal_input)
    #ocp.model.cost_expr_ext_cost_e = 1000*(aux_error.T @ Q @ aux_error)+ 15*(error_dot.T @ error_dot)


    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx + nu)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.x0 = x0

    # Nonlinear constraints
    ocp.model.con_h_expr = constraint.expr
    nsbx = 0
    nh = constraint.expr.shape[0]
    nsh = nh
    ns = nsh + nsbx

    # Gains over the Horizon for the nonlinear constraint
    cost_weights = np.ones((ns, ))
    ocp.cost.zl = 100*np.ones((ns, ))
    ocp.cost.Zl = 100*np.ones((ns, ))
    ocp.cost.Zu = 100*np.ones((ns, ))
    ocp.cost.zu = 100*np.ones((ns, ))

    # Norm of a quaternion should be one
    ocp.constraints.lh = np.array([constraint.min])
    ocp.constraints.uh = np.array([constraint.max])
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))

    # Set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" 
    ocp.solver_options.qp_solver_cond_N = N_horizon // 4
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  
    ocp.solver_options.regularize_method = "CONVEXIFY"  
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.Tsim = ts
    ocp.solver_options.tf = t_horizon

    return ocp