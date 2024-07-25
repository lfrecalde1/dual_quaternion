from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from ode_acados import quadrotorModel
from casadi import Function, MX, vertcat, sin, cos, fabs, DM
import casadi as ca

import numpy as np

def create_ocp_solver(x0, N_horizon, t_horizon, theta_1_max, theta_1_min, theta_2_max, theta_2_min, theta_3_max, theta_3_min, theta_4_max, theta_4_min, theta_5_max, theta_5_min, theta_6_max, theta_6_min, ts)->AcadosOcp:
    # Creation of the optimal control problem

    # Optimal control problem class
    ocp = AcadosOcp()

    # Model of the system
    model, get_trans, get_quat, constraint, dual_error, ln = quadrotorModel()

    # Constructing the optimal control problem
    ocp.model = model

    # Dimension of the problem
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # Set the dimension of the problem
    ocp.p = model.p
    ocp.dims.N = N_horizon

    # Control effort using gain matrices
    R = MX.zeros(6, 6)
    R[0, 0] = 20/theta_1_max
    R[1, 1] = 20/theta_2_max
    R[2, 2] = 20/theta_3_max
    R[3, 3] = 20/theta_4_max
    R[4, 4] = 20/theta_5_max
    R[5, 5] = 20/theta_6_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Desired Dual Quaternion
    dual_d = ocp.p[0:8] 

    # Current Dual Quaternion
    dual = model.x[0:8]

    error = dual_error(dual_d, dual)
    ln_error = ln(error)

    # Inputs
    nominal_input = ocp.p[8:14]
    error_nominal_input = nominal_input - model.u[0:6]

    # Gain Matrix complete error
    Q_l = MX.zeros(6, 6)
    Q_l[0, 0] = 0.1
    Q_l[1, 1] = 0.1
    Q_l[2, 2] = 0.1
    Q_l[3, 3] = 2.0
    Q_l[4, 4] = 2.0
    Q_l[5, 5] = 2.0

    # Cost Funtion
    ocp.model.cost_expr_ext_cost = 10*(ln_error.T@Q_l@ln_error) + 0.05*(error_nominal_input.T @ R @ error_nominal_input)
    ocp.model.cost_expr_ext_cost_e =  10*(ln_error.T@Q_l@ln_error)

    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([theta_1_min, theta_2_min, theta_3_min, theta_4_min, theta_5_min, theta_6_min])
    ocp.constraints.ubu = np.array([theta_1_max, theta_2_max, theta_3_max, theta_4_max, theta_5_max, theta_6_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5])
    ocp.constraints.x0 = x0

    ## Nonlinear constraints
    ocp.model.con_h_expr = constraint.expr
    nsbx = 0
    nh = constraint.expr.shape[0]
    nsh = nh
    ns = nsh + nsbx
#
    ### Gains over the Horizon for the nonlinear constraint
    ocp.cost.zl = 100*np.ones((ns, ))
    ocp.cost.Zl = 100*np.ones((ns, ))
    ocp.cost.Zu = 100*np.ones((ns, ))
    ocp.cost.zu = 100*np.ones((ns, ))
#
    ### Norm of a quaternion should be one
    ocp.constraints.lh = np.array([constraint.min, constraint.min2])
    ocp.constraints.uh = np.array([constraint.max, constraint.max2])
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))
#
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