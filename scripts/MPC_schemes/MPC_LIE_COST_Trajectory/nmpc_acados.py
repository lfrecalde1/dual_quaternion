from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from ode_acados import quadrotorModel
from export_ode_model import quadrotorModelDrag
from casadi import Function, MX, vertcat, sin, cos, fabs, DM
import casadi as ca

import numpy as np

def create_ocp_solver(x0, N_horizon, t_horizon, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min,  L, ts)->AcadosOcp:
    # Creation of the optimal control problem

    # Optimal control problem class
    ocp = AcadosOcp()

    # Model of the system
    model, get_trans, get_quat, constraint, error_lie_2, dual_error, ln, Ad, conjugate, rotation = quadrotorModel(L)

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
    R = MX.zeros(4, 4)
    R[0, 0] = 20/F_max
    R[1, 1] = 60/tau_1_max
    R[2, 2] = 60/tau_2_max
    R[3, 3] = 60/tau_3_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Desired Dual Quaternion
    dual_d = ocp.p[0:8] 

    # Current Dual Quaternion
    dual = model.x[0:8]

    error_total_lie = error_lie_2(dual_d, dual)
    error = dual_error(dual_d, dual)
    error_c = conjugate(error)
    ln_error = ln(error)

    # Inputs
    nominal_input = ocp.p[14:18]
    #error_nominal_input = model.u[0:4]
    error_nominal_input = nominal_input - model.u[0:4]

    # Angular velocities
    w_b = model.x[8:11]
    v_b = model.x[11:14]
    v_i = rotation(model.x[0:4], v_b)

    w_b_d = ocp.p[8:11]
    v_i_d = ocp.p[11:14]
    error_w = w_b - w_b_d
    error_v = v_i - v_i_d

    # Gain Matrix complete error

    Q_l = MX.zeros(6, 6)
    Q_l[0, 0] = 2
    Q_l[1, 1] = 2
    Q_l[2, 2] = 2
    Q_l[3, 3] = 1.6
    Q_l[4, 4] = 1.6
    Q_l[5, 5] = 1.6
    #Q_l[6, 6] = 1.6
    #Q_l[7, 7] = 1.6

    ocp.model.cost_expr_ext_cost = 10*(error_total_lie.T@Q_l@error_total_lie) + 1*(error_nominal_input.T @ R @ error_nominal_input)  + 0.2*(error_w.T@error_w) + 0.2*(error_v.T@error_v)
    ocp.model.cost_expr_ext_cost_e =  10*(error_total_lie.T@Q_l@error_total_lie) + 0.2*(error_w.T@error_w) + 0.2*(error_v.T@error_v)


    #ocp.model.cost_expr_ext_cost = 10*(error_total_lie.T@Q_l@error_total_lie) + 1*(error_nominal_input.T @ R @ error_nominal_input)
    #ocp.model.cost_expr_ext_cost_e =  10*(error_total_lie.T@Q_l@error_total_lie)


    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx + nu)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([F_min, tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([F_max, tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
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

def create_simulation_solver(x0, N_horizon, t_horizon, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min,  L, ts)->AcadosOcp:
    # Creation of the optimal control problem

    # Optimal control problem class
    ocp = AcadosOcp()

    # Model of the system
    model, get_trans, get_quat, constraint, error_lie_2, dual_error, ln, Ad, conjugate, rotation = quadrotorModel(L)

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
    R = MX.zeros(4, 4)
    R[0, 0] = 20/F_max
    R[1, 1] = 60/tau_1_max
    R[2, 2] = 60/tau_2_max
    R[3, 3] = 60/tau_3_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Desired Dual Quaternion
    dual_d = ocp.p[0:8] 

    # Current Dual Quaternion
    dual = model.x[0:8]

    error_total_lie = error_lie_2(dual_d, dual)
    error = dual_error(dual_d, dual)
    error_c = conjugate(error)
    ln_error = ln(error)

    # Inputs
    nominal_input = ocp.p[14:18]
    #error_nominal_input = model.u[0:4]
    error_nominal_input = nominal_input - model.u[0:4]

    # Angular velocities
    w_b = model.x[8:11]
    v_b = model.x[11:14]
    v_i = rotation(model.x[0:4], v_b)

    w_b_d = ocp.p[8:11]
    v_i_d = ocp.p[11:14]
    error_w = w_b - w_b_d
    error_v = v_i - v_i_d

    # Gain Matrix complete error

    Q_l = MX.zeros(6, 6)
    Q_l[0, 0] = 2
    Q_l[1, 1] = 2
    Q_l[2, 2] = 2
    Q_l[3, 3] = 2.5
    Q_l[4, 4] = 2.5
    Q_l[5, 5] = 2.5
    #Q_l[6, 6] = 1.6
    #Q_l[7, 7] = 1.6

    ocp.model.cost_expr_ext_cost = 10*(error_total_lie.T@Q_l@error_total_lie) + 1*(error_nominal_input.T @ R @ error_nominal_input)  + 0.2*(error_w.T@error_w) + 0.2*(error_v.T@error_v)
    ocp.model.cost_expr_ext_cost_e =  10*(error_total_lie.T@Q_l@error_total_lie) + 0.2*(error_w.T@error_w) + 0.2*(error_v.T@error_v)


    #ocp.model.cost_expr_ext_cost = 10*(error_total_lie.T@Q_l@error_total_lie) + 1*(error_nominal_input.T @ R @ error_nominal_input)
    #ocp.model.cost_expr_ext_cost_e =  10*(error_total_lie.T@Q_l@error_total_lie)


    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx + nu)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([F_min, tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([F_max, tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
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