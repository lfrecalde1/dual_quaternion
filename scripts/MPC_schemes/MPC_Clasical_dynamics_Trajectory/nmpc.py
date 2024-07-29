from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from export_ode_model import quadrotorModel, quadrotorModelDrag
from casadi import Function, MX, vertcat, sin, cos, fabs
import numpy as np

def create_ocp_solver(x0, N_horizon, t_horizon, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min, L, ts)->AcadosOcp:
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
    model, f_d, constraint, f_error, error_quaternion, rotation_inverse, error_quaternion_li, error_quaternion_z = quadrotorModel(L)

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
    #Q[0, 0] = 10.0
    #Q[1, 1] = 8.0
    #Q[2, 2] = 6.0

    Q[0, 0] = 4.5
    Q[1, 1] = 4.5
    Q[2, 2] = 4.5

    Qq = MX.zeros(3, 3)
    #Q[0, 0] = 10.0
    #Q[1, 1] = 8.0
    #Q[2, 2] = 6.0

    Qq[0, 0] = 6.5
    Qq[1, 1] = 6.5
    Qq[2, 2] = 6.5

    Q_l = MX.zeros(3, 3)
    Q_l[0, 0] = 2
    Q_l[1, 1] = 2
    Q_l[2, 2] = 2

    # Control effort using gain matrices
    R = MX.zeros(4, 4)
    R[0, 0] = 20/F_max
    R[1, 1] = 60/tau_1_max
    R[2, 2] = 60/tau_2_max
    R[3, 3] = 60/tau_3_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Position error
    error_position = ocp.p[0:3] - model.x[0:3]
    error_nominal_input = ocp.p[13:17] - model.u[0:4]
    #error_nominal_input = model.u[0:4]

    # Quaternion error
    q_d = ocp.p[6:10]
    q = model.x[6:10]
    error_ori_li = error_quaternion_li(q, q_d, Qq)
    error_ori = error_quaternion(q_d, q)


    # Angular velocities
    w = model.x[10:13] - ocp.p[10:13]
    vi = model.x[3:6] - ocp.p[3:6]
    vb = vi

    #ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li) + 0.2*(w.T@w) + 0.2*(vb.T@vb)
    #ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li) + 0.2*(w.T@w) + 0.2*(vb.T@vb)

    #ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li)
    #ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li) + w.T@w + vb.T@vb

    #ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li)
    #ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li)

    ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 10*(error_ori.T@Q_l@error_ori)
    ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 10*(error_ori.T@Q_l@error_ori)


    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx + nu)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([F_min, tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([F_max, tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
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

def create_simulation_solver(x0, N_horizon, t_horizon, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min, L, ts)->AcadosOcp:
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
    model, f_d, constraint, f_error, error_quaternion, rotation_inverse, error_quaternion_li, error_quaternion_z = quadrotorModelDrag(L)

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
    #Q[0, 0] = 10.0
    #Q[1, 1] = 8.0
    #Q[2, 2] = 6.0

    Q[0, 0] = 4.5
    Q[1, 1] = 4.5
    Q[2, 2] = 4.5

    Qq = MX.zeros(3, 3)
    #Q[0, 0] = 10.0
    #Q[1, 1] = 8.0
    #Q[2, 2] = 6.0

    Qq[0, 0] = 7.5
    Qq[1, 1] = 7.5
    Qq[2, 2] = 7.5

    # Control effort using gain matrices
    R = MX.zeros(4, 4)
    R[0, 0] = 20/F_max
    R[1, 1] = 60/tau_1_max
    R[2, 2] = 60/tau_2_max
    R[3, 3] = 60/tau_3_max

    # Definition of the cost functions (EXTERNAL)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Position error
    error_position = ocp.p[0:3] - model.x[0:3]
    error_nominal_input = ocp.p[13:17] - model.u[0:4]

    # Quaternion error
    q_d = ocp.p[6:10]
    q = model.x[6:10]
    error_ori_li = error_quaternion_li(q, q_d, Qq)

    # Angular velocities
    w = model.x[10:13] - ocp.p[10:13]
    vi = model.x[3:6] - ocp.p[3:6]
    vb = vi

    #ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li) + 0.2*(w.T@w) + 0.2*(vb.T@vb)
    #ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li) + 0.2*(w.T@w) + 0.2*(vb.T@vb)


    #ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li)
    #ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li) + w.T@w + vb.T@vb

    ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1*(error_nominal_input.T @ R @ error_nominal_input) + 1*(error_ori_li)
    ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)+ 1*(error_ori_li)

    # Auxiliary variable initialization
    ocp.parameter_values = np.zeros(nx + nu)

    # Constraints
    ocp.constraints.constr_type = 'BGH'

    # Set constraints
    ocp.constraints.lbu = np.array([F_min, tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([F_max, tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
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