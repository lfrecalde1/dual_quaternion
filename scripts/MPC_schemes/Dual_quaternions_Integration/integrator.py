from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from ode_acados import quadrotorModel
from casadi import Function, MX, vertcat, sin, cos, fabs
import numpy as np

def create_integral_solver(L, ts)->AcadosSim:
    sim = AcadosSim()

    # Model of the system
    model, get_trans, get_quat, constraint = quadrotorModel(L)

    # Constructing the optimal control problem
    sim.model = model


    # Dimensions of the problem
    nx = model.x.size()[0]
    nu = model.u.size()[0]


    # Set options
    sim.solver_options.integrator_type = "ERK"
    #sim.solver_options.num_stages = 3
    #sim.solver_options.num_steps = 3
    #sim.solver_options.newton_iter = 3 # for implicit integrator
    #sim.solver_options.Tsim = ts
    sim.solver_options.T = ts

    return sim
