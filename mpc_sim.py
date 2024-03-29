import do_mpc
import numpy as np
from casadi import *
from do_mpc.data import save_results, load_results

def run_mpc_simulation(sim_parameters):
    x0 = np.array(sim_parameters["pocz_poz"])
    desired_lvl = sim_parameters["zad_poz"]
    Tp = sim_parameters["Tp"]
    sim_time = float(sim_parameters['czas_sym'])

    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    h = model.set_variable(var_type='_x', var_name='h')
    dh = model.set_variable(var_type='_z', var_name='dh')
    Q_d = model.set_variable(var_type='_u', var_name='Q_d')

    # uncertain parameters
    # A = model.set_variable('parameter', 'A')
    # Beta = model.set_variable('parameter', 'Beta')
    # hardcoded parameters
    A = sim_parameters["A"]
    Beta = sim_parameters["B"]

    model.set_rhs('h', dh)
    euler_lagrange = A*dh + Beta*sqrt(h) - Q_d
    model.set_alg('euler_lagrange', euler_lagrange)

    model.setup()
    mpc = do_mpc.controller.MPC(model)

    # Optimizer paramaters
    setup_mpc = {
        'n_horizon': 5,
        't_step': Tp,
        'n_robust': 1,
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)

    # Objective function
    _x = model.x
    mterm = (_x['h'] - desired_lvl)**2  # terminal cost
    lterm = (_x['h'] - desired_lvl)**2  # stage cost
    mpc.set_objective(mterm=mterm, lterm=lterm)

    # Penality for the control input
    mpc.set_rterm(Q_d=1e-4)

    # Lower bounds on states:
    mpc.bounds['lower', '_x', 'h'] = 0
    # Upper bounds on states
    mpc.bounds['upper', '_x', 'h'] = 10

    # Lower bounds on inputs:
    mpc.bounds['lower', '_u', 'Q_d'] = 0
    # Lower bounds on inputs:
    mpc.bounds['upper', '_u', 'Q_d'] = 10

    mpc.setup()

    estimator = do_mpc.estimator.StateFeedback(model)

    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
        # Note: cvode doesn't support DAE systems.
        'integration_tool': 'idas',
        'abstol': 1e-10,
        'reltol': 1e-10,
        't_step': Tp
    }
    simulator.set_param(**params_simulator)
    simulator.setup()

    simulator.x0 = x0
    mpc.x0 = x0
    estimator.x0 = x0
    mpc.set_initial_guess()

    simulator.reset_history()
    simulator.x0 = x0
    mpc.reset_history()

    n = round(sim_time / Tp)   # number of steps equal to PID simulation
    for i in range(n):
        u0 = mpc.make_step(x0)
        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)

    save_results([mpc], overwrite=True)
    results = load_results('./results/results.pkl')
    h_results = results['mpc']['_x']
    u_results = results['mpc']['_u']

    mpc_results = {
        "h": h_results,
        "u": u_results
    }

    return mpc_results


