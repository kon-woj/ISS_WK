import math
import do_mpc
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

desired_lvl = 10
model_type = 'continuous'
model = do_mpc.model.Model(model_type)

h = model.set_variable(var_type='_x', var_name='h')
Q_d = model.set_variable(var_type='_u', var_name='Q_d')

# uncertain parameters
# A = model.set_variable('parameter', 'A')
# Beta = model.set_variable('parameter', 'Beta')
# hardcoded parameters
A = 5
Beta = 0.5

model.set_rhs('h', (Q_d - Beta * h) / A)

model.setup()
mpc = do_mpc.controller.MPC(model)

# Optimizer paramaters
setup_mpc = {
    'n_horizon': 10,
    't_step': 0.1,
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
mpc.set_rterm(Q_d=1e-2)

# Lower bounds on states:
mpc.bounds['lower', '_x', 'h'] = 0
# Upper bounds on states
mpc.bounds['upper', '_x', 'h'] = 100

# Lower bounds on inputs:
mpc.bounds['lower', '_u', 'Q_d'] = 0
# Lower bounds on inputs:
mpc.bounds['upper', '_u', 'Q_d'] = 10

mpc.setup()
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=0.1)
simulator.setup()

x0 = np.array(0)
simulator.x0 = x0
mpc.x0 = x0
mpc.set_initial_guess()

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)
fig, ax = plt.subplots(2, sharex=True, figsize=(16,9))
fig.align_ylabels()

mpl.rcParams['font.size'] = 18
mpl.rcParams['axes.grid'] = True

for g in [sim_graphics, mpc_graphics]:
    g.add_line(var_type='_x', var_name='h', axis=ax[0])
    g.add_line(var_type='_u', var_name='Q_d', axis=ax[1])

# u0 = np.zeros((1, 1))
for i in range(100):
    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)

# Plot results until current time
sim_graphics.plot_results()
sim_graphics.reset_axes()
plt.show()
