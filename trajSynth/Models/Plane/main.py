#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import time

from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator

""" User settings: """
show_animation = False
store_results = False

"""
Get configured do-mpc modules:
"""

model = template_model()
mpc = template_mpc(model)
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)

"""
Set initial state
"""

X_s_0 = 2.0 # This is the initial concentration inside the tank [mol/l]
Y_s_0 = 0.0 # This is the controlled variable [mol/l]
Z_s_0 = 0.0
V_s_0 = 0.0 #[C]
Psi_s_0 = 0.0 #[C]
x0 = np.array([X_s_0, Y_s_0, Z_s_0,V_s_0])


mpc.x0 = x0
simulator.x0 = x0
estimator.x0 = x0

mpc.set_initial_guess()

"""
Setup graphic:
"""

#fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(8,5))
#plt.ion()
Ts = .01
"""
Run MPC main loop:
"""
ref = np.array([0])
for k in range(150):
    u0 = mpc.make_step(x0)
    t = Ts*k
    ref = np.append(ref,t)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)


    if show_animation:
        graphics.plot_results(t_ind=k)
        graphics.plot_predictions(t_ind=k)
        graphics.reset_axes()
        plt.show()
        plt.pause(0.01)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(mpc.data['_x'][:,0],mpc.data['_x'][:,1],mpc.data['_x'][:,2],'b')
#plt.plot(ref,ref,'g--')
#print(ref)
#print(mpc.data['_x'])
plt.show()
#graphics.default_plot(states_list =['X_s','Y_s'])

input('Press any key to exit.')

# Store results:
if store_results:
    do_mpc.data.save_results([mpc, simulator], 'DiffDriveMPC')
