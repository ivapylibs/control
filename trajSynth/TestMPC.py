# This is a test for testing the mpcDiff class that implements an MPC tracking
# this is primarily to test the followPath method and the file itself works similiarly to
# how fixedHorizons would work.

import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import do_mpc

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import time

from structures import structure
from trajSynth.mpcDiff import mpcDiff
from Models.DiffDrive.template_model import template_model

""" User settings: """
"""
Get configured do-mpc modules:
"""

"""
Set initial state
"""

X_s_0 = 0.0 # This is the initial concentration inside the tank [mol/l]
Y_s_0 = 0.0 # This is the controlled variable [mol/l]
#V_s_0 = 0.0 #[C]
T_s_0 = 0.0 #[C]
x0 = np.array([X_s_0, Y_s_0, T_s_0])

model = template_model()
"""
Setup graphic:
"""
def desTraj(t):
    myarr =[0,0]
    myarr[0] = 1-cos(1/2*t)
    myarr[1] = sin(1/2*t)
    return myarr
#fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(8,5))
#plt.ion()
Ts = .01
#define the parameters
param = structure()
param.x0 = x0
param.Td = 1
param.Ts = Ts
#define MPC objects
myMPC = mpcDiff(param)

#graphics.default_plot(states_list =['X_s','Y_s'])
#this is basically a test of MPC fixed Horizons just in an unclean way
fig, ax = plt.subplots()
ax.set_prop_cycle(color =['red', 'black', 'yellow'])
for i in range(10):
    myX = myMPC.followPath(x0,desTraj)
    x0 = myX[-1,:]
    plt.plot(myMPC.mpc.data['_x'][:,0],myMPC.mpc.data['_x'][:,1])

plt.show()
input('Press any key to exit.')

# Store results:
if store_results:
    do_mpc.data.save_results([mpc, simulator], 'DiffDriveMPC')
