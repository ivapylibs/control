import numpy as np
import math
from niODERK4 import niODERK4
from controller.linear import linear, care
from simController import simController
import matplotlib.pyplot as plt
from trajSynth.naive import naive
from controlSystem import controlSystem
from structures import ConsStruct

A      = np.array([[0, 1],[0.15, 0.25]])
B      = np.array([[0],[1]])
solver = niODERK4
dt     = 0.01
tspan  = [0, 10]
rtol   = 4e-3
Q = 50*np.diag([6.00, 3.50])
[P, L, K]  = care(A, B, Q)
L = L.reshape((2,1))
K      = -K
bg     = 1
istate = np.array([[0],[0]])
cons = ConsStruct()
cons.sat.min = -5
cons.sat.max =  5

sys = {'A': A, 'B': B, 'solver': solver, 'dt': dt, 'tspan': tspan, 'rtol': rtol, 'K': K, 'istate': istate, 'cons': cons}

#--[1.2] Instantiate the trajectory synthesizer.
metaBuilder = {'regulator': naive.trajBuilder_LinearReg, 'tracker': naive.trajBuilder_LinearTracker}

tMaker = naive(theSystem=sys, metaBuilder=metaBuilder)

#==[2] Configure and instantiate the trajectory tracking builder.
actA      = np.array([[0, 1],[0.35, 0.05]])
actB      = np.array([[0],[1]])
ceom = linear().systemDynamics(actA,actB)


theController = linear()
theController.set(K)
cfs = {'dt': 0.05, 'odeMethod': niODERK4, 'controller': theController}

tTracker =theController.structBuilder(ceom, cfs)


#%==[3] Pack together into a controlSystem object and simulate.
cSim = controlSystem(ceom, tMaker, tTracker);

istate = {'x': np.array([3,0]).reshape((2,1))}
cSim.setInitialState(istate);
sol = cSim.goto(np.array([0.3, 0]).reshape((2,1)))

#==[3.1] Plot outcomes
plt.figure(1)
plt.plot(sol.t, sol.u[0,:], sol.t, sol.x[0,:])

plt.figure(2)
xv = np.array(sol.traj.x(sol.t))
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, xv[:,0,:], 'g-.',  sol.t, xv[:,1,:], 'g-.')


#==[4] Run again from previous terminal point.
#istate = sol.fstate
#cSim.setInitialState(istate)
#sol = cSim.goto([1.0, 0])

#--[4.1] Plot outcomes.
#plt.figure(3)
#plt.plot(sol.t, sol.x)

#plt.figure(4)
#plt.plot(sol.t, sol.u, 'b', sol.t, sol.traj.x(sol.t),'g-.')


plt.show()