import numpy as np
import math
import sys
from niODERK4 import niODERK4
from controller.linear import linear, care
from simController import simController
import matplotlib.pyplot as plt
from trajSynth.naive import naive
from controlSystem import controlSystem
from structures import structure

sys = structure()

sys.A = np.array([[0, 1], [0.15, 0.25]])
sys.B = np.array([[0], [1]])
sys.solver = niODERK4
sys.dt = 0.01
sys.tspan = [0, 10]
sys.rtol = 4e-3
Q = 50 * np.diag([6.00, 3.50])
[P, L, K] = care(sys.A, sys.B, Q)
sys.K = -K
L = L.reshape((2, 1))
sys.cons.sat.min = -5
sys.cons.sat.max = 5


#--[1.2] Instantiate the trajectory synthesizer.
metaBuilder = structure(regulator=naive.trajBuilder_LinearReg, tracker=naive.trajBuilder_LinearTracker)

tMaker = naive(theSystem=sys, metaBuilder=metaBuilder)

#==[2] Configure and instantiate the trajectory tracking builder.
actA      = np.array([[0, 1],[0.35, 0.05]])
actB      = np.array([[0],[1]])
ceom = linear().systemDynamics(actA,actB)


theController = linear()
theController.set(sys.K)
cfs = structure(dt=0.05, odeMethod=niODERK4, controller=theController)

tTracker =theController.structBuilder(ceom, cfs)


#%==[3] Pack together into a controlSystem object and simulate.
cSim = controlSystem(ceom, tMaker, tTracker)

istate = structure(x=np.array([3,0]).reshape((2,1)))
cSim.setInitialState(istate)
sol = cSim.goto(np.array([0.3, 0]).reshape((2,1)))

#==[3.1] Plot outcomes
plt.figure(1)
plt.plot(sol.t, sol.x.transpose())
plt.title('Real trajectory starting at [3,0]')

plt.figure(2)
xv = np.array(sol.traj.x(sol.t))
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, xv.transpose(), 'g-.')
plt.title("Control inputs and planned trajectory")


#==[4] Run again from previous terminal point.
istate = sol.fstate
cSim.setInitialState(istate)
sol = cSim.goto(np.array([1.0, 0]).reshape((2,1)))

#--[4.1] Plot outcomes.
plt.figure(3)
plt.plot(sol.t, sol.x.transpose())
plt.title('Real trajectory starting at previous final point')

plt.figure(4)
xv = np.array(sol.traj.x(sol.t))
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, xv.transpose(),'g-.')
plt.title("Control inputs and planned trajectory")

plt.show()