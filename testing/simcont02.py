import numpy as np
import math
from ivacontrol.niODERK4 import niODERK4
from ivacontrol.controller.linear import linear, care
from ivacontrol.simController import simController
import matplotlib.pyplot as plt


A      = np.array([[0, 1],[0.15, 0.25]]) 
B      = np.array([[0],[1]])
solver = niODERK4
dt     = 0.01
tspan  = [0, 20]
rtol   = 4e-3
Q = 50*np.diag([6.00, 3.50])
[P, L, K]  = care(A, B, Q)
L = L.reshape((2,1))
K      = -K
bg     = 1
istate = np.array([[0],[0]])
#cons.sat.min = -5
#cons.sat.max =  5

#==[2] Specify the trajectory to track.
ra = 1
rw = 1.5
rsig = lambda t: np.array([[ra*np.sin(rw*t)],[(ra*rw)*np.cos(rw*t)]])
xacc = lambda t: -(ra*rw*rw)*np.sin(rw*t)
sFF = 1
uFF = lambda t: sFF * bg * (xacc(t))
stateDep = False

#==[3] Trajectory Tracker
theController = linear()
theController.set(K)
#ceom = simController.linearControlSys(A, B)
ceom = theController.systemDynamics(A,B)
solver = solver(ceom, dt)
#theController = controller.linear()
ctrl = theController.tracker(rsig,uFF,stateDep)
csim = simController(solver, ctrl)
#csim.initialize(tspan, istate)
sol = csim.simulate(tspan,istate)

#%==[4] Plot outcomes.
plt.figure(1)
plt.plot(sol.t, sol.x[0,:], 'b', sol.t, sol.x[1,:],'g-')

#  hold on;
#  plot(sol.t, rsig(sol.t), ':');
#  grid on;
#  hold off;

plt.figure(2);
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, uFF(sol.t),'g-.');
#grid on;
plt.show()
