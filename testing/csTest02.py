import numpy as np
import math
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
sys.tspan = [0, 20]
sys.rtol = 4e-3

Q = 50 * np.diag([6.00, 3.50])
[P, L, K] = care(sys.A, sys.B, Q)
sys.K = -K

#L = L.reshape((2, 1))
sys.bg = 1

sys.cons.sat.min = -5
sys.cons.sat.max = 5


#--[1.2] Instantiate the trajectory synthesizer.
metaBuilder = structure(regulator=naive.trajBuilder_LinearReg, tracker=naive.trajBuilder_LinearTracker)

tMaker = naive(theSystem=sys, metaBuilder=metaBuilder)

#==[2] Specify the trajectory to track.

#--[2.1] Simulation parameters.
tspan = np.array([0, 20])

#--[2.2] Signal to track and feed-forward if using.
ra = 0.75
rw = 1.0/3

rsig = lambda t: np.vstack((ra * np.sin(rw*t),
                            (ra * rw)*np.cos(rw * t)))
xacc = lambda t: -(ra*rw*rw)*np.sin(rw*t)

sFF = 1                                # Preferred since not following
uFF = lambda t: sFF * sys.bg * (xacc(t))    # passive dynamics

desTraj = structure()
desTraj.x = rsig
desTraj.u = uFF
desTraj.statedep = False
desTraj.tspan = tspan


#==[3] Configure and instantiate the trajectory tracking builder.
act = structure
act.A = np.array([[0, 1],[0.35, 0.05]])
act.B = np.array([[0],[1]])
ceom = simController.linearControlSys(A=act.A, B=act.B)

cfs = structure()
cfs.dt=0.05
cfs.odeMethod=niODERK4
cfs.controller=linear()
cfs.controller.set(sys.K)

tTracker =cfs.controller.structBuilder(ceom, cfs)

#==[4] Pack together into a controlSystem object and simulate.
cSim = controlSystem(ceom=ceom, trajGenerator=tMaker, trackBuilder=tTracker)

istate = structure(x=np.array([0,0]).reshape((2,1)))
cSim.setInitialState(istate)
sol = cSim.follow(desTraj)

#==[4.1] Plot outcomes
plt.figure(1)
plt.plot(sol.t, sol.x[0,:], sol.t, sol.x[1,:])
rsigv = rsig(sol.t)
plt.plot(sol.t, rsigv.transpose(), ':')

plt.figure(2)
uv = np.array(sol.traj.u(sol.t))
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, uv[0,:], 'g-.')

#==[5] Run again from previous terminal point.
istate.x = np.array([[0.1],[0]])
cSim.setInitialState(istate)
sol = cSim.follow(desTraj=desTraj)

#--[5.1] Plot outcomes.
plt.figure(3)
plt.plot(sol.t, sol.x.transpose())

plt.figure(4)
uv = np.array(sol.traj.u(sol.t))
plt.plot(sol.t, sol.u[0,:], 'b', sol.t, uv[0,:], 'g-.')

plt.show()