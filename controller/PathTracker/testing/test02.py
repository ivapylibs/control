import numpy as np
from controller.PathTracker.timepoints import timepoints
from controller.linear import linear
from Curves import Explicit
import linepath
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path

import pdb

A = np.zeros((4,4))
A[0,1] = 1
A[2,3] = 1

B = np.zeros((4,2))
B[1,0] = 1
B[3,1] = 1

Q = np.eye(4)


linctrl = linear(np.zeros((4,1)), np.zeros((2,1)))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)


pathgen = linepath.linepath()

ts = structure()
ts.Th = 0.5
ts.Td = 0.2
ts.vec2state = lambda x: x

cfS = structure(dt=0.05, odeMethod=niODERK4, controller=timepoints(pathgen, linctrl, ts))

tspan = [0,20]
fCirc = lambda t: np.array([[np.sin(t/2)], [(1/2)*np.cos(t/2)], [1-np.cos(t/2)], [(1/2)*np.sin(t/2)]])
path = Explicit(fCirc, tspan=tspan)
#pdb.set_trace()
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

istate.x = path.x(0)

sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))

plt.figure()
plt.plot(xsol.t, xsol.x[0,:], 'b')
plt.plot(xsol.t, xdes[0,:], 'g--')
plt.title("X position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[2,:], 'b')
plt.plot(xsol.t, xdes[2,:], 'g--')
plt.title("Y position tracking")

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[2,:], 'b')
plt.plot(xdes[0,:], xdes[2,:], 'g--')
plt.legend(["Actual", "Desired"])
plt.axis('square')
plt.show()
