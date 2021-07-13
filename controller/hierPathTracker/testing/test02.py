import numpy as np
from controller.PathTracker.timepoints import timepoints
from controller.linear import linear
from Curves import Explicit, Bezier, Flight2D
import linepath
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path

from controller.SynthTracker.FixedHorizons import fixedHorizons
from trajSynth.mpcSecondOrder import mpcSecondOrder


#test of hierpathtracker using bezier paths
#set up the system for simulation

A = np.zeros((4,4))
A[0,2] = 1
A[1,3] = 1
B = np.zeros((4,2))
B[2,0] = 1
B[3,1] = 1
Q = np.eye(4)

linctrl = linear(np.array([[0],[0],[0],[0]]), np.array([[0],[0]]))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)

#set up the MPC

#define MPC paramaters
parms = structure()
parms.Ts = .01
parms.x0 = np.array([[0],[0],[0],[0]])
parms.Td = .5
tSynth = mpcSecondOrder(parms)

ts = structure()
ts.Th = .5
ts.Td = .5
ts.Ts = .01

#linctrl.trackerPath(desTraj)
fixedHorizonscontroller=fixedHorizons(tSynth, linctrl, ts)

#Set up Timepoints


pathgen = Flight2D()
pathgen.bezier = Bezier(3)
pathgen.setDynConstraints(0, 10, 4)

ts = structure()
ts.Th = 1
ts.Td = 1
ts.vec2state = lambda x: x



cfS = structure(dt=0.1, odeMethod=niODERK4, controller=timepoints(pathgen, fixedHorizonscontroller, ts))

tspan = [0,1]
fCirc = lambda t: np.array([[np.sin(t/2)] , [1-np.cos(t/2)],[1/2*np.cos(t/2)],[1/2*np.sin(t/2)]])
path = Explicit(fCirc, tspan=tspan)
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

#pdb.set_trace()
istate.x = path.x(0)


sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))

#plt.figure()
#plt.plot(xsol.t, xsol.x[0,:], 'b')
#plt.plot(xsol.t, xdes[0,:], 'g--')
#plt.title("X position tracking")

#plt.figure()
#plt.plot(xsol.t, xsol.x[1,:], 'b')
#plt.plot(xsol.t, xdes[1,:], 'g--')
#plt.title("Y position tracking")

plt.figure(2)
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["Actual", "Desired"])

#plt.axis('square')
plt.show()
