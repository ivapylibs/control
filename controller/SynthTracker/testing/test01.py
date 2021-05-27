import numpy as np
from controller.SynthTracker.FixedHorizons import fixedHorizons
from controller.linear import linear
from Curves import Explicit
from trajSynth.mpcDiff import mpcDiff
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4


A = np.zeros((2,2))
B = np.eye(2)
Q = np.eye(2)

linctrl = linear(np.array([[0],[0]]), np.array([[0],[0]]))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)

#define MPC paramaters
parms = structure()
parms.Ts = .01
parms.x0 = np.array([[0],[0]])
parms.Td = .02
tSynth = mpcTrivial(parms)

ts = structure()
ts.Th = 0.5
ts.Td = 0.2

cfS = structure(dt=0.05, odeMethod=niODERK4, controller=SynthTracker(tSynth, linctrl, ts))

tspan = [0,10]
fCirc = lambda t: np.array([[np.sin(t/2)] , [1-np.cos(t/2)]])
path = Explicit(fCirc, tspan=tspan)
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

#pdb.set_trace()
istate.x = path.x(0)

sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))

plt.figure()
plt.plot(xsol.t, xsol.x[0,:], 'b')
plt.plot(xsol.t, xdes[0,:], 'g--')
plt.title("X position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[1,:], 'b')
plt.plot(xsol.t, xdes[1,:], 'g--')
plt.title("Y position tracking")

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["Actual", "Desired"])

plt.axis('square')
plt.show()
