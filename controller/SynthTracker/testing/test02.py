import numpy as np
from controller.SynthTracker.FixedHorizons import fixedHorizons
from controller.SE2.linearSO import linearSO
from Curves import Explicit
from trajSynth.mpcDiff import mpcDiff
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path


A = np.zeros((2,2))
B = np.eye(2)
Q = np.eye(2)

linctrl = linearSO(np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[0]]))
linctrl.noControl()

K = np.array([[1,0,0],[0,1,1]])

linctrl.set(K)

#need to set the dymanics
def leom(t,x,u):
    sdA = np.zeros((3,3))
    sdB = np.zeros((3,1))
    sdB[0] = u[0]* np.cos(x[2])
    sdB[1] = u[0] * np.sin([x[2]])
    sdB[2] = 1
    return sdB

tspan = [0,10]
fCirc = lambda t: np.array([ [1-np.cos(t/2)],[np.sin(t/2)],[0]])
path = Explicit(fCirc, tspan=tspan)
desTraj = trajectory.Path(path, tspan)

#define MPC paramaters
parms = structure()
parms.Ts = .01
parms.x0 = np.array([[0],[0],[0]])
parms.Td = 10
tSynth = mpcDiff(parms)

ts = structure()
ts.Th = 10
ts.Td = 10
ts.Ts = .01

tSynth.updatefPtr(desTraj.x)
#linctrl.trackerPath(desTraj)
cfS = structure(dt=0.01, odeMethod=niODERK4, controller=fixedHorizons(tSynth, linctrl, ts))



sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

#pdb.set_trace()
istate.x = path.x(0)
#print(np.shape(istate.x))
#input('enter')
#temp = np.array([[0],[0],[0]])
#istate.x= np.concatenate((istate.x,temp))
#print(istate.x)
#input("enter")
sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))

plt.figure()
plt.plot(xsol.t, xsol.x[0,:], 'b')
#plt.plot(xsol.t, xdes[0,:], 'g--')
plt.title("X position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[1,:], 'b')
#plt.plot(xsol.t, xdes[1,:], 'g--')
plt.title("Y position tracking")

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
#plt.plot(xdes[1,:], xdes[0,:], 'g--')
plt.legend(["Actual", "Desired"])

plt.axis('square')
plt.show()
