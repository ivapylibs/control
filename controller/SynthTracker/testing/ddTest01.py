import numpy as np
from controller.SynthTracker.FixedHorizons import fixedHorizons
from controller.TSE2.linear import linear as linearSO
from Curves import Explicit
from trajSynth.mpcDiff import mpcDiff
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import math
import trajectory.Path







#need to set the dymanics
def nlDyn(t,x,u):
    sdA = np.diag([1,1,1],k=3)
    #print(sdA)
    sdB = np.zeros((6,2))
    sdB[3,0] = math.cos(x[2])
    sdB[4,0] = math.sin(x[2])
    sdB[5,1] = 1
    a = np.matmul(sdA,x) + np.matmul(sdB,u)
    return a



#define System Dynamics
nuEq = 100
A = np.diag([1, 1, 1], k=3)
A[4,2] = nuEq
#print(A)
#input()
B = np.zeros((6,2))
B[3,0] = 1
B[5,1] = 1
#print(B)
#input()
linctrl = linearSO(np.array([0, 0, 0, nuEq, 0, 0]).reshape((6,1)), np.zeros((2,1)))
linctrl.noControl()
linctrl.setBodyFrame(0)
#Q = np.diag(np.array([10, 4, 5, 2, 2, 2]))

#linctrl.setByCARE(A, B, Q)
K = np.array([[2, 0, 0, 5, 0, 0],[0, 3, 4, 0, 7, 11]])

#linctrl.setByCARE(A, B, Q)
linctrl.set(K)
#set system Dynamics

leom = nlDyn

tspan = [0,10]
#fCirc = lambda t: np.array([ [np.sin(t/2)],[1-np.cos(t/2)],[np.arctan2(np.cos(t/2)-np.cos((t+.01)/2),np.sin((t+.01)/2)-np.sin(t/2))]])
fCirc = lambda t: np.array([[nuEq*np.sin(t/2)],[nuEq*(1-np.cos(t/2))],[t/2]])
path = Explicit(fCirc, tspan=tspan)
desTraj = trajectory.Path(path, tspan)

#define MPC paramaters
parms = structure()
parms.Ts = .01
parms.x0 = np.array([[0],[0],[0]])
parms.Td = .3
tSynth = mpcDiff(parms)

ts = structure()
ts.Th = .5
ts.Td = .3
ts.Ts = .01

tSynth.updatefPtr(desTraj.x)
#linctrl.trackerPath(desTraj)
cfS = structure(dt=0.01, odeMethod=niODERK4, controller=fixedHorizons(tSynth, linctrl, ts))



sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

#pdb.set_trace()
istate.x = np.array([[0],[0],[0],[0],[0],[0]])
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
plt.plot(xsol.t, xdes[0,:], 'g--')
#plt.title("X position tracking")

plt.plot(xsol.t, xsol.x[1,:], 'y')
plt.plot(xsol.t, xdes[1,:], 'r--')
plt.title("X and Y position tracking")
plt.legend(["X","Xdes","Y","Ydes"])

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["Actual", "Desired"])

plt.axis('square')
plt.show()
