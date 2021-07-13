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

    R = np.zeros([2,2])
    R[0,0] = math.cos(x[2])
    R[0,1] = -math.sin(x[2])
    R[1,0] = math.sin(x[2])
    R[1,1] = math.cos(x[2])
    xi = np.matmul(R.T,x[3:5])
    xi = np.vstack([xi, x[5]])

    A = np.zeros([3,1])
    B = xi[0]*xi[2]*R[:,1]

    LagF = np.vstack([np.zeros([3,1]),np.vstack(xi[0]*xi[2]*R[:,1]),0])
    #print(sdB)
    #input()
    a = np.matmul(sdA,x) + np.matmul(sdB,u) + LagF
    return a



#define System Dynamics
nuEq = 20
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

tspan = [0,50]
rad = 2
#fCirc = lambda t: np.array([ [np.sin(t/2)],[1-np.cos(t/2)],[np.arctan2(np.cos(t/2)-np.cos((t+.01)/2),np.sin((t+.01)/2)-np.sin(t/2))]])
fCirc = lambda t: np.array([[nuEq*np.sin(t/rad)],[nuEq*(1-np.cos(t/rad))],[t/rad]])
path = Explicit(fCirc, tspan=tspan)
desTraj = trajectory.Path(path, tspan)

#define MPC paramaters
parms = structure()
parms.Ts = .1
parms.x0 = np.array([[0],[0],[0]])
parms.Td = .10
tSynth = mpcDiff(parms)

ts = structure()
ts.Th = .10
ts.Td = .10
ts.Ts = .1

tSynth.updatefPtr(desTraj.x)
#linctrl.trackerPath(desTraj)
cfS = structure(dt=ts.Ts, odeMethod=niODERK4, controller=fixedHorizons(tSynth, linctrl, ts))



sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

#pdb.set_trace()
istate.x = np.array([[nuEq*np.sin(tspan[0]/10)],[nuEq*(1-np.cos(tspan[0]/10))],[tspan[0]/10],[0],[0],[0]])
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

plt.plot(xsol.t, xdes[0], 'g--')
plt.title("X position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[1,:], 'b')
plt.plot(xsol.t, xdes[1], 'g--')
plt.title("Y position tracking")

plt.figure()
plt.plot(xsol.t,xsol.x[2,:],'b')
plt.plot(xsol.t,xdes[2],'g--')
plt.title('Theta Tracking')

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
#plt.plot(xdes[1,:], xdes[0,:], 'g--')
plt.legend(["Actual", "Desired"])

plt.figure()
plt.plot(xsol.t,xsol.u[0,:],'b')
plt.plot(xsol.t,xsol.u[1,:],'r')
plt.title('Control Inputs')


#plt.axis('square')
plt.show()
