import numpy as np
from controller.TSE2.linear import linear as linearSO
from Curves import Explicit
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import math
import trajectory.Path

import pdb


def nlDyn(t,x,u):
    sdA = np.diag([1,1,1],k=3)
    #print(sdA)
    sdB = np.zeros((6,2))
    sdB[3,0] = math.cos(x[2])
    sdB[4,0] = math.sin(x[2])
    sdB[5,1] = 1
    a = np.matmul(sdA,x) + np.matmul(sdB,u)
    return a

# Define System Dynamics
nuEq = 100
A = np.diag([1, 1, 1], k=3)
A[4,2] = nuEq
#print(A)

B = np.zeros((6,2))
B[3,0] = 1
B[5,1] = 1
#print(B)

Q = np.diag(np.array([10, 10, 5, 2, 2, 2]))

linctrl = linearSO(np.array([0, 0, 0, nuEq, 0, 0]).reshape((6,1)), np.zeros((2,1)))
K = np.array([[2, 0, 0, 5, 0, 0],[0, 3, 4, 0, 7, 11]])

#linctrl.setByCARE(A, B, Q)
linctrl.set(K)
#linctrl.noControl()

#leom = linearSO.systemDynamics(A, B)


cfS = structure(dt=0.05, odeMethod=niODERK4, controller=linctrl)

#fCirc = lambda t: np.array([[np.sin(t/2)], [(1/2)*np.cos(t/2)], [1-np.cos(t/2)], [(1/2)*np.sin(t/2)]])
pathType = "arc"
if(pathType == "lineX"):
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return nuEq* np.vstack((t, np.zeros((2,length)), np.ones((1,length)), np.zeros((2,length))))
    xi= np.array([0, 10, -np.pi/3, 0, 0, 0])
    tspan = [0,30]
    path = Explicit(line, tspan=tspan)

elif(pathType == 'lineTheta'):
    theta = -0.2
    v = 0.5*nuEq
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        xc = np.cos(theta)
        yc = np.sin(theta)
        out = np.vstack((v*xc*t, v*yc*t, theta*np.ones((1,length)), v*xc*np.ones((1,length)), v*yc*np.ones((1,length)), np.zeros((1,length))))
        return out
    xi= np.array([0, 0, np.pi/4, 0, 0, 0])
    tspan = [0,5]
    path = Explicit(line, tspan=tspan)

elif(pathType == 'arc'):
    def arc(t):
        rad = 10
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return np.vstack((nuEq*rad*np.sin(t/rad), nuEq*rad*(1-np.cos(t/rad)), t/rad, (nuEq)*np.cos(t/rad), nuEq*np.sin(t/rad), np.ones((1,length))/rad))

    xi = np.array([0,0,0,0.7*nuEq,0,0]).reshape((6,1))
    tspan = [0,50]
    path = Explicit(arc, tspan=tspan)

#pdb.set_trace()
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(nlDyn, cfS)

istate = structure()

istate.x = xi

sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))
plt.figure()
plt.plot(xsol.t, xsol.x[0,:], 'b')
plt.plot(xsol.t, xdes[0,:], 'g--')
plt.title("X position tracking")
plt.legend(["actual", "desired"])

plt.figure()
plt.plot(xsol.t, xsol.x[1,:], 'b')
plt.plot(xsol.t, xdes[1,:], 'g--')
plt.title("Y position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[2,:], 'b')
plt.plot(xsol.t, xdes[2,:], 'g--')
plt.title("Theta position tracking")
plt.legend(["actual", "desired"])

plt.figure()
plt.plot(xsol.t, xsol.x[3:6,:].T)
plt.plot(xsol.t, xdes[3:6,:].T)
plt.title("Velocity tracking")
plt.legend(["vx", "vy", "$\omega$", "vx Desired","vy Desired", "$\omega$ desired"])

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["actual", "desired"])
plt.axis('square')

plt.figure()
plt.plot(xsol.t, xsol.u.T)
plt.title("Control Inputs")

plt.show(block=False)
plt.pause(0.001) # Pause for interval seconds.
input("hit[enter] to end.")
plt.close('all') # all open plots are correctly closed after each run
