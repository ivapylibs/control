import numpy as np
from controller.SE2.linearSO import linearSO
from Curves import Explicit
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path

import pdb

# Define System Dynamics
nuEq = 100
A = np.diag([1, 1, 1], k=3)
A[4,2] = nuEq
print(A)

B = np.zeros((6,2))
B[3,0] = 1
B[5,1] = 1
print(B)

Q = np.diag(np.array([10, 10, 5, 2, 2, 2]))

linctrl = linearSO(np.zeros((6,1)), np.zeros((2,1)))
K = np.array([[2, 0, 0, 5, 0, 0],[0, 3, 4, 0, 7, 11]])

linctrl.setByCARE(A, B, Q)
#linctrl.noControl()

leom = linearSO.systemDynamics(A, B)


cfS = structure(dt=0.05, odeMethod=niODERK4, controller=linctrl)

#fCirc = lambda t: np.array([[np.sin(t/2)], [(1/2)*np.cos(t/2)], [1-np.cos(t/2)], [(1/2)*np.sin(t/2)]])
pathType = "arc"
if(pathType == "line"):
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return nuEq* np.vstack((t, np.zeros((2,length)), np.ones((1,length)), np.zeros((2,length))))
    xi= np.array([1, 1, 0, 0, 0, 0])
    tspan = [0,10]

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
    tspan = [0,30]
    path = Explicit(arc, tspan=tspan)


#pdb.set_trace()
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(leom, cfS)

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
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["Actual", "Desired"])
plt.axis('square')

plt.figure()
plt.plot(xsol.t, xsol.u.T)
plt.title("Control Inputs")
plt.show()
