import numpy as np
from controller.PathTracker.timepoints import timepoints
from controller.linear import linear
from Curves import Explicit, Bezier, Flight2D
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path

import pdb

A = np.zeros((4,4))
A[0,2] = 1
A[1,3] = 1
print(A)

B = np.zeros((4,2))
B[2,0] = 1
B[3,1] = 1
print(B)

Q = np.eye(4)


linctrl = linear(np.zeros((4,1)), np.zeros((2,1)))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)


#pathgen = linepath.linepath()
pathgen = Flight2D()
pathgen.bezier = Bezier(3)
pathgen.setDynConstraints(0, 10, 4)

ts = structure()
ts.Th = 0.5
ts.Td = 0.2
ts.vec2state = lambda x: x

cfS = structure(dt=0.05, odeMethod=niODERK4, controller=timepoints(pathgen, linctrl, ts))

tspan = [0,20]

curveType = 'circ'
if(curveType == 'circ'):
    w0 = 1/4
    a0 = 20
    def arc(t):
        rad = 10
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return a0*np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), w0*np.cos(w0*t), w0*np.sin(w0*t)))
    fCurve = arc
elif(curveType == 'line'):
    theta = np.pi/4
    v = 8
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        xc = np.cos(theta)
        yc = np.sin(theta)
        out = np.vstack((v*xc*t, v*yc*t, v*xc*np.ones((1,length)), v*yc*np.ones((1,length))))
        return out
    fCurve = line

path = Explicit(fCurve, tspan=tspan)
#pdb.set_trace()
desTraj = trajectory.Path(path, tspan)

sm = cfS.controller.simBuilder(leom, cfS)

istate = structure()

istate.x = path.x(0)

sim = sm.firstBuild(istate, desTraj)
xsol = sim.simulate()
xdes = np.squeeze(desTraj.x(xsol.t))

plt.figure()
plt.plot(xsol.t, xsol.x[0,:])
plt.plot(xsol.t, xdes[0,:], '--')
plt.plot(xsol.t, xsol.x[1,:])
plt.plot(xsol.t, xdes[1,:], '--')
plt.title("Position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[2,:])
plt.plot(xsol.t, xdes[2,:], '--')
plt.plot(xsol.t, xsol.x[3,:])
plt.plot(xsol.t, xdes[3,:], '--')
plt.title("Velocity")

plt.figure()
plt.plot(xsol.x[0,:], xsol.x[1,:], 'b')
plt.plot(xdes[0,:], xdes[1,:], 'g--')
plt.legend(["Actual", "Desired"])
plt.axis('square')
plt.show()
