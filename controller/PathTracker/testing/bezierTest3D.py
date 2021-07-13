import numpy as np
from controller.PathTracker.timepoints import timepoints
from controller.linear import linear
from Curves import Explicit, Bezier, Flight3D
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4
import trajectory.Path
import Lie.group.SE3.Homog
from Lie.tangent import Element

import pdb

A = np.diag([1, 1, 1], k=3)
print(A)

B = np.zeros((6,3))
B[3:,:] = np.eye(3)
print(B)

Q = np.eye(6)


linctrl = linear(np.zeros((6,1)), np.zeros((3,1)))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)


#pathgen = linepath.linepath()
pathgen = Flight3D()
pathgen.bezier = Bezier(3)
pathgen.setDynConstraints(0, 10, 4)

ts = structure()
ts.Th = 0.5
ts.Td = 0.2
def vec2Tangent(x):
    print(x)
    pos = x.copy()[0:3]
    pos[-1] = 0
    #pdb.set_trace()
    center = np.array([0,1,0]).reshape((3,1))
    r = center - pos # Radially directed 
    unitDir = x[3:]/np.linalg.norm(x[3:])
    z = r/np.linalg.norm(r)
    y = np.cross(np.squeeze(unitDir), np.squeeze(z)).reshape((3,1))

    R = np.hstack((unitDir, y, z))

    g = Lie.group.SE3.Homog(x=x[0:3], R=R)
    return Element(g, x[3:])

ts.vec2state = vec2Tangent

cfS = structure(dt=0.05, odeMethod=niODERK4, controller=timepoints(pathgen, linctrl, ts))

tspan = [0,20]

curveType = 'carousel'

if(curveType == 'circ'):
    w0 = 1/4
    a0 = 1
    def circle(t):
        rad = 10
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return a0*np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), np.ones((1,length)), w0*np.cos(w0*t), w0*np.sin(w0*t), np.zeros((1,length))))
    fCurve = circle
elif(curveType == 'spiral'):
    #theta = np.pi/4
    w0 = 1/4
    a0 = 1
    vz = 1
    def spiral(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return a0*np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), vz*t, w0*np.cos(w0*t), w0*np.sin(w0*t),  vz*np.ones((1,length))))
    fCurve = spiral
elif(curveType == 'carousel'):
    #theta = np.pi/4
    w0 = 1/4
    zAmplitude = 0.1
    zw0 = 3/4
    def spiral(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), zAmplitude*np.sin(zw0*t), w0*np.cos(w0*t), w0*np.sin(w0*t), zAmplitude*zw0*np.cos(zw0*t)))
    fCurve = spiral

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
plt.plot(xsol.t, xsol.x[2,:])
plt.plot(xsol.t, xdes[2,:], '--')
plt.title("Position tracking")

plt.figure()
plt.plot(xsol.t, xsol.x[3,:])
plt.plot(xsol.t, xdes[3,:], '--')
plt.plot(xsol.t, xsol.x[4,:])
plt.plot(xsol.t, xdes[4,:], '--')
plt.plot(xsol.t, xsol.x[5,:])
plt.plot(xsol.t, xdes[5,:], '--')
plt.title("Velocity")

plt.figure()
ax = plt.axes(projection='3d')
ax.plot(xsol.x[0,:], xsol.x[1,:], xsol.x[2], 'b')
plt.plot(xdes[0,:], xdes[1,:], xdes[2,:], 'g--')
plt.legend(["Actual", "Desired"])
#plt.axis('square')
plt.show()
