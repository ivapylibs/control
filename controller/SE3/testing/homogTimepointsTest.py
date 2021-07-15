#========================== homogTimepointsTest ===========================
#
# @brief    Tests a combination of SE3 control and timepoints path
# generation in 3D.
#
# Tracking a curve broken into bezier segments in 3d, using first order 
# dynamics in SE3.
#
#========================== homogTimepointsTest ===========================
#
#
# @file     homogTimepointsTest.py
#
# @author   Varun Madabushi,       vmadabushi3@gatech.edu
# @date     2021/07/14             [created]
#
# @quitf
#========================== homogTimepointsTest ===========================


import numpy as np
import Lie.group.SE3.Homog
from Lie.tangent import Element
from controller.SE3.linear import linear as linearSE3
from numIntegrator.group.simFirstOrder import SimFirstOrder
from simController import simController
import matplotlib.pyplot as plt
from controller.PathTracker.timepoints import timepoints
import Curves.Explicit
import trajectory.Path
from Curves.Flight3D import FlightOptParams, Flight3D
from structures import structure

import pdb

# Controller and group setup

theGroup = Lie.group.SE3.Homog
theCEOM = linearSE3.systemDynamics()

dt = 0.10

theSolver = SimFirstOrder(theGroup, theCEOM, dt)

def mapToSE3(x):
    vel = x[3:6]
    acc = x[6:]
    tan = vel / np.linalg.norm(vel)
    normal = np.cross(np.squeeze(vel), np.squeeze(acc))[:, np.newaxis]
    normal = normal/np.linalg.norm(normal)
    binorm = np.cross(np.squeeze(tan), np.squeeze(normal))[:, np.newaxis]

    R = np.hstack((tan, normal, binorm))

    return Lie.group.SE3.Homog(x=x[0:3], R=R)

theController = linearSE3(xeq=theGroup(), ueq=np.zeros((6,1)))
theController.set(10*np.eye(6))

# Path Gen Setup

bpOrd = 3
pathgen = Flight3D(bezierOrder=bpOrd)
pathgen.setDynConstraints(0, 20, 4)

pathgen.optParams.Wlen   = 0
pathgen.optParams.Wcurv  = 0
pathgen.optParams.Wkdev  = 10
pathgen.optParams.Wspdev = 1
pathgen.optParams.Wagree = 0
pathgen.optParams.doTimeWarp = False
pathgen.spec.vec2state = mapToSE3

ts = structure()
ts.Th = 0.5
ts.Td = 0.2

def vec2Tangent(x):
    g = mapToSE3(x)
    v = np.vstack((x[3:6], np.zeros((3,1))))
    return Element(g, v)

ts.vec2state = vec2Tangent

# Path Specification

tSpan = [0, 10]
curveType = 'carousel'

if(curveType == 'circ'):
    w0 = 1/2
    a0 = 1
    def circle(t):
        rad = 10
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return a0*np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), np.ones((1,length)), \
            w0*np.cos(w0*t), w0*np.sin(w0*t), np.zeros((1,length)), \
            -(w0**2)*np.sin(w0*t), (w0**2)*np.cos(w0*t), np.zeros((1,length))))
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

        return a0*np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), vz*t,\
            w0*np.cos(w0*t), w0*np.sin(w0*t),  vz*np.ones((1,length)),\
            -(w0**2)*np.sin(w0*t), (w0**2)*np.cos(w0*t), np.zeros((1,length))))
    fCurve = spiral
elif(curveType == 'carousel'):
    #theta = np.pi/4
    w0 = 1/4
    zAmplitude = 0.1
    zw0 = 3/4
    def carousel(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return np.vstack((np.sin(w0*t), (1-np.cos(w0*t)), zAmplitude*np.sin(zw0*t),\
             w0*np.cos(w0*t), w0*np.sin(w0*t), zAmplitude*zw0*np.cos(zw0*t),\
            -(w0**2)*np.sin(w0*t), (w0**2)*np.cos(w0*t), -zAmplitude*(zw0**2)*np.sin(zw0*t)))
    fCurve = carousel

path = Curves.Explicit(fCurve, tSpan)
desTraj = trajectory.Path(path, tSpan)
theSolver = SimFirstOrder(theGroup, theCEOM, dt)
tPtsController = timepoints(pathgen, theController, ts)
theController.tracker(desTraj)
tPtsController.tracker(desTraj)
theSim = simController(theSolver,  tPtsController)

startT = vec2Tangent(desTraj.x(0))
theSim.initialize(tSpan, startT.base())

sol = theSim.simulate()

plt.figure(1)
ax = plt.axes(projection='3d')
t = np.linspace(tSpan[0], tSpan[1], 10)
tplt = np.linspace(tSpan[0], tSpan[1], 100)
xDes = desTraj.x(tplt)
ax.plot(xDes[0,:], xDes[1,:], xDes[2,:])
plt.title("Desired frames")
for i in t:
    mapToSE3(desTraj.x(i)).plot(ax=ax, scale=0.5)
ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])



fig = plt.figure(2)
ax = plt.axes(projection='3d')
ax.plot(xDes[0,:], xDes[1,:], xDes[2,:])
tDisc = np.floor(np.linspace(0, len(sol.t), 21))[:-1]
for i in tDisc:
    sol.x[int(i)].plot(ax=ax, scale=0.2)
ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
plt.title("Tracking")
plt.show()