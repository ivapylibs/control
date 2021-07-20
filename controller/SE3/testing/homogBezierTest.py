#========================== homogBezierTest ===========================
#
# @brief  Tracks bezier curve using SE3 controller and mapToSE3 function
#
#
#========================== homogBezierTest ===========================
#
#
# @file     homogBezierTest.py
#
# @author   Varun Madabushi,       vmadabushi3@gatech.edu
# @date     2021/07/14             [created]
#
# @quitf
#========================== homogBezierTest ===========================

import numpy as np
import Lie.group.SE3.Homog
from controller.SE3.linear import linear as linearSE3
from numIntegrator.group.simFirstOrder import SimFirstOrder
from simController import simController
import matplotlib.pyplot as plt
import Curves.Explicit
from Curves.Flight3D import FlightOptParams, Flight3D

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
theController.ref2group =  mapToSE3

# Bezier Path Specification

vMin =  5
vMax = 10
maxG = 4
ts = 0
tf   = 2
tSpan = [ts, tf]
order = 4

gi = theGroup()

R = np.matmul(np.matmul(theGroup.RotX(np.pi/2), theGroup.RotY(np.pi/3)), theGroup.RotZ(np.pi/4))

x = np.array([[5], [3], [2]])
gf = theGroup(R=R, x=x)

optS1 = FlightOptParams(init=5, final=3)

fp1 = Flight3D(gi, gf, tspan=tSpan, bezierOrder= order, optParams=optS1)
fp1.spec.vec2state = lambda x: x
fp1.optimizeBezierPath()
desTraj = fp1

#--[2.2] Simulate
#
theSim = simController(theSolver,  theController.tracker(desTraj))
theSim.initialize(tSpan, gi)
sol = theSim.simulate()

# Plot
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

plt.figure(2)
ax = plt.axes(projection='3d')
ax.plot(xDes[0,:], xDes[1,:], xDes[2,:])
tDisc = np.floor(np.linspace(0, len(sol.t), 21))[:-1]
for i in tDisc:
    solPos = sol.x[int(i)]
    solPos.plot(ax=ax, scale=0.5)
plt.title("Tracking")
ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
plt.show()