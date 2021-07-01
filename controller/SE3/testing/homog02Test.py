import numpy as np
import Lie.group.SE3 as SE3
from controller.SE3.linear import linear as linearSE3
from numIntegrator.group.simFirstOrder import SimFirstOrder
from simController import simController
import matplotlib.pyplot as plt

import pdb

'''
Tests regulator ability of the linearSE3 class
'''
theGroup = SE3.SE3
theCEOM = linearSE3.systemDynamics()

dt = 0.10

theSolver = SimFirstOrder(theGroup, theCEOM, dt)

theController = linearSE3(xeq=theGroup(), ueq=np.zeros((6,1)))
theController.set(np.eye(6))


tSpan = np.array([0, 5])

g0 = theGroup()
initFlag = 2
if(initFlag == 0):
    gd = theGroup(x=np.array([1, 1, 1]).reshape((3,1)), R=np.eye(3))
elif(initFlag == 1):
    e1 = np.array([2, 5, -6]).reshape((3,1))
    e1 = e1/np.linalg.norm(e1)
    e2 = np.array([0, 6, 5]).reshape((3,1))
    e2 = e2/np.linalg.norm(e2)
    e3 = np.cross(np.squeeze(e1), np.squeeze(e2))
    gd = theGroup(x=np.zeros((3,1)), R=np.hstack((e1, e2, e3[:, np.newaxis])))
elif(initFlag == 2):
    e1 = np.array([2, 5, -6]).reshape((3,1))
    e1 = e1/np.linalg.norm(e1)
    e2 = np.array([0, 6, 5]).reshape((3,1))
    e2 = e2/np.linalg.norm(e2)
    e3 = np.cross(np.squeeze(e1), np.squeeze(e2))
    gd = theGroup(x=2*np.ones((3,1)), R=np.hstack((e1, e2, e3[:, np.newaxis])))
print(gd)

theSim = simController(theSolver, theController.regulator(gd)[0])
theSim.initialize(tSpan, g0)
sol = theSim.simulate()

print(sol.x[0])
print(sol.x[-1])

plt.figure()
ax = plt.axes(projection='3d')
for i in range(len(sol.t)):
    sol.x[i].plot(ax=ax, scale=0.2)
#SE3().plot(ax=ax, scale=0.2)
ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
plt.show()