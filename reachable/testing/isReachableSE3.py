import numpy as np
from reachable.SE3.fsDiffDriveFO import DiffDriveFO
from structures import structure
import matplotlib.pyplot as plt
import pdb

specs = structure()
specs.dt   = 7
specs.Gmax = 4*32.1740
specs.angType = 'GForce'
specs.vBounds = [250, 1100]

rs = DiffDriveFO(specs)

rs.setLimitsFromAcceleration(600, np.array([-30, 15]).reshape((2,1)))
rs.compute()

dg = structure()
dg.r     = 0.2
dg.theta = 0.05
theBdy = rs.getBoundary(dg)
pMin = np.amin(theBdy,axis=1)
pMax = np.amax(theBdy,axis=1)

ng = 10
[xx,yy, zz] = np.meshgrid(np.linspace(pMin[0],pMax[0],ng), \
    np.linspace(pMin[1],pMax[1],ng), \
    np.linspace(pMin[1],pMax[1],ng))

tPts = np.vstack((xx.T.ravel(), yy.T.ravel(), zz.T.ravel()))
isReach = np.squeeze(rs.isReachablePoint(tPts))

'''
ax.scatter(tPts[0, isReach==True],tPts[1, isReach==True], tPts[2, isReach==True],color='g', marker='.')
ax.scatter(tPts[0, isReach==False],tPts[1, isReach==False], tPts[2, isReach==False],color='r', marker='.')
plt.show()
'''

rs.plotBoundary(dg)