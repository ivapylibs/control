import numpy as np

from reachable.SE2.fsDiffDriveFO import DiffDriveFO
from structures import structure
from matplotlib import pyplot as plt
import pdb

#==[1] Reachable set instance
specs = structure()
specs.dt   = 3
specs.vLim = np.array([5, 7])
specs.wLim = np.array([np.pi/5 , np.pi/10])
specs.Gmax = 3.5
specs.angType = 'GForce'


rs = DiffDriveFO(specs)

rs.compute()

dg = structure
dg.r     = 0.2
dg.theta = 0.05
rs.plotBoundary(dg)

plt.plot(0,0,'r+')
plt.axis('equal')
theBdy = rs.getBoundary(dg)
pMin = np.amin(theBdy,axis=1)
pMax = np.amax(theBdy,axis=1)
ng = 40
[xx,yy] = np.meshgrid(np.linspace(pMin[0],pMax[0],ng), np.linspace(pMin[1],pMax[1],ng))

tPts = np.transpose(np.vstack((xx.T.ravel(), yy.T.ravel())))
isReach = np.squeeze(rs.isReachablePoint(tPts))

plt.plot(tPts[np.where(isReach==True), 0], tPts[np.where(isReach==True), 1],'g.')
plt.plot(tPts[np.where(isReach==False), 0], tPts[np.where(isReach==False), 1],'r.')

plt.show()
