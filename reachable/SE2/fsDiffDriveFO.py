import numpy as np
from numpy.core.numeric import isscalar
from reachable.SE2.fromSpecFO import FromSpecFO
from scipy.spatial.distance import pdist
from scipy.interpolate import RegularGridInterpolator
from matplotlib import pyplot as plt
import pdb

class DiffDriveFO(FromSpecFO):
    def __init__(self, specs):
        super().__init__(specs)

    def setLimitsFromAcceleration(self, v0, aLims):
        if(np.isscalar(self.specs.dt)):
            self.specs.vLim = np.max(v0 + 0.5*aLims*self.specs.dt, 0)
        else:
            self.specs.vLim = np.max(v0 + 0.5*aLims*self.specs.dt[1], 0)

        if('vBounds' in self.specs):
            self.specs.vLim[0] = np.max(self.specs.vLim[0], self.specs.vBounds[0])
            self.specs.vLim[1] = np.min(self.specs.vLim[1], self.specs.vBounds[1])
    
    def compute(self):
        if(np.isscalar(self.specs.dt)):
            if(self.specs.angType == 'linear'):
                theta = self.specs.wLim * self.specs.dt/2
                (rad, thetalim)  = self.limRadius(self.specs.vLim,self.specs.wLim)
                self.rad2ang = RegularGridInterpolator(points = (rad,), values=theta, bounds_error=False, fill_value=None)

            elif(self.specs.angType == 'interp'):
                wGrid = np.linspace(self.specs.wLim[0],self.specs.wLim[1],30)
                vGrid = np.linspace(self.specs.vLim[0],self.specs.vLim[1],30)
                (rv, av) = self.limRadius(vGrid, wGrid)
                self.rad2ang = RegularGridInterpolator(points=(rv,), values=av, bounds_error=False, fill_value=None)

            elif(self.specs.angType == 'GForce'):
                vGrid = np.linspace(self.specs.vLim[0],self.specs.vLim[1],30)
                wGrid = self.specs.Gmax/vGrid
                (rv, av) = self.limRadius(vGrid, wGrid)
                self.rad2ang = RegularGridInterpolator(points=(rv,), values=av, bounds_error=False, fill_value=None)
  
                self.specs.wLim = self.specs.Gmax/self.specs.vLim
        
        else:
            raise NotImplementedError
    
    def limRadius(self, v, w, dt=None):
        if(dt == None):
            if(np.isscalar(self.specs.dt)):
                dt = self.specs.dt
            else:
                dt = self.specs.dt[0]
        rNom = v*dt
        theta = w*dt
        saInd = abs(theta) < self.aTol
        rLim = np.zeros((len(saInd),))
        if(any(saInd)):
            rLim[saInd]  =  rNom * np.sqrt(1 - (theta(saInd)**4)/24)
            rLim[~saInd] = (rNom/(theta(~saInd))) * np.sqrt(2 - 2*np.cos(theta(~saInd)))
        else:
            rLim = (rNom/theta) *np.sqrt(2 - 2*np.cos(theta))
        theta = theta/2
        return (rLim, theta)

    def diameter(self, dg):
        bts = self.getBoundary(dg)
        dvals = pdist(bts)
        return np.max(dvals)
    
    def isReachablePoint(self, p):
        r = np.linalg.norm(p,2,1)
        a = np.arctan2(p[:,1], p[:,0])
        canReach = np.zeros((np.shape(p)[0],), dtype=bool)

        t1 = self.testThetaLimits(a)
        idx1 = np.where(t1 == True)[0]

        if(len(idx1) > 0):
            apos = abs(a[idx1])
            rMin = self.inRadius(apos)
            rMax = self.outRadius(apos)

            t2 = (r[idx1] >= rMin) & (r[idx1] <= rMax)
            idx2 = idx1[np.where(t2 == True)[0]]
            if(len(idx2) > 0):
                aLim = self.angLimit(r[idx2])
                #pdb.set_trace()
                canReach[idx2] = apos[idx2] <= aLim
        print(canReach)
        return canReach

    def inRadius(self, a):
        if(np.isscalar(self.specs.dt)):
            dt = self.specs.dt

            rNom = self.specs.vLim[0] * dt
            rMin = np.zeros((len(a),))

            asInd = abs(a) < self.aTol
            if(any(asInd)):
                rMin[ asInd] = rNom * np.sqrt(1 - (a[asInd]**4)/3)
                rMin[~asInd] = (rNom/(2*a[~asInd])) * np.sqrt(2 - 2*np.cos(2*a[~asInd]))
            else:
                rMin = (rNom/(2*a)) * np.sqrt(2 - 2*np.cos(2*a))
            return rMin
        else:
            return  self.rmin.f(a)

    def outRadius(self, a):
        if(np.isscalar(self.specs.dt)):
            dt = self.specs.dt

            rNom = self.specs.vLim[1] * dt
            rMax = np.zeros((len(a),))

            asInd = abs(a) < self.aTol
            if(any(asInd)):
                rMax[ asInd] = rNom * np.sqrt(1 - (a[asInd]**4)/3)
                rMax[~asInd] = (rNom/(2*a[~asInd])) * np.sqrt(2 - 2*np.cos(2*a[~asInd]))
            else:
                rMax = (rNom/(2*a)) * np.sqrt(2 - 2*np.cos(2*a))
            return rMax
        else:
            return  self.rmax.f(a)
    
    def angLimit(self, r):
        return self.rad2ang(r)

    def getBoundary(self, dg):
        if (np.isscalar(self.specs.dt)):
            thetaMax = self.specs.wLim * self.specs.dt/2

            #== Inner arc.
            thetaGrd = np.linspace(0, thetaMax[0], round(thetaMax[0]/dg.theta))
            inRad  = self.inRadius(thetaGrd)
        else:
            thetaMax = self.specs.wLim * self.specs.dt[1]/2

            thetaGrd = self.rmin.a
            inRad = self.rmin.r

        pIn  = inRad * np.vstack((np.cos(thetaGrd), np.sin(thetaGrd)))
        
        #== Outer Arc
        thetaGrd = np.linspace(0, thetaMax[1], round(thetaMax[1]/dg.theta))
        outRad = self.outRadius(thetaGrd)
        pOut  = outRad * np.vstack((np.cos(thetaGrd), np.sin(thetaGrd)))

        #== Edges
        r1edge = inRad[-1]
        r2edge = outRad[-1]

        radGrd    = np.linspace(r1edge, r2edge, round((r2edge-r1edge)/dg.r))
        thetaEdge = self.angLimit(radGrd)

        eLeft  = radGrd * np.vstack((np.cos(thetaEdge), np.sin(thetaEdge)))

        bdPts = np.hstack((pIn, eLeft, np.fliplr(pOut)))
        bdPtsFlip = np.fliplr(np.matmul(np.diag([1,-1]),bdPts))
        bdPts = np.hstack((bdPts, bdPtsFlip))
        return bdPts

    def plotBoundary(self, dg):
        boundaryPts = self.getBoundary(dg)
        plt.plot(boundaryPts[0,:], boundaryPts[1,:], 'k-')

    def testThetaLimits(self, a):
        if (np.isscalar(self.specs.dt)):
            dt = self.specs.dt
        else:
            dt = self.specs.dt[1]
        thetaMax = max(self.specs.wLim) * dt
        return abs(a) <= thetaMax

  


    def isReachable(self, x):
        return False