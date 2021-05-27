from numpy.core.function_base import linspace
from structures import structure
import numpy as np
from numpy.core.fromnumeric import reshape
import reachable.SE2.fsDiffDriveFO
import Lie.SE3
import matplotlib.pyplot as plt
import pdb


class DiffDriveFO(reachable.SE2.fsDiffDriveFO.DiffDriveFO):
    def __init__(self, specs):
        super().__init__(specs)

    def isReachablePoint(self, p):
        n = np.vstack((0, p[1:3, 0, np.newaxis]))

        if(np.linalg.norm(n) == 0):
            n = np.array([[0], [1], [0]])
        else:
            n = n/np.linalg.norm(n)

        u = np.array([[1], [0], [0]])
        Rx = Lie.SE3.RotX(np.pi/2)
        Rx[0,0] = 0
        R = np.hstack((u, n, np.matmul(Rx, n))).T

        R = R[0:2, :]
        z = np.matmul(R, p)
        return super().isReachablePoint(z)

    def getHalfBoundaries(self,dg):
        if (np.isscalar(self.specs.dt)):
            thetaMax = self.specs.wLim * self.specs.dt/2

            #== Inner arc.
            thetaGrd = np.linspace(0, thetaMax[0], round(thetaMax[0]/dg.theta))
            inRad  = self.inRadius(thetaGrd)
            pIn  = inRad * np.vstack((np.cos(thetaGrd) , np.sin(thetaGrd)))
        else:
            thetaMax = self.specs.wLim * self.specs.dt[1]/2

            thetaGrd = self.rmin.a
            inRad = self.rmin.r

            pIn  = inRad * np.vstack((np.cos(thetaGrd) , np.sin(thetaGrd)))

        thetaGrd = np.linspace(0, thetaMax[1], round(thetaMax[1]/dg.theta))
        outRad = self.outRadius(thetaGrd)
        pOut = outRad * np.vstack((np.cos(thetaGrd) , np.sin(thetaGrd)))
  
        #== Edges
        r1edge = inRad[-1]
        r2edge = outRad[-1]
  
        radGrd    = np.linspace(r1edge, r2edge, round((r2edge-r1edge)/dg.r))
        thetaEdge = self.angLimit(radGrd)
  
        eLeft  = radGrd * np.vstack((np.cos(thetaEdge) , np.sin(thetaEdge)))
  
        #== Combine in a structure
        
        halfBounds = structure()
        halfBounds.outer = pOut
        halfBounds.inner = pIn
        halfBounds.side  = eLeft
        return halfBounds

    def plotBoundary(self, dg):
        hbLines = self.getHalfBoundaries(dg)

        c1 = structure(X=None, Y=None, Z=None)
        (c1.X, c1.Y, c1.Z) = DiffDriveFO.cylinder(hbLines.outer[1,:])
        c1.Z = np.tile(hbLines.outer[0,:, np.newaxis], (1,np.shape(c1.X)[1]))

        c2 = structure(X=None, Y=None, Z=None)
        (c2.X, c2.Y, c2.Z) = DiffDriveFO.cylinder(hbLines.inner[1,:])
        c2.Z = np.tile(hbLines.inner[0,:, np.newaxis], (1,np.shape(c2.X)[1]))

        c3 = structure(X=None, Y=None, Z=None)
        (c3.X, c3.Y, c3.Z) = DiffDriveFO.cylinder(hbLines.side[1,:])
        c3.Z = np.tile(hbLines.side[0,:, np.newaxis], (1,np.shape(c3.X)[1]))

        c = structure()
        c.X = np.vstack((c1.Z , c3.Z , np.flipud(c2.Z)))
        c.Y = np.vstack((c1.Y , c3.Y , np.flipud(c2.Y)))
        c.Z = np.vstack((c1.X , c3.X , np.flipud(c2.X)))

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot_surface(c1.X, c1.Y, c1.Z)
        ax.plot_surface(c2.X, c2.Y, c2.Z)

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot_surface(c.X, c.Y, c.Z, alpha=0.05)

        plt.show()




    # Implements the matlab cylinder() method
    @staticmethod
    def cylinder(r, n=20):
        if(np.isscalar(r)):
            length = 2
            r = np.tile(r, (2,1))
        else:
            length = len(r)
        z = np.linspace(0, 1, length).reshape((length, 1))
        z = np.tile(z, (1,n))
        theta = np.linspace(0, 2*np.pi, n)
        x = np.tile(np.cos(theta), (length,1))
        x = (x.T * r).T
        y = np.tile(np.sin(theta), (length,1))
        y = (y.T * r).T
        return (x, y, z)