import trajectory
import numpy as np
import controller.linear
from Curves import CurveBase
from Lie import SE2
from structures import structure
from simController import simController

import pdb

class linear(controller.linear.linear):
    def __init__(self, xeq=0, ueq=0, K=0):
        super().__init__(xeq, ueq, K)
        self.inBodyFrame = False

    def setBodyFrame(self, isBodyFrame):
        self.inBodyFrame = isBodyFrame

    def setByCARE(self, A, B, Q):
        (K, P, Leig)= super().setByCARE(A, B, Q)
        self.K = -K
        return (K, P, Leig)

    def set(self, K):
        self.K = K
        return K

    def setByCareFromStruct(self, cfs):
        (K, P, Leig) = self.setByCARE(A=cfs.A, B=cfs.B, Q=cfs.Q)
        return (K, P, Leig)

    def stabilizer(self):
        def stabLinControl(t=None, x=None):
            if x is None:
                u = self.ueq + np.zeros((self.K.shape[0],1))
            else:
                u = self.ueq + np.matmul(self.K, (x-self.xeq))

            rc = self.xeq
            return (u, rc)

        self.control = stabLinControl
        return stabLinControl

    def noControl(self):
        def doNothing(t=None, x=None):
            return (self.ueq, self.xeq)
        self.compute = doNothing
        return doNothing

    def rectan(r, phi):
        return r*(np.cos(phi) + np.sin(phi) * 1j)

    def tracker(self, xdes, uFF=None, stateDep=False):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                rc = xdes(t)
                gDes = SE2(x=rc[0:2], R = SE2.rotationMatrix(rc[2]))
                vDes = rc[3:6]

                gCur = SE2(x=x[0:2], R=SE2.rotationMatrix(x[2]))
                vCur = x[3:6]

                gCInv = gCur.inv()
                gErr = gCInv*gDes

                aErr = gErr.getAngle()
                pErr = gErr.getTranslation()

                if(self.inBodyFrame):
                    xErr = np.vstack((pErr, aErr, vDes - gCInv * vCur))
                else:
                    xErr = np.vstack((pErr, aErr, gCInv*(vDes - vCur)))
                u = self.ueq + np.matmul(self.K, xErr)
                #pdb.set_trace()

            else:
                u = np.zeros((np.shape(self.K)[0], 1))
                rc = np.zeros((np.shape(self.K)[1], 1))
            return u, rc

        self.compute = trackLinControl
        return trackLinControl

    def trackerPath(self, xdes, uFF=None, stateDep=False):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                myt = int(t/.01)
                #print(myt)
                rc = xdes[myt]
                #print(rc)
                #input()
                #print(np.shape(rc))
                rc = np.array(rc)[np.newaxis]
                rc = rc.T
                #print(rc)
                #input("enter")
                gDes = SE2(x=rc[0:2], R = SE2.rotationMatrix(rc[2]))
                temp = np.array([rc[3],0])[np.newaxis]
                temp = temp.T

                vDes = np.concatenate((temp,[rc[4]]))
                #print(gDes)
                #print(vDes)
                #input('enter')

                gCur = SE2(x=x[0:2], R=SE2.rotationMatrix(x[2]))
                vCur = x[3:6]
                #print(self.xeq)
                #print(vCur)
                #input("enter")
                gCInv = gCur.inv()
                gErr = gCInv*gDes

                aErr = gErr.getAngle()
                pErr = gErr.getTranslation()

                if(self.inBodyFrame):
                    xErr = np.vstack((pErr, aErr))
                else:
                    xErr = np.vstack((pErr, aErr))
                u = self.ueq + np.matmul(self.K, xErr) + rc[2:4]
                #pdb.set_trace()

            else:
                u = np.zeros((np.shape(self.K)[0], 1))
                rc = np.zeros((np.shape(self.K)[1], 1))
            return u, rc
        self.compute = trackLinControl
        return trackLinControl

    def trackerNew(self, desTraj):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                rc = desTraj.x(t)
                gDes = SE2(x=rc[0:2], R = SE2.rotationMatrix(rc[2]))
                vDes = rc[3:6]

                gCur = SE2(x=x[0:2], R=SE2.rotationMatrix(x[2]))
                vCur = x[3:6]

                gCInv = gCur.inv()
                gErr = gCInv*gDes

                aErr = gErr.getAngle()
                pErr = gErr.getTranslation()

                if(self.inBodyFrame):
                    xErr = np.vstack((pErr, aErr, vDes - gCInv * vCur))
                else:
                    xErr = np.vstack((pErr, aErr, gCInv*(vDes - vCur)))
                u = self.ueq + np.matmul(self.K, xErr)
                #pdb.set_trace()

            else:
                u = np.zeros((np.shape(self.K)[0], 1))
                rc = np.zeros((np.shape(self.K)[1], 1))
            return u, rc

        if(isinstance(desTraj, CurveBase) or isinstance(desTraj, trajectory.Path)):
            self.compute = trackLinControl
            return trackLinControl

    @staticmethod
    def simBuilder(ceom, cfS):
        def initialize(istate, desTraj):
            cfS.controller.trackerNew(desTraj)
            theSim = simController(solver, cfS.controller)
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim

        def reconfigure(theSim, istate, desTraj):
            cfS.controller.trackerNew(desTraj)
            theSim.controller = cfS.controller

            theSim.reset()
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim

        solver = cfS.odeMethod(ceom, cfS.dt)

        simInit = structure()
        simInit.firstBuild = initialize
        simInit.reconfig = reconfigure

        return simInit
