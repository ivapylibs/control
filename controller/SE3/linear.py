from structures import structure
from simController import simController
import controller.linear
import trajectory.Path
import Curves.CurveBase
import numpy as np
from numIntegrator.group.simFirstOrder import SimFirstOrder

import pdb

class linear(controller.linear.linear):
    def __init__(self, xeq=0, ueq=0, K=0) -> None:
        super().__init__(xeq, ueq, K)

        self.ref2group = lambda x: x # Identity function unless overridden
        self.state2group = lambda x: x
        self.err2vec = None
    
    def stabilizer(self):
        gDes = self.ref2group(self.xeq)
        def stabLinControl(t=None, x=None):
            if(x is not None):
                gCur = self.state2group(x)
                gCInv = gCur.inv()
                gErr = gCInv * gDes
                if(self.err2vec is None):
                    vErr = gErr.log()
                else:
                    vErr = self.err2vec(gErr)

                u = self.ueq + np.matmul(self.K, vErr)
            else:
                u = self.ueq
            rc = self.xeq
            return u, rc
        self.compute = stabLinControl
        return stabLinControl

    def regulator(self, xDes):
        gDes = self.ref2group(xDes)
        def stabLinControl(t=None, x=None):
            if(x is not None):
                gCur = self.state2group(x)
                gCInv = gCur.inv()
                gErr = gCInv * gDes
                if(self.err2vec is None):
                    vErr = gErr.log()
                else:
                    vErr = self.err2vec(gErr)

                u = self.ueq + np.matmul(self.K, vErr)
            else:
                u = self.ueq
            rc = xDes
            return u, rc
        linLaw = stabLinControl
        self.compute = stabLinControl
        return linLaw, xDes

    def tracker(self, desTraj):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                rc = desTraj.x(t)
                gDes = self.ref2group(rc)
                gCur = self.ref2group(x)

                gCInv = gCur.inv()
                gErr = gCInv*gDes

                if(self.err2vec is None):
                    vErr = gErr.log()
                else:
                    vErr = self.err2vec(gErr)

                u = self.ueq + np.matmul(self.K, vErr)
            else:
                u = np.zeros((np.shape(self.K)[0], 1))
                rc = np.zeros((np.shape(self.K)[1], 1))
            return u, rc

        if(isinstance(desTraj, Curves.CurveBase) or isinstance(desTraj, trajectory.Path)):
            self.compute = trackLinControl
            return trackLinControl
    
    @staticmethod
    def systemDynamics(B=None):
        def twistCEOM(t, g, u):
            return u
        def twistCEOMwB(t, g, u):
            return np.matmul(B, u)
        if(B is None):
            return twistCEOM
        else:
            return twistCEOMwB
    
    def simBuilder(theGroup, ceom, cfS):
        #------------------------- theInitializer ------------------------
        #!
        #!
        def theInitializer(istate, desTraj):
            #! Controller needs building.
            cfS.controller.tracker(desTraj)

            #! simController configuration should be instantiated.
            theSim = simController(solver, cfS.controller)
            theSim.initializeByStruct(desTraj.tspan, istate)

        #-------------------------- reconfigure --------------------------
        #!
        #!
        def reconfigure(theSim, istate, desTraj):
            #! The controller needs reconfiguration.
            cfS.controller.tracker(desTraj)
            theSim.setController( cfS.controller )

            #! The simulated system needs to be reset and given new initial state.
            theSim.reset()
            theSim.initializeByStruct(desTraj.tspan, istate)
        #!--[1] Base equations.
        if ('simOpts' in cfS and cfS.simOpts is not None):
            solver = SimFirstOrder( theGroup, ceom, cfS.dt, \
                                                                cfS.simOpts)
        else:
            solver = SimFirstOrder( theGroup, ceom, cfS.dt)
        # @todo   What about optional args.

        #!--[4] Provide access to the function handles.
        simInit = structure()
        simInit.firstBuild = theInitializer
        simInit.reconfig   = reconfigure
        return simInit
