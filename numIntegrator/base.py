from enum import Enum
import numpy as np
from abc import ABC, abstractmethod
import pdb

class base(ABC):
    def __init__(self, theDiffEq, dt, opts=None):
        self.diffeq = theDiffEq
        self.dt = dt

        self.NOTREADY    = 0
        self.INITIALIZED = 1
        self.INTEGRATING = 2
        self.LASTSTEP    = 3
        self.DONE        = 4

        self.state = self.NOTREADY

    
    def setDynamics(this, theFunc):
        this.diffeq.setDynamics(theFunc)

    def initialize(self, tinfo, x0):
        self.extra = False
        self.x0 = x0
        self.t0 = tinfo
        self.state = self.INITIALIZED
        if (not np.isscalar(tinfo)):
            #tInt = [tinfo(1):this.dt:tinfo(2)]
            tInt = np.arange(tinfo[0], tinfo[1]+self.dt, self.dt)
            if (tInt[-1] != tinfo[1]):
                self.extra = True
                tInt = np.hstack((tInt, tinfo[1]))

            self.t = tInt
        else:
            self.t = tinfo[0]

        self.xc = x0
        self.tc = tinfo[0]

        self.state = self.INITIALIZED

    def reset(self):
        if(self.state == self.INITIALIZED or self.state == self.NOTREADY):
            return
        else:
            self.initialize(self.t0, self.xc)

    #@abstractmethod
    def preStep(self):
        return

    #@abstractmethod
    def runStep(self):
        return

    #@abstractmethod
    def advance(self):
        return

    #============================= integrate =============================
    #
    # @brief  Run the numerical integration scheme.
    #
    # This version is empty/does nothing, and should be overloaded.
    #
    #
    # @param[in]  tinfo   The time info: start point (and end point optional).
    # @param[in]  x0      The initial state.
    #
    # @param[out] tv      The time value(s).
    # @param[out] xv      The state value(s).
    #
    def integrate(self):
        return