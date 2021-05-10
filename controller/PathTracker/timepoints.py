import numpy as np
import scipy.linalg
from controller.base import base
from simController import simController
from structures import structure
from enum import Enum
from simController import simController

import pdb

class timepoints(base):
    class TimePointState(Enum):
        EMPTY    = 0
        GENERATE = 1
        TRACK    = 2

    def __init__(self, pathGenerator, controlLaw, specs):
        super().__init__()
        self.path = pathGenerator
        self.control = controlLaw
        self.specs = specs

    def tracker(self, desTraj):
        def pathTrackerTP(t=None, x=None):
            if(t is not None and x is not None):
                if (self.pState == self.TimePointState.GENERATE):
                    self.nextLeg(t)
                    self.control.trackerNew(self.path)
                    self.pState = self.TimePointState.TRACK

                if (t >= self.tNext):
                    self.pState = self.TimePointState.GENERATE
                # Always provide control.
                return self.control.compute(t, x)
            else:
                return self.control.compute()
        self.desTraj = desTraj
        self.pState = self.TimePointState.GENERATE

        self.compute = pathTrackerTP

    
    def nextLeg(self, tc):
        tTerm = tc + self.specs.Th
        tNext = tc + self.specs.Td
        if (tTerm > self.desTraj.tspan[-1]):
            tTerm  = self.desTraj.tspan[-1]

        if (tNext > self.desTraj.tspan[-1]):
            tNext = self.desTraj.tspan[-1]
        
        self.tNext = tNext

        #! Desired, current and terminal states
        xDesCurr = self.specs.vec2state(self.desTraj.x(tc))
        xDesTerm = self.specs.vec2state(self.desTraj.x(tTerm))

        #pdb.set_trace() 
        self.path.generate(tc, xDesCurr, tTerm, xDesTerm)

    @staticmethod
    def simBuilder(ceom, cfS):
        def initialize(istate, desTraj):
            cfS.controller.tracker(desTraj)

            theSim = simController(solver, cfS.controller)
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim
        
        def reconfigure(theSim, istate, desTraj):
            cfS.controller.tracker(desTraj)
            theSim.controller = cfS.controller

            theSim.reset()
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim

        solver = cfS.odeMethod(ceom, cfS.dt)

        simInit = structure()
        simInit.firstBuild = initialize
        simInit.reconfig = reconfigure

        
        return simInit
