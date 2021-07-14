import numpy as np
import scipy.linalg
from controller.base import base
from simController import simController
from structures import structure
from enum import Enum
from simController import simController
from matplotlib import pyplot as plt

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
                    #self.control.trackerNew(self.path)
                    #self.pState = self.TimePointState.TRACK

                #input()
                self.control.refresh(self,t)
                if (t >= self.tNext):
                    self.pState = self.TimePointState.GENERATE
                # Always provide control.
                A = self.control.compute(t, x)
                return A
            else:
                return self.control.compute()
        self.desTraj = desTraj
        self.pState = self.TimePointState.GENERATE

        self.compute = pathTrackerTP


    def nextLeg(self, tc):
        #print(tc)
        #input("hello")
        tTerm = tc + self.specs.Th
        tNext = tc + self.specs.Td
        if (tTerm > self.desTraj.tspan[-1]):
            tTerm  = self.desTraj.tspan[-1]

        if (tNext > self.desTraj.tspan[-1]):
            tNext = self.desTraj.tspan[-1]

        self.tNext = tNext

        #print(tNext)
        #input()
        #! Desired, current and terminal states
        xDesCurr = self.specs.vec2state(self.desTraj.x(tc))
        xDesTerm = self.specs.vec2state(self.desTraj.x(tTerm))

        self.path.generate(tc, xDesCurr, tTerm, xDesTerm)
        myt = 0
        for i in range(1,100):
            myt += .01
            rc = self.path.x(tc+ myt)
            plt.figure(1)
            plt.scatter(rc[0,0],rc[1,0])

    def updatepState(self,input):
        self.pState = input

    @staticmethod
    def simBuilder(ceom, cfS):
        def initialize(istate, desTraj):
            cfS.controller.tracker(desTraj)
            cfS.controller.control.simrefresh(desTraj)

            theSim = simController(solver, cfS.controller)
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim

        def reconfigure(theSim, istate, desTraj):
            cfS.controller.tracker(desTraj)
            cfS.controller.control.simrefresh(desTraj)
            theSim.controller = cfS.controller
            #input()
            theSim.reset()
            theSim.initializeByStruct(desTraj.tspan, istate)
            return theSim

        solver = cfS.odeMethod(ceom, cfS.dt)

        simInit = structure()
        simInit.firstBuild = initialize
        simInit.reconfig = reconfigure


        return simInit
