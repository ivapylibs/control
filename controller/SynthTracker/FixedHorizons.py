import numpy as np
from controller.base import base
from simController import simController
from structures import structure
from enum import Enum

import pdb

class fixedHorizons(base):
    class fixedHorizonsState(Enum):
        EMPTY = 0
        SYNTHESIZE = 1
        TRACK = 2

    def __init__(self, tSynth, uLaw, specs):
        self.synthesizer = tSynth
        self.control = uLaw
        self.specs = specs
        self.tState = None

        this.uc = this.control.compute
        this.compute = this.control.compute

        def vec2state(x):
            return x
        def state2state(x):
            return x

        self.specs.vec2state = vec2state
        self.specs.state2state = state2state

        if specs.Td > specs.Th:
            specs.Td = specs.Th

    def setSynthesizer(self, tSynth):
        self.synthesizer = tSynth

    def tracker(self, desTraj):
        def pathTrackerTP(t=None,x=None):
            myarr = [0,0]
            if t is not None and x is not None:
                if self.tState == self.fixedHorizonsState.SYNTHESIZE:
                    self.nextHorizon(t,x)
                    self.control.trackerNew(self.synTraj)
                    self.tState = self.fixedHorizonsState.TRACK

                myarr = self.control.compute(t,x)
                self.uc = u
                if t>=self.tNext:
                    this.tState = this.fixedHorizonsState.SYNTHESIZE
            else:
                myarr = this.control.compute()
        self.desTraj = desTraj
        self.tState = self.fixedHorizonsState.SYNTHESIZE
        self.compute = pathTrackerTP

    def nextHorizon(self,tc,xc):
        tTerm = tc + self.specs.Th
        tNext = tc + self.specs.Td
        if tTerm > self.desTraj.tspan[-1]:
            tTerm = self.desTraj.tspan[-1]
        if tNext > self.desTraj.tspan[-1]:
            tNext = self.desTraj.tspan[-1]
        pathSeg = self.desTraj.segment([tc,tTerm])
        istate.x = self.specs.state2state(xc)
        istate.u = self.uc
        this.synTraj = this.synthesizer.followPath(istate,pathSeg)

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
