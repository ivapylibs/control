import numpy as np
from controller.base import base
from simController import simController
from structures import structure
from enum import Enum
from matplotlib import pyplot as plt

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
        self.tNext = specs.Td

        self.uc = self.control.compute
        self.compute = self.control.compute
        self.myt = 0;

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
                self.myt += .01
                #print(self.tState)
                if self.tState == self.fixedHorizonsState.SYNTHESIZE:
                    self.myt = 0;
                    self.nextHorizon(t,x)
                    #self.control.tracker(self.synTraj)
                    self.control.trackerPath(self.synTraj)
                    self.tState = self.fixedHorizonsState.TRACK


                myarr = self.control.compute(self.myt,x)
                self.uc = myarr[1]
                if self.myt>=(self.tNext-self.specs.Ts):
                    self.tState = self.fixedHorizonsState.SYNTHESIZE
            else:
                myarr = self.control.compute()
            return myarr
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
        myarr = np.arange(tc,tTerm,.01).tolist()
        finalx = []
        finaly = []
        for i in range(len(myarr)):
            finalx.append(pathSeg.x(myarr[i])[0])
            finaly.append(pathSeg.x(myarr[i])[1])

        #print(finalx)
        #plt.plot(finalx[:],finaly[:], 'b')
        istate = structure()
        istate = self.specs.state2state(xc)
        #print(xc)
        #print(istate)

        self.synTraj = self.synthesizer.followPath(istate,pathSeg.x)
        #print(istate)
        plt.plot(self.synTraj[:,0],self.synTraj[:,1],'b')

    @staticmethod
    def simBuilder(ceom, cfS):
        def initialize(istate, desTraj):
            cfS.controller.tracker(desTraj)

            theSim = simController(solver, cfS.controller.compute)
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
