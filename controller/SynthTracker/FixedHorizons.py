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
        self.desTraj = None;
        self.counter = 0;

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
            #print(t)
            #input('T in pathTrackerTP')
            myarr = [0,0]
            if t is not None and x is not None:
                #print(self.tState)
                if(self.myt  >= self.tNext):
                    self.updateTstate(self.fixedHorizonsState.SYNTHESIZE)
                if self.tState == self.fixedHorizonsState.SYNTHESIZE:
                    self.myt = 0;
                    self.nextHorizon(t,x)
                    #self.control.tracker(self.synTraj)
                    self.control.trackerPath(self.synTraj)
                    self.tState = self.fixedHorizonsState.TRACK


                myarr = self.control.compute(self.myt,x)
                self.uc = myarr[1]
                #print(self.myt)
                #print(self.tNext)
                #input()

                self.myt += self.specs.Ts
            else:
                myarr = self.control.compute()
            return myarr

        self.desTraj = desTraj
        self.tState = self.fixedHorizonsState.SYNTHESIZE
        self.compute = pathTrackerTP

    def nextHorizon(self,tc,xc):
        #print(tc)
        #input('printing tc in nextHorizon')
        tTerm = tc + self.specs.Th
        #print(self.specs.Th)
        tNext = tc + self.specs.Td
        #print(self.specs.Td)
        #print(self.desTraj.tspan)
        #nput()

        if tTerm > self.desTraj.tspan[-1]:
            tTerm = self.desTraj.tspan[-1]
        if tNext > self.desTraj.tspan[-1]:
            tNext = self.desTraj.tspan[-1]
        #print([tc,tTerm])

        #input('self desTraj tspan in next horion')
        #print(tTerm)
        #print(tc)
        #input()
        pathSeg = self.desTraj.segment([tc,tTerm])
        #print([tc,tTerm])
        #input()
        myarr = np.arange(tc,tTerm,self.specs.Ts).tolist()
        myt = 0 + tc


        #print(finalx)
        #plt.plot(finalx[:],finaly[:], 'b')
        istate = structure()
        istate = self.specs.state2state(xc)
        #print(xc)
        #print(istate)
        #input()
        plt.scatter(istate[0,0],istate[1,0])

        self.synTraj = self.synthesizer.followPath(istate,pathSeg,tc)
        #print(pathSeg.tspan())
        #input()
        #print(istate)
        #input()
        #myarry = np.concatenate((self.synTraj['_x'],self.synTraj['_u']),axis=1)
        #plt.plot(myarry[:,0],myarry[:,1])

    def updateDesTraj(self, path):
        #print(path.tspan)
        #input('update destraj')
        self.desTraj = path
    def updateTstate(self,value):
        self.tState = value

    def refresh(self,prev,t):
        if (prev.pState == prev.TimePointState.GENERATE):
            #self.control.trackerPath(prev.path)
            self.updateTstate(self.fixedHorizonsState.SYNTHESIZE)
            prev.updatepState(prev.TimePointState.TRACK)
            self.updateDesTraj(prev.path)
        if (t>self.tNext):
            self.updateTstate(self.fixedHorizonsState.SYNTHESIZE)

    def simrefresh(self,desTraj):
        #print(desTraj)
        #input('simRefresh')
        self.tracker(desTraj.x)
        #print(self.counter)
        #input()
            #input('Refresh finding out if plotting works')
        #self.updateDesTraj(desTraj)

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
