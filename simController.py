# !/usr/bin/env python

import numpy as np
from controller.base import base
from structures import structure

class simController(object):

    def __init__(self, theSolver, theLaw):
        self.solver = theSolver
        self.cLaw = theLaw

    def setController(self, theLaw):
        self.cLaw = theLaw



    def initialize(self, tspan, x0):
        self.solver.initialize(tspan, x0)

        if (isinstance(self.cLaw, base)):
            (u0, rc) = self.cLaw.compute()
        else:
            (u0, rc) = self.cLaw()
        print(u0)
        self.u = np.zeros((u0.shape[0], self.solver.t.size))

    def initializeByStruct(self, tspan, istate):
        self.initialize(tspan=tspan, x0=istate.x)

    def reset(self):
        self.solver.reset()

        if isinstance(self.cLaw, base):
            u0 = self.cLaw.compute()
        else:
            u0 = self.cLaw()

        self.u = np.zeros_like(self.u)

    def preStep(self):
        pass

    def postStep(self):
        pass

    def advance(self, varargin):
        self.preStep()

        if isinstance(self.cLaw, base):
            (self.uc, rc) = self.cLaw.compute(self.solver.tc, self.solver.xc)
        else:
            (self.uc, rc) = self.cLaw(self.solver.tc, self.solver.xc)
        self.u[:,self.solver.ci:self.solver.ci+1] = self.uc

        self.solver.advance( varargin=self.uc)

        self.postStep()
        #print(str(self.solver.ci) + ", " + str(self.solver.xc.transpose())+ ", " + str(self.uc))
        #print(self.solver.xc.transpose())
        #print(self.uc)

    def simulate(self, tspan=None, x0=None, varargin=None):

        if x0 is not None and tspan is not None:
            self.initialize(tspan=tspan, x0=x0)
        else:
            if self.solver.state != self.solver.INITIALIZED:
                print("ERROR!")
            else:
                if tspan is not None:
                    self.initialize(tspan=tspan, x0=self.solver.xc)


        while self.solver.state != self.solver.DONE:
            self.advance(varargin)

        if isinstance(self.cLaw, base):
            (self.uc, rc) = self.cLaw.compute(self.solver.tc, self.solver.xc)
        else:
            (self.uc, rc) = self.cLaw(self.solver.tc, self.solver.xc)

        self.u[:, self.solver.ci:self.solver.ci+1] = self.uc

        t = self.solver.t
        x = self.solver.x
        u = self.u

        sol = structure(t=t,x=x,u=u)
        return sol

    def getState(self):
        sol = structure(t=self.solver.tc, x=self.solver.xc, u=self.uc)
        return sol

    @staticmethod
    def linearControlSys(A, B):
        def LinearCEOM(t, x, u):
            dx = np.matmul(A,x) + np.matmul(B,u)
            return dx

        return LinearCEOM