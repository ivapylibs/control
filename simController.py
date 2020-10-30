# !/usr/bin/env python

import numpy as np
from controller.base import base

class simController(object):

    class solution(object):
        def __init__(self,t,x,u):
            self.t=t
            self.x=x
            self.u=u

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

        self.u = np.zeros((u0.shape[0], self.solver.t.size))

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

    def simulate(self, tspan, x0=None, varargin=None):

        if x0 is not None:
            self.initialize(tspan=tspan, x0=x0)
        else:
            if self.solver.state != self.solver.INITIALIZED:
                print("ERROR!")
            else:
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

        sol = simController.solution(t,x,u)
        return sol
