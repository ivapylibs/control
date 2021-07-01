from os import stat
from numpy import dtype
from numIntegrator.simulation import simulation
import pdb

class SimFirstOrder(simulation):
    def __init__(self, theGroup, theDiffEq, dt, opts=None) -> None:
        super().__init__(theDiffEq, dt, opts=opts)
        self.group = theGroup

    def initialize(self, tinfo, x0):
        super().initialize(tinfo, x0)

        self.x = [self.group()]*len(self.t)
        self.x[0] = x0
        self.ci = 0

    
    def advance(self, varargin):
        if (self.state == self.INITIALIZED):
            self.state = self.INTEGRATING;      #! Upgrade to integrating.
        
        if (self.state == self.INTEGRATING):   #! Perform integration step
            xi = self.diffeq(self.tc, self.xc, varargin)
            #pdb.set_trace()
            gi = self.group.exp(xi, self.dt)
            xn = gi* self.xc
            self.xc = xn

            self.tc = self.tc + self.dt
            self.ci += 1
            self.x[self.ci] = xn

            if (self.ci == (len(self.t)-2)):
                self.state = self.LASTSTEP

        elif(self.state == self.LASTSTEP):
            if (self.extra):
                dt = self.t[-1] - self.t[-2]

                xi = self.diffeq(self.tc, self.xc, varargin)
                gi = self.group.exp(xi, dt)
                xn = gi * self.xc; 
            else:
                xi = self.diffeq(self.tc, self.xc, varargin)
                gi = self.group.exp(xi, self.dt)
                xn = gi * self.xc; 
            
            self.xc = xn
            self.tc = self.t[-1] #! or set to tn?
            self.ci = self.ci + 1
            self.x[self.ci] = xn

            self.state = self.DONE

    @staticmethod
    def buildFromArgs(theScheme):
        def makeSim(dynamics, dt, opts=None):
            if(opts is None):
                return SimFirstOrder(dynamics, dt)
            else:
                return SimFirstOrder(dynamics, dt, opts)
        
        return makeSim
