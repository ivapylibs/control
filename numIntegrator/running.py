from numIntegrator.base import base

class Running(base):
    def __init__(self, theDiffEq, dt, opts=None):
        super().__init__(theDiffEq, dt, opts=opts)

    def initialize(self, tinfo, x0):
        super().initialize(tinfo, x0)
        self.ci = 1
    
    def advance(self, varargin):
        if(self.state == self.INITIALIZED):
            self.state = self.INTEGRATING

        if (self.state == self.INTEGRATING):   #! Perform integration step
            xn, tn = self.diffeq.runStep(self.tc, self.xc, self.dt, varargin)

            self.xc = xn
            self.tc = tn
            self.ci = self.ci + 1
            # @todo     Do we have to worry about overflow on the counter?

            if (self.ci == (len(self.t)-2)):
                self.state = self.LASTSTEP
        
        elif(self.state == self.LASTSTEP):
            if (self.extra):
                dt = self.t[-1] - self.t[-2]
                xn, tn = self.diffeq.runStep(self.tc, self.xc, dt, varargin)
            else:
                xn, tn = self.diffeq.runStep(self.tc, self.xc, self.dt, varargin)
        self.xc = xn
        self.tc = self.t[-1]
        self.ci = self.ci + 1

        self.state = self.DONE

    def integrate(self, tspan=None, x0=None, varargin=None):
        if(tspan is not None):
            self.initialize(tspan, x0)
        else:
            if(self.state is not self.INITIALIZED):
                raise Exception("Num Integrator Not Initialized")
        
        self.state = self.INTEGRATING
        for ct in self.t:
            self.advance(varargin)
        
        return self.tc, self.xc


