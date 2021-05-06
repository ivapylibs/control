import numpy as np
from Curves.CurveBase import CurveBase

class linepath(CurveBase):
    def __init__(self, tspan = [0,1], xspan = np.array([[0, 1]])):
        super().__init__(tspan)
        self.xspan = xspan

    # t is scalar or numpy array of scalars
    def x(self, t):
        m = (self.xspan[:,1] - self.xspan[:,0])/(self.tspan[1]-self.tspan[0])
        return np.outer(m,(t-self.tspan[0])) + self.xspan[:,0][:, None]

    # t0, t1 are scalars, x0, x1 are column vectors representing states
    def generate(t0, x0, t1, x1):
        self.tspan = [t0, t1]
        self.xspan = np.hstack((x0, x1))