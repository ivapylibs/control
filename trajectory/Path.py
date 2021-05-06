import numpy as np

class Path:
    def __init__(self, curve, tspan):
        self.xt = curve
        self.tspan = tspan

    def x(self, t):
        return self.xt.x(t)

    def segment(self, tspan):
        return Path(self.xt, tspan)

    # TODO: Implement
    @staticmethod
    def fromPoints(pts, times):
        return
