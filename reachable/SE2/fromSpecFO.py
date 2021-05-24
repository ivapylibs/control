import numpy as np
from reachable.fromSpec import FromSpec

class FromSpecFO(FromSpec):
    def __init__(self, specs):
        super().__init__(specs)
        self.aTol = 1e-6

    

