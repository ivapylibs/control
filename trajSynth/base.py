import scipy
import numpy as np

class base(object):

    def __init__(self, theSystem, metaBuilder):
        self.system = theSystem
        self.metaBuilder = metaBuilder

    def setSystem(self, theSystem):
        self.system = theSystem

    def point2point(self, istate, fstate):
        pass

    def followPath(selfistate, desTraj):
        pass

    @staticmethod
    def discretizeLinearCEOM(Ac, Bc, tau, nOrder=4):
        Ad = scipy.linalg.expm(np.matmul(Ac, tau))

        AcPow = np.matmul(np.eye(Ac.shape), tau)
        facti = 1

        Bd = AcPow

        for ii in range(2, nOrder+1):
            AcPow = np.matmul(np.matmul(AcPow, Ac), tau)
            facti = -facti*ii
            Bd = Bd + AcPow/(facti)

        Bd = np.matmul(np.matmul(Ad, Bd), Bc)

        return (Ad, Bd)