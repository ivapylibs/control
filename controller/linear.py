from base import base
import numpy as np
import scipy.linalg
from simController import simController

def care(A, B, Q, R=None):

    """Solve the continuous time lqr controller.

    dx/dt = A x + B u

    cost = integral x.T*Q*x + u.T*R*u
    """

    # ref Bertsekas, p.151

    if R is None:
      R = np.eye(B.shape[1])

    # first, try to solve the ricatti equation
    X = np.array(scipy.linalg.solve_continuous_are(A, B, Q, R))

    # compute the LQR gain
    K = np.array(np.dot(scipy.linalg.inv(R), (np.dot(B.T, X))))

    eigVals, eigVecs = scipy.linalg.eig(A - np.dot(B, K))

    return X, eigVals, K



class linear(base):

    class TrajStruct(object):
        def __init__(self, tspan, x, u, statedep=False):
            self.tspan = tspan
            self.x = x
            self.u = u
            self.statedep = statedep

    def __init__(self, xeq=0, ueq=0, K=None):
        super(linear, self).__init__()
        self.xeq = xeq
        self.ueq = ueq

        if K is not None:
            self.set(K=K)

    def set(self, K):
        self.K = K

    def systemDynamics(self, A, B):
        def LinearCEOM(t, x, u):
            return np.matmul(A,x) + np.matmul(B,u)

        return LinearCEOM


    def regulator(self, xdes):
        if self.K.shape[1] == 2*xdes.size:
            #TODO: Add zero velocities
            pass
            print("Needs to add zero velocities")

        def regLinControl(t=None, x=None):
            if x is None:
                u = self.ueq + np.zeros((self.K.shape[0],1))
            else:
                u = self.ueq + np.matmul(self.K, (x-xdes))

            rc = xdes
            return (u, rc)


        linlaw = regLinControl
        self.compute = linlaw

        return (linlaw, xdes)


    def regulatorConstrained(self, xdes, cons=None):
        if self.K.shape[1] == 2*xdes.size:
            #TODO: Add zero velocities
            pass
            print("Needs to add zero velocities")

        if cons is None:
            linLaw = self.regulator(xdes=xdes)
        else:
            mins = cons.sat.min * np.ones_like(self.ueq)
            maxs = cons.sat.max * np.ones_like(self.ueq)

            def regLinControlSat(t=None, x=None):
                if x is None:
                    u = self.ueq + np.zeros((self.K.shape[0], 1))
                else:
                    u = self.ueq + np.matmul(self.K, (x - xdes))
                    u = np.minimum(maxs, np.maximum(mins, u))

                rc = xdes

                return (u, rc)

            linLaw = regLinControlSat

        self.compute = linLaw

        return (linLaw, xdes)









    def tracker(self, xdes, uFF=None, stateDep=False):

        def trackLinControlWithFF(t=None, x=None):
            if t is None or x is None:
                rc = np.zeros((self.K.shape[1],1))
                u = np.zeros((self.K.shape[0], 1))
            else:
                rc = xdes(t)
                u = self.ueq + np.matmul(self.K, x-rc) + uFF(t)

            return u, rc



        linLaw = trackLinControlWithFF

        return linLaw


    @staticmethod
    def structBuilder(ceom, cfs):
        solver = cfs['odeMethod'](ceom, cfs['dt'])

        def theInitializer(tspan, istate, rsig, uFF, statedep=False):
            control = cfs['controller'].tracker(rsig, uFF, statedep)

            theSim = simController(solver, control)
            theSim.initializeByStruct(tspan, istate)
            return theSim

        def theInitializerFromStruct(istate, theTraj):
            if theTraj.statedep is None:
                theTraj.statedep = False

            theSim = theInitializer(theTraj.tspan, istate, theTraj.x, theTraj.u, theTraj.statedep)
            return theSim

        def reconfigure(theSim, tspan, istate, rsig, uFF, statedep):
            control = cfs['controller'].tracker(rsig, uFF, statedep)
            theSim.setController(control)

            theSim.reset()
            theSim.initializeByStruct(tspan, istate)

        def reconfigureFromStruct(theSim, istate, theTraj):
            if theTraj.statedep is None:
                theTraj.statedep = False

            reconfigure(theSim, theTraj.tspan, istate, theTraj.x, theTraj.u, theTraj.statedep)

        simInit = {'firstBuild': theInitializer, 'reconfig': reconfigure, 'firstBuildFromStruct': theInitializerFromStruct, 'reconfigFromStruct': reconfigureFromStruct}

        return simInit