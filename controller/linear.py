from .base import base
import numpy as np
import scipy.linalg
from simController import simController
from structures import structure
from Curves import CurveBase

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

    def __init__(self, xeq=0, ueq=0, K=None):
        super(linear, self).__init__()
        self.xeq = xeq
        self.ueq = ueq

        if K is not None:
            self.set(K=K)


    #============================= setByCARE =============================
    #
    # @brief  Determine feedback control matrix by solving the CARE.
    #
    # @param[in]  A       Estimated linear dynamics.
    # @param[in]  B       Estimated input gain matrix.
    # @param[in]  Q       Target stabilization matrix.
    #
    # @param[out] K       The gains computed.
    # @param[out] P       The pos. def. symmetric (Lyapunov) matrix
    # @param[out] Leig    The eigenvalues (closed-loop).
    #
    def setByCARE(self, A, B, Q):
        [P, Leig, K] = care(A, B, Q)
        K = -K
        self.set(K)

        return (K, P, Leig)


    def setByCareFromStruct(self, cfs):
        (K, P, Leig) = self.setByCARE(A=cfs.A, B=cfs.B, Q=cfs.Q)
        return (K, P, Leig)

    def set(self, K):
        self.K = K


    def regulator(self, xdes):
        if self.K.shape[1] == 2*xdes.size:
            xdes = np.pad(xdes.flatten(), (0,xdes.size), mode='constant').reshape((self.K.shape[1],1))
            print("Added zero velocities")

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
            xdes = np.pad(xdes.flatten(), (0, xdes.size), mode='constant').reshape((self.K.shape[1], 1))
            print("Added zero velocities")

        if cons is None:
            linLaw = self.regulator(xdes=xdes)
        else:
            mins = cons.sat.min * np.ones_like(self.ueq)
            maxs = cons.sat.max * np.ones_like(self.ueq)

            def regLinControlSat(t=None, x=None):
                if x is None:
                    u = self.ueq + np.zeros((self.K.shape[0], 1))
                else:
                    uraw = self.ueq + np.matmul(self.K, (x - xdes))
                    u = np.minimum(maxs, np.maximum(mins, uraw))

                rc = xdes

                return (u, rc)

            linLaw = regLinControlSat

        self.compute = linLaw

        return (linLaw, xdes)

    def noControl(self):
        def doNothing(t=None, x=None):
            return (self.ueq, self.xeq)
        self.compute = doNothing
        return doNothing




    def tracker(self, xdes, uFF=None, stateDep=False):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                rc = xdes
                u = self.ueq +  np.matmul(self.K, x - rc)
            else:
                u = np.zeros(np.shape(self.K)[0], 1)
                rc = np.zeros(np.shape(self.K)[1], 1)
            return u, rc
        def trackLinControlWithFF(t=None, x=None):
            if t is None or x is None:
                rc = np.zeros((self.K.shape[1],1))
                u = np.zeros((self.K.shape[0], 1))
            else:
                rc = xdes(t)
                u = self.ueq + np.matmul(self.K, x-rc) + uFF(t)

            return u, rc

        if(uFF == None):
            linLaw = trackLinControl
        else:
            linLaw = trackLinControlWithFF

        return linLaw

    def trackerNew(self, desTraj):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                rc = desTraj.x(t)
                u = self.ueq +  np.matmul(self.K, x - rc)
            else:
                u = np.zeros(np.shape(self.K)[0], 1)
                rc = np.zeros(np.shape(self.K)[1], 1)
            return u, rc


        if(isinstance(desTraj, CurveBase)):
            self.compute = trackLinControl
            return trackLinControl

    def transpose(self,vec):
        myvec = np.zeros((len(vec),1))
        for i in range(len(vec)):
            myvec[i,0] = vec[i]
        return myvec


    def trackerPath(self,desTraj):
        def trackLinControl(t=None, x=None):
            if(t is not None and x is not None):
                myt = int(t/.01)
                #print(myt)
                #input("Press Enter")
                rc = self.transpose(desTraj[myt])
                u = self.ueq +  np.matmul(self.K, x-rc)
                #print(u)
                #input("Press Enter")
            else:
                u = np.zeros((np.shape(self.K)[0], 1))
                rc = np.zeros((np.shape(self.K)[1], 1))
            return u, rc



        self.compute = trackLinControl
        return trackLinControl


    @staticmethod
    def systemDynamics(A, B):
        def LinearCEOM(t, x, u):
            return np.matmul(A,x) + np.matmul(B,u)

        return LinearCEOM

    @staticmethod
    def structBuilder(ceom, cfs):
        solver = cfs.odeMethod(ceom, cfs.dt)

        def theInitializer(tspan, istate, rsig, uFF, statedep=False):
            control = cfs.controller.tracker(rsig, uFF, statedep)

            theSim = simController(solver, control)
            theSim.initializeByStruct(tspan, istate)
            return theSim

        def theInitializerFromStruct(istate, theTraj):
            if(hasattr(theTraj, 'statedep')):
                if theTraj.statedep is None:
                    theTraj.statedep = False
            else:
                theTraj.statedep = False


            theSim = theInitializer(theTraj.tspan, istate, theTraj.x, theTraj.u, theTraj.statedep)
            return theSim

        def reconfigure(theSim, tspan, istate, rsig, uFF, statedep):
            control = cfs.controller.tracker(rsig, uFF, statedep)
            theSim.setController(control)

            theSim.reset()
            theSim.initializeByStruct(tspan, istate)

        def reconfigureFromStruct(theSim, istate, theTraj):
            if theTraj.statedep is None:
                theTraj.statedep = False

            reconfigure(theSim, theTraj.tspan, istate, theTraj.x, theTraj.u, theTraj.statedep)

        simInit = structure()
        simInit.firstBuild=theInitializer
        simInit.reconfig=reconfigure
        simInit.firstBuildFromStruct=theInitializerFromStruct
        simInit.reconfigFromStruct=reconfigureFromStruct

        return simInit
