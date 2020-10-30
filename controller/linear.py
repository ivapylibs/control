from base import base
import numpy as np
import scipy.linalg


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

    def set(self, K):
        self.K = K

    def systemDynamics(self, A, B):
        def LinearCEOM(t, x, u):
            return np.matmul(A,x) + np.matmul(B,u)

        return LinearCEOM

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