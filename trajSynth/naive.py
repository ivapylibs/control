from base import base
from simController import simController
from controller.linear import linear
import numpy as np
import scipy.interpolate

class sys(object):

    def __init__(self, A, B, K, solver, tspan, dt, rtol):
        self.A=A
        self.B=B
        self.K=K
        self.solver = solver
        self.tspan = tspan
        self.dt = dt
        self.rtol = rtol


class naive(base):

    def __init__(self, theSystem, metaBuilder):
        super(naive, self).__init__(theSystem=theSystem, metaBuilder=metaBuilder)

        self.regulator = metaBuilder['regulator'](theSystem)
        self.tracker = metaBuilder['tracker'](theSystem)

    def point2point(self, istate, fstate, tspan=None):
        ceom = self.regulator(istate, fstate)

        if tspan is not None:
            simout = ceom.simulate(tspan=tspan)
        else:
            simout = ceom.simulate()

        if simout.x.shape[0]==1 and False:
            self.xTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x)
        else:
            xgrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x[ind], bounds_error=False) for ind in range(simout.x.shape[0])]
            self.xTraj = lambda t: np.array([g([t]) for g in xgrid_interps]).reshape((-1,1))

        if simout.u.shape[0] == 1 and False:
            self.uTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u)
        else:
            ugrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u[ind], bounds_error=False) for ind in range(simout.u.shape[0])]
            self.uTraj = lambda t: np.array([g([t]) for g in ugrid_interps]).reshape((-1, 1))

        tSol = linear.TrajStruct(tspan=simout.t, x=self.xTraj, u=self.uTraj)
        return tSol


    @staticmethod
    def trajBuilder_LinearReg(sys):

        rtol = sys['rtol']
        tspan = sys['tspan']
        A = sys['A']
        B = sys['B']
        K = sys['K']
        dt = sys['dt']

        xdes = []

        def linSysBuilder(istate, fstate):
            if 'cons' in sys:
                (ctrl, xdes) = theControl.regulatorConstrained(fstate, sys['cons'])
            else:
                (ctrl, xdes) = theControl.regulator(fstate)

            csim = simController(solver, ctrl)
            csim.initialize(tspan=tspan,x0=istate)

            return csim

        def detectArrival(t, x):
            errNorm = np.linalg.norm(x - xdes) -  rtol
            if errNorm < 0:
                errNorm = 0

            isterminal = True
            direction = 0

            return (errNorm, isterminal, direction)

        opts = {'event': detectArrival}

        ceom =  linear().systemDynamics(A, B)
        #ceom =  simController.linearControlSys(A, B)
        solver = sys['solver'](ceom, dt, opts)

        theControl = linear()

        theControl.set(K)

        theBuilder = linSysBuilder

        return theBuilder


    @staticmethod
    def trajBuilder_LinearTracker(sys):

        rtol = sys['rtol']
        tspan = sys['tspan']
        A = sys['A']
        B = sys['B']
        K = sys['K']
        dt = sys['dt']

        xdes = []

        def linSysBuilder(istate, desTraj):
            ctrl = theControl.tracker(desTraj['x'], desTraj['u'], desTraj['statedep'])

            csim = simController(solver, ctrl)

            csim.initialize(tspan=desTraj['tspan'], x0=istate['x'])

            return csim

        def detectArrival(t, x):
            errNorm = np.linalg.norm(x - xdes) - rtol
            if errNorm < 0:
                errNorm = 0

            isterminal = True
            direction = 0

            return (errNorm, isterminal, direction)

        opts = {'event': detectArrival}

        ceom = linear().systemDynamics(A,B)
        #ceom = simController.linearControlSys(A, B)
        solver = sys['solver'](ceom, dt, opts)

        theControl = linear()

        theControl.set(K)

        theBuilder = linSysBuilder

        return theBuilder