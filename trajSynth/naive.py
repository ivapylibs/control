from base import base
from simController import simController
from controller.linear import linear
import numpy as np
import scipy.interpolate
from structures import structure

class naive(base):

    def __init__(self, theSystem, metaBuilder):
        super(naive, self).__init__(theSystem=theSystem, metaBuilder=metaBuilder)

        self.regulator = metaBuilder.regulator(theSystem)
        self.tracker = metaBuilder.tracker(theSystem)

    def point2point(self, istate, fstate, tspan=None):
        ceom = self.regulator(istate, fstate)

        if tspan is not None:
            simout = ceom.simulate(tspan=tspan)
        else:
            simout = ceom.simulate()

        if simout.x.shape[0]==1 and False:
            xTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x)
        else:
            xgrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x[ind], bounds_error=False) for ind in range(simout.x.shape[0])]
            xTraj = lambda t: np.array([g([t]) for g in xgrid_interps]).reshape((-1,1))

        if simout.u.shape[0] == 1 and False:
            uTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u)
        else:
            ugrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u[ind], bounds_error=False) for ind in range(simout.u.shape[0])]
            uTraj = lambda t: np.array([g([t]) for g in ugrid_interps]).reshape((-1, 1))

        def ValMapper(f):
            def g(t):
                try:
                    outv = [f(x) for x in t]
                except TypeError:
                    outv = f(t)
                return outv
            return g

        self.xTraj = ValMapper(xTraj)
        self.uTraj = ValMapper(uTraj)

        tSol = structure(tspan=simout.t[[0,-1]], x=self.xTraj, u=self.uTraj)
        return tSol

    def followPath(self, istate, desTraj):
        ceom = self.tracker(istate, desTraj)

        simout = ceom.simulate()

        if simout.x.shape[0]==1 and False:
            xTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x)
        else:
            xgrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.x[ind], bounds_error=False) for ind in range(simout.x.shape[0])]
            xTraj = lambda t: np.array([g([t]) for g in xgrid_interps]).reshape((-1,1))

        if simout.u.shape[0] == 1 and False:
            uTraj = scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u)
        else:
            ugrid_interps = [scipy.interpolate.RegularGridInterpolator(points=(simout.t,), values=simout.u[ind], bounds_error=False) for ind in range(simout.u.shape[0])]
            uTraj = lambda t: np.array([g([t]) for g in ugrid_interps]).reshape((-1, 1))

        def ValMapper(f):
            def g(t):
                try:
                    outv = [f(x) for x in t]
                except TypeError:
                    outv = f(t)
                return outv
            return g

        self.xTraj = ValMapper(xTraj)
        self.uTraj = ValMapper(uTraj)

        tSol = structure(tspan=simout.t[[0, -1]], x=self.xTraj, u=self.uTraj)
        return tSol


    @staticmethod
    def trajBuilder_LinearReg(sys):

        #Doesn't do anything?
        xdes = []

        def linSysBuilder(istate, fstate):
            if 'cons' in sys:
                (ctrl, xdes) = theControl.regulatorConstrained(fstate, sys.cons)
            else:
                (ctrl, xdes) = theControl.regulator(fstate)

            if theControl.K.shape[1] == 2 * istate.size:
                istate = np.pad(istate.flatten(), (0,istate.size), mode='constant').reshape((theControl.K.shape[1],1))

            csim = simController(solver, ctrl)
            csim.initialize(tspan=sys.tspan,x0=istate)

            return csim

        # Doesn't do anything?
        def detectArrival(t, x):
            errNorm = np.linalg.norm(x - xdes) -  sys.rtol
            if errNorm < 0:
                errNorm = 0

            isterminal = True
            direction = 0

            return (errNorm, isterminal, direction)

        # Doesn't do anything?
        opts = structure(event=detectArrival)

        ceom =  linear().systemDynamics(sys.A, sys.B)
        solver = sys.solver(ceom, sys.dt, opts)

        theControl = linear()
        if 'K' in sys:
            theControl.set(sys.K)
        elif 'Q' in sys:
            theControl.setByCareFromStruct(sys)

        theBuilder = linSysBuilder

        return theBuilder


    @staticmethod
    def trajBuilder_LinearTracker(sys):

        xdes = []

        def linSysBuilder(istate, desTraj):
            ctrl = theControl.tracker(desTraj.x, desTraj.u, desTraj.statedep)

            if theControl.K.shape[1] == 2 * istate.x.size:
                istate.x = np.pad(istate.x.flatten(), (0,istate.x.size), mode='constant').reshape((theControl.K.shape[1],1))

            csim = simController(solver, ctrl)

            csim.initialize(tspan=desTraj.tspan, x0=istate.x)

            return csim

        def detectArrival(t, x):
            errNorm = np.linalg.norm(x - xdes) - sys.rtol
            if errNorm < 0:
                errNorm = 0

            isterminal = True
            direction = 0

            return (errNorm, isterminal, direction)

        opts = structure(event=detectArrival)

        ceom = linear().systemDynamics(sys.A,sys.B)
        #ceom = simController.linearControlSys(A, B)
        solver = sys.solver(ceom, sys.dt, opts)

        theControl = linear()
        if 'K' in sys:
            theControl.set(sys.K)
        elif 'Q' in sys:
            theControl.setByCareFromStruct(sys)

        theBuilder = linSysBuilder

        return theBuilder