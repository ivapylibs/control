# !/usr/bin/env python

import numpy as np
from ivacontrol.numIntegrator import numIntegrator


class niODERK4(numIntegrator):
    coeffHeun = np.array([1.0, 1.0 / 2, 1.0 / 2])
    coeffMidpoint = np.array([1.0 / 2, 0, 1])
    coeffRalston = np.array([3.0 / 4, 1.0 / 3, 2.0 / 3])

    coeffs = {
        "Heun": coeffHeun,  # Coefficients for Heun's method.
        "midpoint": coeffMidpoint,  # Coefficients for midpoint method.
        "Ralston": coeffRalston  # Coefficients for Ralston's method.
    }

    # =========================== niODERK2 ===========================
    #
    ## @brief     Constructor for the Runge-Kutta 4th order numerical integrator.
    #
    ## @param[in] theFunc     The function to integrate.
    ## @param[in] dt          The timestep to apply.
    ## @param[in] opts        Optional arguments informing integration method.
    #
    def __init__(self, theFunc, dt, opts=None):
        super(niODERK4, self).__init__(theFunc=theFunc, dt=dt, opts=opts)

        #method = opts['method'] if opts is not None and 'method' in opts and opts['method'] in niODERK2.coeffs else 'Heun'
        #self.coeffs = niODERK2.coeffs[method]

    # NOTE: What is this supposed to be used? Currently, it doesn't copy everything, such as the coeffs, or method type
    # ============================= duplicate =============================
    #
    # @brief  Create a duplicate copy of this type of numerical integrator.
    #
    def duplicate(self, newDynamics=None, newdt=None):
        dynamics = newDynamics if newDynamics is not None else self.dynamics
        dt = newdt if newdt is not None else self.dt

        objCopy = niODERK4(theFunc=dynamics, dt=dt)

        return objCopy

    # ============================== runStep ==============================
    #
    # @brief  The actual numerical integration step. Runge-Kutta 4th.
    #
    # Run the Runge-Kutta 4th order integration scheme.
    #
    #
    def runStep(self, dt, varargin):
        self.xc = niODERK4.DoStep(dynamics=self.dynamics, x=self.xc, t=self.tc, dt=dt, varargin=varargin)
        self.x[:, self.ci + 1: self.ci+2] = self.xc

        return self.xc

    #
    # ============================ Static Methods ===========================
    #

    @staticmethod
    def DoStep(dynamics, x, t, dt, varargin):

        rk1 = dt * dynamics(t, x, varargin)
        rk2 = dt * dynamics(t + dt / 2, x + rk1 / 2, varargin)
        rk3 = dt * dynamics(t + dt / 2, x + rk2 / 2, varargin)
        rk4 = dt * dynamics(t + dt, x + rk3, varargin)

        new_xc = x + (1.0 / 6) * rk1 + (1.0 / 3) * rk2 + (1.0 / 3) * rk3 + (1.0 / 6) * rk4
        return new_xc

    # ============================= integrate =============================
    #
    # @brief  Run an integration calculation with the Runge-Kutta solver
    #         using externally passed information.  For systems that don't
    #         need all of the machinery of the class and just need the basic
    #         integration function.  Everything else is handled externally.
    #
    # @todo   Is this best approach?  Should there be two types of classes?
    #
    @staticmethod
    def integrateODE(dynamics, x0, t0, dT, dt, varargin=None):

        extra = False
        if dT is None:  # Without changing the order of arguments, can't easily allow this
            tInt = [t0]
            dT = dt
        elif (dT < dt):
            dt = dT
            tInt = [t0]
        else:
            t1 = t0 + dT - dt
            tInt = np.arange(start=t0, stop=t1, step=dt)
            extra = tInt[-1] != t1

        nSteps = len(tInt)
        x = np.array(x0)
        t = t0

        for ct in range(nSteps):
            x = niODERK4.DoStep(dynamics=dynamics, x=x, t=t, dt=dt, varargin=varargin)

        if extra:
            dt = t1 - tInt[-1]
            x = niODERK4.DoStep(dynamics=dynamics, x=x, t=t, dt=dt, varargin=varargin)

        return x

    # =============================== solve ===============================
    #
    # @brief  Single invocation version that tries to look like odeXY
    #
    # Single invocation version whose arguments are meant to resemble
    # the built-in odeXY Matlab functions.  One difference is that
    # self class requires the dt to be specified. It is sent in as
    # the last mandatory argument.
    #
    @staticmethod
    def solve(theFunc, tspan, x0, dt, varargin=None):

        scheme = niODERK4(theFunc=theFunc, dt=dt)
        (ti, xi) = scheme.integrate(tspan, x0)

        return (ti, xi)

    # ============================== solvePtr =============================
    #
    # @brief  Returns handle to single invocation version looking like odeXY
    #
    # Returns a function handle whose function arguments resemble the
    # built-in odeXY Matlab functions.  Since the dt needs to be specified,
    # it gets pre-specified then used during the invocation.  Allows existing
    # code to transition to self custom class version without much hassle.
    #
    @staticmethod
    def solvePtr(dt):
        def odeLike(theFunc, tspan, x0, varargin=None):
            scheme = niODERK4(theFunc=theFunc, dt=dt)
            (ti, xi) = scheme.integrate(tspan, x0)
            return (ti, xi)

        return odeLike

#
# ============================= niODERK2 =============================
