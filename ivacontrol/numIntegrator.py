# ============================= numIntegrator =============================
#
## @class   numIntegrator
#
## @brief   Simple numerical integrator interface possibly with less
#           capabilities on the numerics, but more flexibility with
#           regards to control (adaptive, optimal, etc.).
#
# ============================= numIntegrator =============================

#
## @file    numIntegrator.py
#
## @author  Patricio A. Vela,   pvela@gatech.edu
## @date    2020/09/07
#
## @note    Translated from Matlab version.
#
# NOTE ON FORMATTING:
#   set indent to 2 spaces.
#   set tab to 4 spaces (with conversion to spaces).
#
# ============================= numIntegrator =============================


# !/usr/bin/env python

import numpy as np


class numIntegrator(object):
    NOTREADY = 0
    INITIALIZED = 1
    INTEGRATING = 2
    LASTSTEP = 3
    DONE = 4

    # =========================== numIntegrator ===========================
    #
    ## @brief     Constructor for the numerical integrator class
    #
    ## @param[in] theFunc     The function to integrate.
    ## @param[in] dt          The timestep to apply.
    ## @param[in] opts        Optional arguments informing integration method.
    #
    def __init__(self, theFunc, dt, opts=None):
        self.dynamics = theFunc  # Specification of dynamics to integrate (function)
        self.dt = dt  # The numerical time step to use.

        self.opts = [] if opts is not None else opts  # Additional options structure.

        self.state = self.NOTREADY

    # ============================= initialize ============================
    #
    ## @brief     Prepare for a new numerical integration run.
    #
    ## @param[in] tspan       The time span as a range.
    ## @param[in] x0          The initial condition.
    #
    def initialize(self, tspan, x0):
        x0 = np.array(x0).reshape((-1,1))
        self.extra = False
        tInt = np.arange(start=tspan[0], stop=tspan[1], step=self.dt)

        if (tInt[-1] != tspan[1]):
            self.extra = True
            tInt = np.append(tInt, tspan[1])

        self.x = np.zeros((x0.size, len(tInt)))
        self.t = tInt
        self.x[:, 0:1] = x0

        self.xc = x0
        self.tc = tspan[0]
        self.ci = 0

        self.state = self.INITIALIZED

    # =============================== reset ===============================
    #
    def reset(self):

        if ((self.state == self.INITIALIZED) or (self.state == self.NOTREADY)):
            return

        x0 = self.x[:, 0]
        tspan = np.array([self.t[0], self.t[-1]])

        self.initialize(tspan, x0)

    # ============================== preStep ==============================
    #
    def preStep(self):
        pass

    # ============================== runStep ==============================
    #
    def runStep(self, dt, vars):
        pass

    # ============================== postStep =============================
    #
    def postStep(self):
        pass

    # ============================== advance ==============================
    #
    def advance(self, varargin):
        if (self.state == self.INITIALIZED):
            self.state = self.INTEGRATING  # Upgrade to integrating.

        if (self.state == self.INTEGRATING):  # Perform integration step.
            self.preStep()
            self.runStep(self.dt, varargin)
            self.postStep()

            self.ci = self.ci + 1
            self.tc = self.tc + self.dt

            if (self.ci == (len(self.t) - 2)):
                self.state = self.LASTSTEP


        elif (self.state == self.LASTSTEP):  # Integrate and set to done.
            self.preStep()

            if (self.extra):
                dt = self.t[-1] - self.t[-2]
                self.runStep(dt, varargin)
            else:
                self.runStep(self.dt, varargin)

            self.postStep()

            self.ci = self.ci + 1
            self.tc = self.t[-1]

            self.state = self.DONE

    # ============================= integrate =============================
    #
    ## @brief Run the numerical integration scheme.
    #
    # This is the short and long form version of numerical integration.  If the
    # system has been initialized, then tspan and x0 are optional arguments.  The
    # single invocation version would call with all arguments.  The
    # multi-invocation version would first initialize then integrate as separate
    # calls.
    #
    ## @param[in] tspan       The time span as a range.
    ## @param[in] x0          The initial condition.
    ## @param[in] varargin    Additional (optional) arguments sent to function.
    #
    def integrate(self, tspan=None, x0=None, varargin=None):
        if tspan is not None and x0 is not None:
            self.initialize(tspan=tspan, x0=x0)

        else:
            if (self.state is not self.INITIALIZED):
                print('numIntegrator: Not INITIALIZED.  Cannot simulate')
                return

        self.state = self.INTEGRATING
        for ct in self.t:
            self.advance(varargin=varargin)

        tv = self.t
        xv = self.x

        return (tv, xv)

#
# ============================= numIntegrator =============================
