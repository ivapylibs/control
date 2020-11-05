# !/usr/bin/env python

import numpy as np
from ivacontrol.numIntegrator import numIntegrator

class niODERK2(numIntegrator):
    coeffHeun =  np.array([1.0 , 1.0/2,  1.0/2])
    coeffMidpoint = np.array([1.0/2, 0, 1])
    coeffRalston = np.array([3.0/4 , 1.0/3, 2.0/3])

    coeffs = {
        "Heun": coeffHeun,      # Coefficients for Heun's method.
        "midpoint": coeffMidpoint,          # Coefficients for midpoint method.
        "Ralston": coeffRalston        # Coefficients for Ralston's method.
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
        super(niODERK2, self).__init__(theFunc=theFunc, dt=dt, opts=opts)

        method = opts['method'] if opts is not None and 'method' in opts and opts['method'] in niODERK2.coeffs else 'Heun'
        self.coeffs = niODERK2.coeffs[method]


    #NOTE: What is this supposed to be used? Currently, it doesn't copy everything, such as the coeffs, or method type
    #============================= duplicate =============================
    #
    # @brief  Create a duplicate copy of this type of numerical integrator.
    #
    def duplicate(self, newDynamics=None, newdt=None):
        dynamics = newDynamics if newDynamics is not None else self.dynamics
        dt = newdt if newdt is not None else self.dt

        objCopy = niODERK2(theFunc=dynamics, dt=dt)

        return objCopy



  #============================== runStep ==============================
  #
  # @brief  The actual numerical integration step. Runge-Kutta 4th.
  #
  # Run the Runge-Kutta 4th order integration scheme.
  # 
  #
    def runStep(self, dt, varargin):

        rk1 = dt*self.dynamics(self.tc, self.xc, varargin)
        rk2 = dt*self.dynamics(self.tc + self.coeff[0]*dt, self.xc + self.coeff[0]*rk1, varargin)

        self.xc = self.xc + self.coeff[1]*rk1 + self.coeff[2]*rk2
        self.x[:, self.ci+1] = self.xc

        return self.xc
    

#
#============================ Static Methods ===========================
#


    #============================= integrate =============================
    #
    # @brief  Run an integration calculation with the Runge-Kutta solver
    #         using externally passed information.  For systems that don't
    #         need all of the machinery of the class and just need the basic
    #         integration function.  Everything else is handled externally.
    #
    # @todo   Is this best approach?  Should there be two types of classes?
    #
    @staticmethod
    def integrateODE(dynamics, x0, t0, dT, dt, coeff=None, varargin=None):

        coeff = coeff if coeff is not None else niODERK2.coeffHeun
        
        extra = False
        if dT is None:  #Without changing the order of arguments, can't easily allow this
            tInt = [t0]
            dT = dt
        elif (dT < dt):
            dt = dT
            tInt = [t0]
        else:
            t1   = t0+dT-dt
            tInt = np.arange(start=t0, stop=t1, step=dt)
            extra = tInt[-1] != t1

        nSteps = len(tInt)
        x = np.array(x0)
        t = t0
        
        for ct in range(nSteps):
            rk1 = dt*dynamics(t, x, varargin)
            rk2 = dt*dynamics(t + coeff(1)*dt/2 , x + coeff(1)*rk1, varargin)

            x   = x + coeff[1]*rk1 + coeff[2]*rk2

        if (extra):
            dt = t1 - tInt[-1]
            rk1 = dt*dynamics(t, x, varargin)
            rk2 = dt*dynamics(t + coeff[0]*dt/2 , x + coeff[0]*rk1, varargin)

            x   = x + coeff[1]*rk1 + coeff[2]*rk2
        return x



    #=============================== solve ===============================
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

        scheme = niODERK2(theFunc=theFunc, dt=dt)
        (ti, xi) = scheme.integrate(tspan, x0)

        return (ti, xi)

    #============================== solvePtr =============================
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
            scheme = niODERK2(theFunc=theFunc, dt=dt)
            (ti, xi) = scheme.integrate(tspan, x0)
            return (ti, xi)

        return odeLike

#
# ============================= niODERK2 =============================
