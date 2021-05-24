#trajectory synth.MPC
#uses NLMPC from do-MPC in order to synthesize forward time trajectories and control signals
from base import base
from structures import structure
import numpy as np
import do_mpc
from Models.DiffDrive.template_model import template_model
from Models.DiffDrive.template_mpc import template_mpc
from Models.DiffDrive.template_simulator import template_simulator


class mpc(base):
    def fPtr(x,ptr):
    def __init__(self,model,mpc,sim,param):
        self.model = model
        self.mpc = template_mpc(model,
        self.sim = sim
        self.x0 = param.x0
        self.tSpan = param.tSpan #tSpan is the total number of steps. The actual tspan is defined in the mpc and sim files
        self.Td = param.Td #number of iterations before MPC recomputes
        self.curTime = 0 #the current time in the simulation, used to ensure the correct position on the path is being pulled
        self.estimator = do_mpc.estimator.StateFeedback(model)

    def mainloop(self,x0):
        self.mpc.x0 = self.x0
        self.sim.x0 = self.x0
        self.estimator.x0 = self.x0

        self.mpc.set_initial_guess()

        for k in range(self.nextCompute):
            u0 = self.mpc.make_step(self.x0)
            y_next = self.sim.make_step(u0)
            self.x0 = self.estimator.make_step(y_next)
    #def updateModel(self,nFptr): #update the model to point to a new trajectory instance


    def followPath(self,x0):
        """
        Setup graphic:
        """

        #fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(8,5))
        #plt.ion()
        """
        Run MPC main loop:
        """
        for k in range(1):
            self.mainloop(x0)
