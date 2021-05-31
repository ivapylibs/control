#trajectory synth.MPC
#uses NLMPC from do-MPC in order to synthesize forward time trajectories and control signals
from trajSynth.base import base
from structures import structure
import numpy as np
import do_mpc
from trajSynth.Models.DiffDrive.template_model import template_model
from trajSynth.Models.DiffDrive.template_mpc import template_mpc
from trajSynth.Models.DiffDrive.template_simulator import template_simulator


class mpcDiff(base):

    def __init__(self,param):
        self.model = template_model()
        self.x0 = param.x0
        #self.tSpan = param.tSpan #tSpan is the total number of steps. The actual tspan is defined in the mpc and sim files
        self.Td = param.Td #number of iterations before MPC recomputes
        self.curTime = 0 #the current time in the simulation, used to ensure the correct position on the path is being pulled
        self.Ts = param.Ts
        self.estimator = do_mpc.estimator.StateFeedback(model)

    def updatefPtr(self,desTraj):
        self.fPtr = desTraj

    def mainloop(self,x0,desTraj):
        self.updatefPtr(desTraj)
        self.mpc = template_mpc(self.model,self.fPtr,self.curTime)
        self.sim = template_simulator(self.model,self.curTime)
        self.mpc.x0 = x0
        self.sim.x0 = x0
        self.estimator.x0 = x0

        self.mpc.set_initial_guess()

        for k in range(int(self.Td/self.Ts)):
            self.curTime += self.Ts
            u0 = self.mpc.make_step(self.x0)
            y_next = self.sim.make_step(u0)
            self.x0 = self.estimator.make_step(y_next)
    #def updateModel(self,nFptr): #update the model to point to a new trajectory instance


    def followPath(self,x0,desTraj):
        """
        Setup graphic:
        """

        #fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(8,5))
        #plt.ion()
        """
        Run MPC main loop:
        """
        for k in range(1):
            self.mainloop(x0,desTraj)
        return self.mpc.data['_x']
