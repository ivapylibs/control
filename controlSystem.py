


class controlSystem(object):

    def __init__(self, ceom, trajGenerator, trackBuilder, config=None):

        self.ceom = ceom
        self.trajGen = trajGenerator

        self.trackBuilder = trackBuilder
        self.trackSim = None
        self.simOut = []

        if config is not None:
            self.config = config

        self.iflag = False

    def setInitialState(self, istate):
        self.istate = istate
        self.iflag = True

    def goto(self, fstate, duration=None):

        if not self.iflag:
            print("Error, no initial state")

        if len(self.simOut) == 0:
            tshift = 0
        else:
            tshift = self.simOut[-1].t[-1]

        if duration is not None:
            tspan = [0, duration]
            theTraj = self.trajGen.point2point(self.istate.x, fstate, tspan)
        else:
            theTraj = self.trajGen.point2point(self.istate.x, fstate)

        if self.trackSim is None:
            self.trackSim = self.trackBuilder['firstBuildFromStruct'](self.istate, theTraj)
        else:
            self.trackBuilder['reconfigFromStruct'](self.trackSim, self.istate, theTraj)

        simsol = self.trackSim.simulate()

        simsol.traj = theTraj
        simsol.istate = self.istate
        simsol.fstate = self.trackSim.getState()

        self.addSim(simsol)
        self.iflag = False

        return simsol


    def addSim(self, simOut):
        #if len(self.simOut) == 0:
        self.simOut.append(simOut)