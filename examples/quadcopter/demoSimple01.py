from examples.quadcopter.linQuadCopter import linQuadCopter
from ivacontrol.niODERK4 import niODERK4
from ivacontrol.trajSynth.naive import naive
from ivacontrol.controller.linear import linear
from ivacontrol.structures import structure
import numpy as np

#==[1] Setup the control system.
#
#--[1.1] Define the dynamics of the system.
#
Parms = structure()
Parms.m  =  1.52           #  kg
Parms.r  =  0.09           #  m
Parms.Jx = 0.0347563       #  kg-m^2
Parms.Jy = 0.0458529       #  kg-m^2
Parms.Jz = 0.0977          #  kg.m^2
Parms.KT = 0.00000854858   #  ???
Parms.KD = 0.016*Parms.KT  #  ???
Parms.g  = 9.80            #  m/s^2
(linDyn, A, B, uEq) = linQuadCopter.dynamicsAtHover(Parms)

#--[1.2] Define the trajectory synthesizer.
#
tsys = structure()
tsys.A  = A
tsys.B  = B
tsys.Q  = (4e8)*np.diag([4, 4, 9, 4, 4, 1, 3, 3, 10, 7, 7, 3])
#tsys.Q  = np.diag([200, 300, 400, 100, 10, 10, 20, 20, 10, 10, 10, 10])
tsys.dt = 0.05
tsys.solver = niODERK4
tsys.tspan  = [0, 40]

metaBuilder = structure()
metaBuilder.regulator = naive.trajBuilder_LinearReg
metaBuilder.tracker   = naive.trajBuilder_LinearTracker

tMaker = naive(tsys, metaBuilder)

#--[1.3] Define the trajectory tracking builder.
#
csys = structure()
csys.dt = 0.05
csys.odeMethod = niODERK4
csys.controller = linear(np.zeros((12,1)),0*uEq)
csys.Q  = (1e8)*np.diag([2, 2, 6, 4, 4, 1, 3, 3, 10, 9, 9, 3]) # Can be weaker than the tsys.Q
#csys.Q  = np.diag([200, 300, 400, 100, 10, 10, 20, 20, 10, 10, 10, 10])
csys.controller.setByCARE(A, B, csys.Q)

tTracker = csys.controller.structBuilder(linDyn, csys)

#--[1.4] Pack together into control system.
#
cSim = linQuadCopter(linDyn, tMaker, tTracker)

#==[2] Simulate the control system.
#
simType = 'reg2'
if simType == 'regX':
    xi = np.array([0, 0, 0, 0, 0, 0])
    xf = np.array([1, 0, 0, 0, 0, 0])
elif simType == 'regY':
    xi = np.array([0, 0, 0, 0, 0, 0])
    xf = np.array([0, 1, 0, 0, 0, 0])
elif simType == 'regZ':
    xi = np.array([0, 0, 0, 0, 0, 0])
    xf = np.array([0, 0, 1, 0, 0, 0])
elif simType =='reg1':
    xi = np.array([0, 0, 0, 0, 0, 0])
    xf = np.array([1, 1,0.5,0,0,0])
elif simType =='reg2':
    xi = np.array([1, -0.1, 0, 0, 0, 0])
    xf = np.array([1.85, 1, 1.2, 0, 0, 0])

initState = structure()
initState.x = xi
cSim.setInitialState(initState)
sol = cSim.goto(xf, 40)

#==[3] Plot outcomes.
#
desTraj = structure()
desTraj.x = cSim.trajGen.xTraj
desTraj.u = cSim.trajGen.uTraj

cSim.plotSignals(1, desTraj)
