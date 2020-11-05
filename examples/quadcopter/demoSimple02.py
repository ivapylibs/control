from examples.quadcopter.linQuadCopter import linQuadCopter
from ivacontrol.niODERK4 import niODERK4
from ivacontrol.trajSynth.naive import naive
from ivacontrol.controller.linear import linear
from ivacontrol.structures import structure
import numpy as np
from numpy.matlib import repmat

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
tsys.Q  = (4e8)*np.diag([15, 15, 20, 0.1, 0.1, 0.05, 80, 80, 140, 0.3, 0.3, 0.1])
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
csys.Q  = (1e8)*np.diag([8, 8, 14, 2, 2, 1, 15, 15, 20, 3, 3, 2]) # Can be weaker than the tsys.Q
#csys.Q  = np.diag([200, 300, 400, 100, 10, 10, 20, 20, 10, 10, 10, 10])
csys.controller.setByCARE(A, B, csys.Q)

tTracker = csys.controller.structBuilder(linDyn, csys)

#--[1.4] Pack together into control system.
#
cSim = linQuadCopter(linDyn, tMaker, tTracker)

#==[2] Simulate the control system.
#
simType = 'curve03'

desTraj = structure()
if simType == 'track1':     #Justin: This is probably of interest.
    v = 0.15
    # Linear growth in position, constant velocity
    desTraj.x = lambda t: v*np.vstack((t, t, 0.5*t,
                                     np.zeros((3,t.size)),
                                     repmat(np.array([[1],[1],[0.5]]),1,t.size),
                                     np.zeros((3,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,15])
    desTraj.statedep = False
    xi = np.zeros((12,1))

elif simType == 'track2':
    v = 0.15
    # Linear growth in position, constant velocity
    desTraj.x = lambda t: v*np.vstack((t, t, 0.5*t,
                                     np.zeros((3,t.size)),
                                     repmat(np.array([[1],[1],[0.5]]),1,t.size),
                                     np.zeros((3,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,15])
    desTraj.statedep = False
    xi = np.vstack((0.3, -0.1, 0,0,0,0, np.zeros((6,1)))) #Starts at wrong position.

elif simType == 'sine01':
    # Sinusoidal position and velocity
    desTraj.x = lambda t: np.vstack((np.sin(t), np.sin(t), 1-np.cos(0.5*t),
                                     np.zeros((3,t.size)),
                                     np.cos(t), np.cos(t), 0.5*np.sin(0.5*t),
                                     np.zeros((3,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,25])
    desTraj.statedep = False
    xi = np.zeros((12,1))
    xi[6] = 1
    xi[7] = 1

elif simType == 'curve01':
    # Sinusoidal position and velocity
    desTraj.x = lambda t: np.vstack((np.sin(t), 1-np.cos(t),
                                     np.zeros((4,t.size)),
                                     np.cos(t), np.sin(t),
                                     np.zeros((4,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,15])
    desTraj.statedep = False
    xi = np.zeros((12,1))
    xi[6] = 1

elif simType == 'curve02':  #Justin: This is probably of interest.
    # Low freq. sinusoidal position and velocity so looks like an arc.
    wa = 0.25
    desTraj.x = lambda t: np.vstack((np.sin(wa*t), 1-np.cos(wa*t),
                                     np.zeros((4,t.size)),
                                    wa*np.cos(wa*t), wa*np.sin(wa*t),
                                     np.zeros((4,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,15])
    desTraj.statedep = False
    xi = np.zeros((12,1))
    xi[6] = wa             # Assume already moving in right direction.

elif simType == 'curve03':  #Justin: This is probably of interest.
    # Low freq. sinusoidal position and velocity so looks like an arc.
    # Slow change in height.
    wa = 0.25
    vz = 0.05
    desTraj.x = lambda t: np.vstack((np.sin(wa*t), 1-np.cos(wa*t), vz*t,
                                     np.zeros((3,t.size)),
                                    wa*np.cos(wa*t), wa*np.sin(wa*t), vz*np.ones((1, t.size)),
                                     np.zeros((3,t.size))))
    desTraj.u = lambda t: np.zeros((4,t.size))  #No feed-forward.
    desTraj.tspan = np.array([0,10])
    desTraj.statedep = False
    xi = np.zeros((12,1))
    xi[6] = wa             # Assume already moving in right direction.

initState = structure()
initState.x = xi
cSim.setInitialState(initState)
sol = cSim.follow(desTraj)

#==[3] Plot outcomes.
#


cSim.plotSignals(1, desTraj);
