import numpy as np
from controller.PathTracker.timepoints import timepoints
from controller.linear import linear
from Curves import Explicit
import linepath
from matplotlib import pyplot as plt
from structures import structure
from niODERK4 import niODERK4

A = np.zeros((2,2))
B = np.eye(2)
Q = np.eye(2)

linctrl = linear(np.array([[0,0], [0,0]]))
linctrl.setByCARE(A, B, Q)
linctrl.noControl()

leom = linear.systemDynamics(A, B)


pathgen = linepath.linepath()

ts = structure()
ts.Th = 0.5
ts.Td = 0.2

cfS = structure()
cfs = structure(dt=0.05, odeMethod=niODERK4, controller=timepoints(pathgen, linctrl, ts))

tspan = [0,10]
fCirc = lambda t: np.array([[sin(t/2)] , [1-cos(t/2)]])
path = Explicit(tspan, fCirc)
desTraj = trajectory.path(path, tspan)