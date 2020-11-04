import numpy as np
import math
from niODERK4 import niODERK4
from controller.linear import linear, care
from simController import simController
import matplotlib.pyplot as plt
from trajSynth.naive import naive
from structures import structure

#==[1] Configure the linear system.
sys = structure()

sys.A = np.array([[0, 1], [0.15, 0.25]])
sys.B = np.array([[0], [1]])
sys.solver = niODERK4
sys.dt = 0.01
sys.tspan = [0, 10]
sys.rtol = 4e-3

Q = 50 * np.diag([6.00, 3.50])
[P, L, K] = care(sys.A, sys.B, Q)
sys.K = -K

sys.cons.sat.min = -5
sys.cons.sat.max = 5

#--[2] Instantiate the trajectory synthesizer.
metaBuilder = structure(regulator=naive.trajBuilder_LinearReg, tracker=naive.trajBuilder_LinearTracker)

tMaker = naive(theSystem=sys, metaBuilder=metaBuilder)

print('Done contructing class. Now constructing trajectory')
tMaker.point2point(istate=np.array([[3],[0]]), fstate=np.array([[0.3],[0]]))

pdt = 0.03
time = np.arange(start=sys.tspan[0], stop=sys.tspan[1], step=pdt)

plt.figure(1)
xv = np.array(tMaker.xTraj(time))
plt.plot(time, xv.transpose())

plt.figure(2)
uv = np.array(tMaker.uTraj(time))
plt.plot(time, uv[0,:])

plt.show()