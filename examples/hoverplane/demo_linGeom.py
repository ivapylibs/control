from examples.hoverplane.hoverplane import hoverplane
from controller.SE2.linearSO import linearSO
from controller.linear import linear
from structures import structure
import numpy as np
from niODERK4 import niODERK4
from trajSynth.naive import naive

parms = structure()
parms.linD = 0*np.array([0.01, 0.023])
parms.angD = 0*0.015
parms.Md   = 0*0.50          # Mr != 0 :: weather-vane effect.
parms.Ma   = 0*0.02          # Ma != 0 :: weather-vane effect.
parms.lagF = 1            # lagF = 1 :: perfect cancellation.

nuEq = 10

(nlDyn, A, B, uEq) = hoverplane.dynamicsForward(parms, nuEq)


tsys = structure()
tsys.A  = A
tsys.B  = B
tsys.Q  = 4*np.diag([6, 1, 3, 10, 3, 17])
tsys.dt = 0.01
tsys.solver = niODERK4
tsys.tspan  = [0, 10]

metaBuilder = structure(regulator = naive.trajBuilder_LinearReg, tracker= naive.trajBuilder_LinearTracker)

tMaker = naive(tsys, metaBuilder)

csys = structure()
csys.dt = 0.01
csys.odeMethod  = niODERK4
csys.controller = linearSO(np.array([0,0,0,nuEq,0,0]).reshape((6,1)),uEq)

manual = True
if(manual):
    K = np.array([[2, 0, 0, 5, 0, 0],[0, 3, 4, 0, 7, 11]])
    csys.controller.set(K)
else:
    Q = np.diag(np.array([10, 10, 5, 2, 2, 2]))
    csys.controller.setByCARE(A, B, Q)
    csys.controller.set(np.round(csys.controller.K, 5))


tTracker = csys.controller.structBuilder(nlDyn, csys)

cSim = hoverplane(nlDyn, tMaker, tTracker)

desTraj = structure()

def zeroU(t):
    if(np.isscalar(t)):
        length = 1
    else:
        length = len(t)

    return np.zeros((2,length))

desTraj.u = zeroU

pathType = "arc"
if(pathType == "lineX"):
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return nuEq* np.vstack((t, np.zeros((2,length)), np.ones((1,length)), np.zeros((2,length))))
    xi = np.array([0,5,0,nuEq,0,0]).reshape((6,1))
    desTraj.x = line
    desTraj.tspan = [0,10]
    desTraj.stateDep = False
elif(pathType == 'lineTheta'):
    theta = -0.2
    v = 0.5*nuEq
    def line(t):
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        xc = np.cos(theta)
        yc = np.sin(theta)
        out = np.vstack((v*xc*t, v*yc*t, theta*np.ones((1,length)), v*xc*np.ones((1,length)), v*yc*np.ones((1,length)), np.zeros((1,length))))
        return out
    xi= np.array([0, 0, 0, nuEq, 0, 0]).reshape((6,1))
    desTraj.x = line
    desTraj.tspan = [0,10]
    desTraj.stateDep = False

elif(pathType == 'arc'):
    def arc(t):
        rad = 10
        if(np.isscalar(t)):
            length = 1
        else:
            length = len(t)

        return np.vstack((nuEq*rad*np.sin(t/rad), nuEq*rad*(1-np.cos(t/rad)), t/rad, (nuEq)*np.cos(t/rad), nuEq*np.sin(t/rad), np.ones((1,length))/rad))

    xi = np.array([0,0,0,0.7*nuEq,0,0]).reshape((6,1))
    desTraj.x = arc
    desTraj.tspan = [0,50]
    desTraj.stateDep = False

initState = structure(x=xi)
cSim.setInitialState(initState)
sol = cSim.track(desTraj)

cSim.plotSignals(1, desTraj)