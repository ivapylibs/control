from ivacontrol.niODERK4 import niODERK4
from ivacontrol.util import Timer
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import numpy as np

x0 = 5
tspan = [0 , 5]
dt = 0.125

def scalarSys(t, x, varargin=None):
    return -0.25*x

print('Matlab built-in.')
#tic
#[to,xo] = ode45(@scalarSys, tspan, x0);
#toc

with Timer():
    to = np.arange(start=tspan[0], stop=tspan[1], step=dt)
    to = np.append(to,tspan[1])
    xo = odeint(func=scalarSys, y0=x0, t=to, tfirst=True)

print('numIntegrator versions:')

with Timer():
    scheme = niODERK4(scalarSys, dt)
    [ti,xi] = scheme.integrate(tspan, x0)


with Timer():
    [ti2, xi2] = niODERK4.solve(scalarSys, tspan, x0, dt)

with Timer():
    odeLike = niODERK4.solvePtr(dt)
    [ti3, xi3] = odeLike(scalarSys, tspan, x0)

xo=xo.flatten()
to=to.flatten()
ti=ti.flatten()
xi=xi.flatten()

plt.figure(1)
plt.plot(to, xo, 'g-.', ti, xi, 'b-')
#plt.legend('ode45','numInt')

plt.figure(2)
plt.plot(np.array([range(len(to))]).flatten(), to, 'g-.', np.array([range(len(ti))]).flatten(), ti, 'b-')
plt.show()
print('Mean timesteps (ode vs custom):');
#print([mean(diff(to)) , mean(diff(ti))]);
