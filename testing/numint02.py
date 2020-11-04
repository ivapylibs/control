from niODERK4 import niODERK4
from util import Timer
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import numpy as np

x0 = np.array([5,-2])
tspan = [0 , 5]
dt = 0.125

def swirlSys(t, x, varargin=None):
    return np.matmul(np.array([[-0.25, 1], [-1, -0.25]]), x)


with Timer('scipy built-in'):
    to = np.arange(start=tspan[0], stop=tspan[1], step=dt)
    to = np.append(to,tspan[1])
    xo = odeint(func=swirlSys, y0=x0, t=to, tfirst=True)

with Timer('numIntegrator'):
    scheme = niODERK4(swirlSys, dt)
    [ti,xi] = scheme.integrate(tspan, x0.reshape((2,1)))

xo = np.transpose(xo)

plt.figure(1)
plt.plot(to, xo.transpose(), 'g-.', ti, xi.transpose(), 'b-')
#plt.legend('ode45','ode45','numInt','numInt');


plt.figure(2)
plt.plot(np.array([range(len(to))]).flatten(), to, 'g-.', np.array([range(len(ti))]).flatten(), ti, 'b-')
plt.show()
print('Mean timesteps (ode vs custom):');
#print([mean(diff(to)) , mean(diff(ti))]);
