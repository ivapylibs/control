import numpy as np
import math

def HoverPlaneDynamics(t,x,u):
    sdA = np.diag([1,1,1],k=3)
    sdB = np.zeros(6,2)
    sdB[2,0] = math.cos(x[2])
    sdB[3,0] = math.sin(x[2])
    sdB[5,1] = 1


    return np.matmul(sdA,x) + np.matmul(sdB,u)
