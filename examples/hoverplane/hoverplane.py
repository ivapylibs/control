from controlSystem import controlSystem
import matplotlib.pyplot as plt
import numpy as np
import Lie.group.SE2.Homog
import pdb

class hoverplane(controlSystem):
    def __init__(self, ceom, trajGenerator, trackBuilder):
        super().__init__(ceom, trajGenerator, trackBuilder)

    def plotSignals(self, fn, desTraj):
        haveDes = desTraj is not None

        sigArgs = {'linewidth':2.5}
        desArgs = {'linewidth':4.5}

        def sigPlot(t, x):
            plt.plot(t, x, '-', **sigArgs)

        def desPlot(t, x):
            plt.plot(t, x, ':', **desArgs)

        t = self.simOut[-1].t
        x = self.simOut[-1].x
        u = self.simOut[-1].u

        if (haveDes):
            xDes = np.array(desTraj.x(t))
            uDes = np.array(desTraj.u(t))

        fh = plt.figure(fn)

        sigPlot(t, x[0:2,:].transpose())
        plt.title("Position")
        plt.legend(['x', 'y'])
        if haveDes:
            desPlot(t, xDes[0:2,:].transpose())
            plt.legend(['xDes', 'yDes'])
            pass

        fn = fn + 1
        fh = plt.figure(fn)
        sigPlot(t, x[3:5,:].transpose())
        plt.title("Velocity")
        plt.legend(['$\dot{x}$', '$\dot{y}$'])
        if (haveDes):
            #hold on;
            desPlot(t, xDes[3:5,:].transpose())
            plt.legend(['$\dot{x}_{des}$', '$\dot{y}_{des}$'])
            #hold off;
            pass

        #legend('$\dot x$','$\dot y$','$\dot z$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Velocity');


        fn = fn + 1
        fh = plt.figure(fn)
        #clf;
        sigPlot(t, (180/np.pi)*x[2,:].transpose())
        sigPlot(t, (180/np.pi)*x[5,:].transpose())
        plt.legend(['$\theta$', '$\dot{\theta}$'])
        plt.title("Angular States")
        if (haveDes):
            #hold on;
            desPlot(t, (180/np.pi)*xDes[2,:].transpose())
            desPlot(t, (180/np.pi)*xDes[5,:].transpose())
            plt.legend(['$\theta$_{des}', '$\dot{\theta}_{des}$'])
            #hold off;
            pass

        fn = fn + 1
        fh = plt.figure(fn)
        sigPlot(x[0,:], x[1,:])
        if (haveDes):
            desPlot(xDes[0,:], xDes[1,:])
        
        plt.show(block=False)
        plt.pause(0.001) # Pause for interval seconds.
        input("hit[enter] to end.")
        plt.close('all') # all open plots are correctly closed after each run

    @staticmethod
    def dynamicsForward(Parms, nuEq):
        def hoverplaneNonLinear(t, x, u):
            sdA = np.diag(np.ones((3,)), k=3)
            R = Lie.group.SE2.Homog.rotationMatrix(x[2])

            Delta = np.array([[Parms.linD[0], 0],[0, -Parms.linD[1]]])
            sdA[3:5, 3:5] = np.matmul(np.matmul(R, Delta), R.T)

            sdA[5,5] = -Parms.angD

            rb = np.hstack((R[:,0, np.newaxis], np.array([[0], [0]])))
            sdB = np.vstack((np.zeros((3,2)), rb, np.array([0, 1])))

            xi = np.vstack((np.matmul(R.T, x[3:5]), x[5]))
            LagF = Parms.lagF * np.vstack((np.zeros((3,1)),  xi[0]*xi[2]*R[:,1, np.newaxis],  0))

            wvF = np.vstack((np.zeros((3,1)), -Parms.Md * R[:,1, np.newaxis], Parms.Ma)) * np.dot(R[:, 1], x[3:5])
            #pdb.set_trace()
            return np.matmul(sdA, x) + np.matmul(sdB, u) + LagF + wvF

        A = np.diag(np.ones((3,)), k=3)
        A[3:5, 3:5] = -np.diag(Parms.linD)
        A[5,5] = -Parms.angD

        A[4,2] = (Parms.linD[1] + Parms.Md + Parms.lagF) * nuEq

        A[5, 2] = - Parms.Ma * nuEq
        A[5, 5] = Parms.Ma
            
        B = np.zeros((6,2))
        B[3,0] = 1
        B[5,1] = 1

        uEq = np.array([Parms.linD[0] * nuEq, 0]).reshape((2,1))
        dyn = hoverplaneNonLinear
        
        return (dyn, A, B, uEq)
