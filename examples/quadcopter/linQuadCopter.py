from ivacontrol.controlSystem import controlSystem
from ivacontrol.controller.linear import linear
import numpy as np
import matplotlib.pyplot as plt
from ivacontrol.util import Timer

class linQuadCopter(controlSystem):

    def __init__(self, ceom, trajGenerator, trackBuilder, config=None):
        super(linQuadCopter, self).__init__(ceom, trajGenerator, trackBuilder, config)

    def plotSignals(self, fn, desTraj):
        haveDes = desTraj is not None

        sigArgs = {'linewidth':2.5}
        desArgs = {'linewidth':4.5}

        def sigPlot(t, x):
            plt.plot(t, x, '-', **sigArgs)

        def desPlot(t, x):
            plt.plot(t, x, ':', **desArgs)

        #si = len(self.simOut)

        t = self.simOut[-1].t
        x = self.simOut[-1].x
        u = self.simOut[-1].u

        if (haveDes):
            with Timer(name="get desired"):
                xDes = np.array(desTraj.x(t))
                uDes = np.array(desTraj.u(t))

        fh = plt.figure(fn)
        #clf

        sigPlot(t, x[0:3,:].transpose())
        if haveDes:
            #hold on;
            desPlot(t, xDes[0:3,:].transpose())
            #hold off;
            pass
        #legend('$x$','$y$','$z$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Position');


        fn = fn + 1
        fh = plt.figure(fn)
        #clf;
        sigPlot(t, x[6:9,:].transpose())
        if (haveDes):
            #hold on;
            desPlot(t, xDes[6:9,:].transpose())
            #hold off;
            pass

        #legend('$\dot x$','$\dot y$','$\dot z$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Velocity');


        fn = fn + 1
        fh = plt.figure(fn)
        #clf;
        sigPlot(t, (180/np.pi)*x[3:6,:].transpose())
        if (haveDes):
            #hold on;
            desPlot(t, (180/np.pi)*xDes[3:6,:].transpose())
            #hold off;
            pass

        #legend('$R$','$P$','$Y$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Euler Angular States');

        fn = fn + 1
        fh = plt.figure(fn)
        #clf;
        sigPlot(t, (180/np.pi)*x[9:12,:].transpose())
        if (haveDes):
            #hold on;
            desPlot(t, (180/np.pi)*xDes[9:12,:].transpose())
            #hold off;
            pass

        #legend('$\dot R$','$\dot P$','$\dot Y$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Euler Angular Rates');


        fn = fn + 1
        fh = plt.figure(fn)
        '''
        #clf
        plot3(x(1,:), this.simOut(si).x(2,:), this.simOut(si).x(3,:), sigArgs{:});
        hold on;
        if (haveDes):
            plot3(xDes(1,:), xDes(2,:), xDes(3,:), desArgs{:});
            plot3(xDes(1,1), xDes(2,1), xDes(3,1), 'go');
            plot3(xDes(1,end), xDes(2,end), xDes(3,end), 'rs');
        else:
            plot3(this.simOut(si).x(1,1), this.simOut(si).x(2,1),  ...
                                        this.simOut(si).x(3,1), 'go');
            plot3(this.simOut(si).x(1,end), this.simOut(si).x(2,end),  ...
                                          this.simOut(si).x(3,end),'rs');

        hold off;
        xlabel('$x$','interpreter','latex');
        ylabel('$y$','interpreter','latex');
        ylabel('$z$','interpreter','latex');
        axis equal;
        set(fh, 'Name', 'Quad-Copter Parametric Plot');
        '''
        fn = fn + 1
        fh = plt.figure(fn)
        #clf;
        plt.plot(t,u.transpose())
        if (haveDes):
            #hold on;
            plt.plot(t, uDes.transpose())
            #hold off;

        #hold on;
        #%plot(this.simOut(si).t([1,end]), this.controller.ueq*[1 1],'-.');
        #%TODO: Figure out how to manage this part. Same as above.
        #hold off;
        #legend('$u_1$','$u_2$','$u_3$','$u_4$','interpreter','latex');
        #set(fh, 'Name', 'Quad-Copter Control Signal');

        plt.show()

    @staticmethod
    def dynamicsAtHover(qcParm):
        '''
        The state first has position coordinates, then angular coordinates,
        followed by their velocities in the same order. This is preferred
        format to align with decomposition of tangent space into base and
        vector spaces. The dynamics live in the second order jet.
        '''
        A = np.diag(np.ones((6)),6)   #Top right block is identity matrix.
        B = np.zeros((12,4))

        uEq = ((qcParm.m*qcParm.g)/(4*qcParm.KT))*np.ones((4,1))

        A[6,4] = qcParm.g   #Control/state coupling.
        A[7,3] = -qcParm.g

        if 'Dt' in qcParm:
            A[0:3, 6:9] = -qcParm.Dt * np.eye(3) / qcParm.m
        if 'Dr' in qcParm:
            A[3:6,9:12] = -qcParm.Dr * np.eye(3) / qcParm.m

        ThZ = 2 * qcParm.KT / qcParm.m
        ThR = 2 * qcParm.r * qcParm.KT / qcParm.Jx
        ThP = 2 * qcParm.r * qcParm.KT / qcParm.Jy
        ThYaw = 2 * qcParm.KD / qcParm.Jz

        B[8,:]  =  ThZ
        B[9, 2] = ThR
        B[9, 3] = -ThR
        B[10, 0] = -ThP
        B[10, 1] = ThP
        B[11, [0, 1]] = -ThYaw
        B[11, [2, 3]] = ThYaw

        dyn = linear.systemDynamics(A, B)

        return (dyn, A, B, uEq)