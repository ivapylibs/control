import numpy as np
from casadi import *
from casadi.tools import *
from matplotlib import pyplot as plt
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_mpc(model,fptr,curTime):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 5,
        'n_robust': 0,
        'open_loop': 0,
        't_step': .01,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 2,
        'collocation_ni': 2,
        'store_full_solution': True,
        # Use MA27 linear solver in ipopt for faster calculations:
        #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
    }
    n_horizon = 5;
    mpc.set_param(**setup_mpc)

    mterm = (model.x['X_s']-model.x['X_s'])# terminal cost
    lterm = (model.x['X_s']-model.tvp['xDes'])**2 + (model.x['Y_s']-model.tvp['yDes'])**2 # Legrangian
    # stage cost

    mpc.set_objective(mterm=mterm, lterm=lterm)

    mpc.set_rterm(inp1=1e-4,inp2 = 1e-4) # input penalty


    #mpc.bounds['lower', '_x', 'V_s'] = 0.0
    #mpc.bounds['upper','_x',   'V_s'] = 4.0

    #a = .5
    #mpc.bounds['lower','_u','inp1'] = -a
    #mpc.bounds['lower','_u','inp2'] = -a
    #mpc.bounds['upper','_u','inp1'] = a
    #mpc.bounds['upper','_u','inp2'] = a

    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_now):
        t_now = t_now + curTime
        #print(curTime)
        #print(t_now)
        #input()
        myarray = fptr(t_now)
        if(t_now > 0):
        #print(t_now)
        #print(myarray)
        #input()
        #plt.figure(1)
        #print(myarray[0,0])
        #print(myarray[1,0])
            plt.scatter(myarray[0,0],myarray[1,0])


        for k in range(n_horizon+1):
            tvp_template['_tvp',k,'xDes'] = myarray[0]
            tvp_template['_tvp',k,'yDes'] = myarray[1]
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc
