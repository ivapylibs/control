import numpy as np
from casadi import *
from casadi.tools import *
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
        'n_horizon': 20,
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
    n_horizon = 20;
    mpc.set_param(**setup_mpc)

    mterm = (model.x['X_s']-model.tvp['mytime'])**2 + (model.x['Y_s']-model.tvp['mytime2'])**2 # terminal cost
    lterm = (model.x['X_s']-model.tvp['mytime'])**2 + (model.x['Y_s']-model.tvp['mytime2'])**2 # Legrangian
    # stage cost

    mpc.set_objective(mterm=mterm, lterm=lterm)

    mpc.set_rterm(inp1=1e-4,inp2 = 1e-4) # input penalty


    #mpc.bounds['lower', '_x', 'V_s'] = 0.0
    #mpc.bounds['upper','_x',   'V_s'] = 4.0


    mpc.bounds['lower','_u','inp1'] = 0.0
    mpc.bounds['lower','_u','inp2'] = 0.0

    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_now):
        t_now = t_now + curTime
        for k in range(n_horizon+1):
            myarray = fptr(t_now)
            tvp_template['_tvp',k,'mytime'] = myarray[0]
            tvp_template['_tvp',k,'mytime2'] = myarray[1]
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc
