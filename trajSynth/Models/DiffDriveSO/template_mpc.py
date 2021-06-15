#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

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

    mterm = (model.x['Y_s']-model.tvp['yDes'])**2 + (model.x['X_s']-model.tvp['xDes'])**2 # terminal cost
    lterm = (model.x['Y_s']-model.tvp['yDes'])**6 + (model.x['X_s']-model.tvp['xDes'])**6 + (model.x['T_s'] - model.tvp['TDes'])**8 + (model.x['V_x'] - model.tvp['VxDes'])**2 + (model.x['V_y'] - model.tvp['VyDes'])**2 + (model.x['W'] - model.tvp['WDes'])**2# Legrangian
    # stage cost

    mpc.set_objective(mterm=mterm, lterm=lterm)

    mpc.set_rterm(inp1=1e-4,inp2 = 1e-4) # input penalty


    #mpc.bounds['lower', '_x', 'V_s'] = 0.0
    #mpc.bounds['upper','_x',   'V_s'] = 4.0



    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_now):
        t_now = t_now + curTime
        myarray = fptr(t_now)
        #print(myarray[0])
        #print(myarray[1])
        #input("Enter")
        for k in range(n_horizon+1):

            tvp_template['_tvp',k,'xDes'] = myarray[0]
            tvp_template['_tvp',k,'yDes'] = myarray[1]
            tvp_template['_tvp',k,'TDes'] = myarray[2]
            tvp_template['_tvp',k,'VxDes'] = myarray[3]
            tvp_template['_tvp',k,'VyDes'] = myarray[4]
            tvp_template['_tvp',k,'WDes'] = myarray[5]
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc
