import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_simulator(model,curTime):
    """
    --------------------------------------------------------------------------
    template_simulator: tuning parameters
    --------------------------------------------------------------------------
    """
    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
        'integration_tool': 'cvodes',
        'abstol': 1e-10,
        'reltol': 1e-10,
        't_step': .01
    }

    simulator.set_param(**params_simulator)
    # Get the template
    tvp_template = simulator.get_tvp_template()

    # Define the function (indexing is much simpler ...)
    def tvp_fun(t_now):
        t_now = t_now + curTime
        tvp_template['mytime'] = t_now
        tvp_template['mytime2'] = t_now
        return tvp_template

    # Set the tvp_fun:
    simulator.set_tvp_fun(tvp_fun)
    simulator.setup()

    return simulator
