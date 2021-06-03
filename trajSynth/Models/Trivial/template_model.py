import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_model():
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Certain parameters

    # States struct (optimization variables):
    X_s = model.set_variable('_x',  'X_s')  # X Pos
    Y_s = model.set_variable('_x',  'Y_s')  # Y Pos
    #V_s = model.set_variable('_x',  'V_s')  # Vel
    # Input struct (optimization variables):
    inp1 = model.set_variable('_u',  'inp1') # Xdot
    inp2 = model.set_variable('_u',  'inp2') # Ydot

    mytime = model.set_variable(var_type = '_tvp', var_name = 'xDes') #time parameters
    mytime2 = model.set_variable(var_type = '_tvp', var_name = 'yDes')
    # Differential equations
    model.set_rhs('X_s', inp1)
    model.set_rhs('Y_s', inp2)
    #model.set_rhs('V_s', inp1)

    # Build the model
    model.setup()

    return model
