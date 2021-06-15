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
    T_s = model.set_variable('_x',  'T_s')  # Theta
    V_x = model.set_variable('_x',  'V_x')  # Velocity in X
    V_y = model.set_variable('_x',  'V_y')
    W = model.set_variable('_x',    'W')

    #V_s = model.set_variable('_x',  'V_s')  # Vel
    # Input struct (optimization variables):
    inp1 = model.set_variable('_u',  'inp1') # Accl
    inp2 = model.set_variable('_u',  'inp2') # Omega

    xDes = model.set_variable(var_type = '_tvp', var_name = 'xDes') #time parameters
    yDes = model.set_variable(var_type = '_tvp', var_name = 'yDes')
    TDes = model.set_variable(var_type ='_tvp', var_name = 'TDes')
    VxDes = model.set_variable(var_type = '_tvp', var_name ='VxDes')
    VyDes = model.set_variable(var_type = '_tvp', var_name ='VyDes')
    WDes = model.set_variable(var_type = '_tvp', var_name ='WDes')
    # Differential equations
    model.set_rhs('X_s', V_x)
    model.set_rhs('Y_s', V_y)
    model.set_rhs('T_s', W)
    model.set_rhs('V_x',np.cos(T_s)*inp1)
    model.set_rhs('V_y',np.sin(T_s)*inp2)
    model.set_rhs('W',inp2)
    #model.set_rhs('V_s', inp1)

    # Build the model
    model.setup()

    return model
