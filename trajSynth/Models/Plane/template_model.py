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
    g = 9.8
    K = 1
    m = 1
    # States struct (optimization variables):
    X_s = model.set_variable('_x',  'X_s')  # X Pos
    Y_s = model.set_variable('_x',  'Y_s')  # Y Pos
    Z_s = model.set_variable('_x',  'Z_s')  # Z Pos
    Psi_s = model.set_variable('_u',  'Psi_s')  # Theta
    V_s = model.set_variable('_x',  'V_s') #Veloctiy

    #V_s = model.set_variable('_x',  'V_s')  # Vel
    # Input struct (optimization variables):
    thrust = model.set_variable('_u',  'thrust') # Accl
    gam = model.set_variable('_u',  'gam') # Omega
    phi = model.set_variable('_u',  'phi') #phi


    # Differential equations
    model.set_rhs('X_s', V_s*np.cos(Psi_s)*np.cos(gam))
    model.set_rhs('Y_s', V_s*np.sin(Psi_s)*np.cos(gam))
    model.set_rhs('Z_s', V_s*np.sin(gam))
    model.set_rhs('V_s', -K/m*V_s**2 - g*np.sin(gam) + thrust/m)
    #model.set_rhs('Psi_s', g/V_s*np.tan(phi))
    #model.set_rhs('V_s', inp1)

    # Build the model
    model.setup()

    return model
