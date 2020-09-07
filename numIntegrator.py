#============================= numIntegrator =============================
#
## @class   numIntegrator
#
## @brief   Simple numerical integrator interface possibly with less
#           capabilities on the numerics, but more flexibility with
#           regards to control (adaptive, optimal, etc.).
#
#============================= numIntegrator =============================

#
## @file    numIntegrator.py
#
## @author  Patricio A. Vela,   pvela@gatech.edu
## @date    2020/09/07
#
## @note    Translated from Matlab version.
#
# NOTE ON FORMATTING:
#   set indent to 2 spaces.
#   set tab to 4 spaces (with conversion to spaces).
#
#============================= numIntegrator =============================
def numIntegrator():

  class intState(Enum):     # TODO: Is correct?
    NOTREADY    = 0;
    INITIALIZED = 1;
    INTEGRATING = 2;
    LASTSTEP    = 3;
    DONE        = 4;

  #=========================== numIntegrator ===========================
  #
  ## @brief     Constructor for the numerical integrator class
  #
  ## @param[in] theFunc     The function to integrate.
  ## @param[in] dt          The timestep to apply.
  ## @param[in] opts        Optional arguments informing integration method.
  #
  def __init__(this, theFunc, dt, opts):
    this.dynamics = theFunc;
    this.dt       = dt;

    if (len(sys.argv) < 3):
      this.opts = [];
    else:
      this.opts = opts;
    end

    this.state = intState.NOTREADY;    # TODO: Seems wrong.

    this.x  = [];
    this.t  = [];
    this.x0 = [];
    this.xc = [];
    this.tc = [];
    this.ci = [];

    this.extra = false;

  #============================= initialize ============================
  #
  ## @brief     Prepare for a new numerical integration run.
  #
  ## @param[in] tspan       The time span as a range.
  ## @param[in] x0          The initial condition.
  #
  def initialize(this, tspan, x0):


  #=============================== reset ===============================
  #
  def reset(this):


  #============================== preStep ==============================
  #
  def preStep(this):


  #============================== runStep ==============================
  #
  def runStep(this):


  #============================== postStep =============================
  #
  def postStep(this):

  #============================== advance ==============================
  #
  def advance(this, varargin):      # TODO: python equivalent?

    if (this.state == intState.INITIALIZED):
      this.state = intState.INTEGRATING;    # Upgrade to integrating.

    if (this.state == intState.INTEGRATING):
      # Integration step here.

    elif (this.state == intState.LASTSTEP)  # Integrat and set to done.
      this.preStep();

      # other steps here ...

      this.postStep();

      this.ci = this.ci + 1;
      this.tc = this.t(end);

      this.state = intState.DONE;


  #============================= integrate =============================
  #
  ## @brief Run the numerical integration scheme. 
  #
  # This is the short and long form version of numerical integration.  If the
  # system has been initialized, then tspan and x0 are optional arguments.  The
  # single invocation version would call with all arguments.  The
  # multi-invocation version would first initialize then integrate as separate
  # calls.
  #
  ## @param[in] tspan       The time span as a range.
  ## @param[in] x0          The initial condition.
  ## @param[in] varargin    Additional (optional) arguments sent to function.
  #
  def integrate(this, tspan, x0, varargin)  # TODO: python equivalent of varargin?
    # Translate code here.


#
#============================= numIntegrator =============================
