

Examples
--------

Using the module
****************

For example:

>>> import basicpid

>>> pid = basicpid.BasicPid()

Or just:

>>> from basicpid import Pid

>>> pid = Pid()

>>> pid.name()
'BasicPid'

>>> pid.vers()
'v0.02.07'

>>> pid.whoami()
BasicPid v0.02.07

>>> pid.desc()
'Easy to use classic PID controller'


Mode Examples
*************

The following examples require **robotime**
to be installed for the timing and delay functions.

.. code-block:: console

    $ pip install robo-time


Getting around...

.. code-block:: python

  from robotime.time import delay
  from basicpid import BasicPid
  
  pid = BasicPid()
  
  pid.whoami()
  BasicPid v0.02.06

  #check default settings
  pid.getGains()
  (1.0, 0, 0)

  pid.setGains(1,0.025,0.0001)

  # check setttings
  pid.getGains()
  (1, 0.025, 0.0001)

  # use integrate mode
  pid.setIntegrateModeOn()

  # check mode flags
  pid.inIntegrateMode()
  True

  pid.inIterateMode()
  False

  # clears previous results, keeps gains intact
  pid.reset()

  #check
  pid.getGains()
  (1, 0.025, 0.0001)

The PID controller automatically calculates the error for the current timestep
that is the difference between the reference signal and the actual process output.
Expressed using standard notation, that is **e(t) = r - y(t)** where **e(t)** is
the error for the  current timestep, **t** is the current timestep, **r** is the
reference signal to track, and **y(t)** is the output at the current timestep
from the process plant.

In **Integrate Mode** the PID controller updates its current
output using its state from the previous timestep. 

At each timestep, the three term PID equation is recalculated. 

The proportional term is updated with the current error. The integral 
term of the PID equation is reintegrated. And the derivative term is
recalculated using the current error and the difference between
consecutive timesteps.

In this example, the process output is a constant fixed value that
is not and never can be the reference. Shows the effect of timestep
integrations being calculated internally with the PID controller in **Integrative Mode**. 
Expect to see additive integrations gradually increase the PID output without bound
since the imaginary process does not react and a constant, instead of the real output 
from a process, is fed back into the controller with each timestep. So its
output can never stabilize to the reference, and in this example surpasses it.


.. code-block:: python

  ref_sig = 1.5 # tracking reference signal
  output_sig = 0.5 # output signal or measurement value from the process or device
  
  for i in range(25): 
    delay(500)
    print(round( pid.get(ref_sig, output_sig) ,10))

    1.0126
    1.0375
    1.0625
    1.0875
    1.1125
    1.1375
    1.1625
    1.1875
    1.2125
    1.2375
    1.2625
    1.2875
    1.3125
    1.3375
    1.3625
    1.3875
    1.4125
    1.4375
    1.4625
    1.4875
    1.5125
    1.5375
    1.5625
    1.5875
    1.6125


In this example check that the PID controller is detecting a
stable state at reference properly when in **Integrative Mode**. 
Since the output from the process is the same as the reference signal 
sent to the PID controller, it will output nothing but zeros and there would 
be no change to the input control signal being sent to the process from whatever 
value it had stablized at.


.. code-block:: python

  pid.reset()

  pid.getGains()
  (1, 0.025, 0.0001)

.. code-block:: python

  ref_sig = 1 # tracking reference signal
  output_sig = 1 # output signal or measurement value from the process or device

  for i in range(5): 
    delay(500)
    print(round( pid.get(ref_sig, output_sig) ,10))

  0.0
  0.0
  0.0
  0.0
  0.0


Repeating the same example above, but now use **Iterative Mode**

First, set the controller in **Iterate** mode

.. code-block:: python

  # use iterative mode
  pid.setIterateModeOn()

  # check mode flags

  # not this mode
  pid.inIntegrateMode() # not this mode
  False

  # check really using iterate mode
  pid.inIterateMode()
  True




Wheel-Motor Velocity Controller
*******************************

.. code-block:: python

  # example of wheel/motor velocity PID control
  # using BasicPid in timestep iterative mode
  # assume that IoScan is a class that has background process
  # input signal processing & buffering capability
  # and a component object of WheelVelocity is clock 
  # that can return the uptime of the clock since
  # instantiation of the WheelVelocity object 
  # in milliseconds with the call clock.millis()
  #
  # (c) 2023, 2022 - Mike Knerr
  #

  from robotime.clocks import Clock
  from basicpid import BasicPid
  

  class WheelVelocity(IoScan):

    def __init__(self, wheel):
        super(WheelVelocity, self).__init__()

        self._name = "WheelVelocity"
        self._desc = "WheelVelocity"
        self._vers = "v0.01.02"  # 0.09 w/ velocity

        self._wheel = wheel #contains motor 
        self.clock = Clock()

        self.pid = BasicPid() # on ext interface
        
        self._v_ref = 0 # signal reference velocity
        self._v = 0 # current instantaneous velocity
        self._v_avg = 0
        
        self._pid_out = 0
        self._pid_out_prev = 0
        
        self._rate = 0
        self._rate_prev = 0
        self._rate_pid = 0
    
        self._vmax = 0.50 # of wheels/motors
        
        self._default_scanfreq = 50
        self._default_bufsize = 5
        # clock from IoScan
        # used in interation process thread
        self._dur_start_time = self.clock.millis()
        self._dur = None
     
        #init
        #self.deActivate()
        self.stopScanning()
        self.setScanFreq(self._default_scanfreq)
        self.setBufferingOff()
        self.setBufSize(self._default_bufsize)
        self.setBufferingOn()
        #important
        self.pid.setIterateModeOn()
        self.startScanning()
        

     # this function would be called every self.getTimeinc() timesteps
     # by a process thread that is running in the WheelVelocity object
     # handled by class IoScan that WheelVelocity is decendant from

    def _velocity_handler(self):
        
        # else process signal
      
        #ok, use ONLY this call from WheelVelocity object
        self._v =  self._wheel._velocity._getVelocityGo()
      
        if self.isBuffering():
              if len(self._buf) > 0 \
                  and self._v != None: #be robust
                self._buf.pop(0)
                self._buf.append(self._v)
              ## ok
              self._v_avg  = self.getBufAvg()
        else:
            # really want to use  buffered velocity, 
            self._v_avg = self._v
            
        #set timestep always, it can change dynamically
        time_inc_sec = self.getTimeinc()/1000
        self.pid.setTimeinc(time_inc_sec)
        
        if self._v_ref > 0:
            self._pid_out = self.pid.getPid(self._v_ref, self._v_avg) #,time
        
        if self._v_ref < 0:
            self._pid_out = self.pid.getPid(abs(self._v_ref), abs(self._v_avg)) #,time
        
        # similar to technique used w/ stanley simulator
        # for throttle control signal
        # pid in iterative mode for timestep discretized version
        self._rate_pid = self._rate_prev + self._pid_out
        
        # rate is a speed, not a vector like velocity
        # so it is always constrained in [1,100]
        
        # if there is an active signal
        # zero is no active signal
        
        if self._v_ref > 0:
            self._rate = constrain(self._rate_pid,0,100)
            # or in [1,100]
            #self._rate = constrain(self._rate_pid,1,100)
           
            if self._rate >0:
             self._wheel.forward(self._rate)
         
         # if there is an active signal
        if self._v_ref < 0:
            # or in [1,100]
            #self._rate = constrain(self._rate_pid,1,100)
            #use abs of pid out for v_reg < 0?
            self._rate = constrain(self._rate_pid,0,100)
            
            if self._rate >0:
             self._wheel.reverse(self._rate)
             
        self._rate_prev = self._rate 
            
        if self._dur != None:
          if (self.clock.millis() - self._dur_start_time) > self._dur:
              self._wheel.stop()
              self._dur = None
        return
    







