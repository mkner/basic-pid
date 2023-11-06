

**Basic PID** is a classic PID controller that is easy to use, works and does the job.

The PID controller is designed to be used in discretized time regulators and
implements timestep integration that supports 2 modes of operation: **Integrative** and **Iterative**

In **Integrative Mode**, the timestep integrations are calculated inside the controller
and the output from the PID controller for the current timestep is used directly or with 
modifications and sent to the device or process plant as the current input signal without
further iterative integration.

In **Iterative Mode**, the PID timestep integrations are manually calculated and updated
outside the controller in the algorithm that calls the PID controller for the output of
the PID at the current timestep.

**Basic PID** has proven to be a tested, and reliable PID controller. It has been used, for example, 
with mobile robotic systems for LVC (Linear Velocity Control) for regulating wheel velocities, 
LVDR (Lateral Velocity Differential Regulator) for keeping the wheel velocities in sync for 
differential drive mobile robots when traversing a straight line path and 
ADVR (Angular Differential Velocity Regulator) for tracking a heading angle using typical
motion control input signals (v,w) for linear and angular velocities.


Installation: 

$ **pip** install basic-pid

For documentation see https://basic-pid.readthedocs.io/en/latest/





