# Controls

This document will walk you through the process of the workflow of creating a controller for a simple robotic system.
For this document, we will use the simple example of a flywheel.

## Overview
First we will model the system and design for the desired controller characteristics in python. Then, we will set up a rule for the python code to write the system constants into C++, so that they can be used from the robot code. Finally, we will write the C++ code to actually use these system constants.

This document is for those who already understand state-space control system design and modelling and wish to create state-space controllers using muan. If you do not yet understand state-space control, talk to a veteran member or a mentor who can explain it to you.

## Modelling the system
The first step in creating a state-space controller is to create a good system model. The modelling code is written in python, so that's what we'll use for this part. Create a python rule in bazel:
```bazel
py_binary(
  name = 'flywheel_python',
  srcs = ['flywheel.py'],
  main = 'flywheel.py',
  deps = ['@muan//control:python_controls']
)
```
Then, open up `flywheel_demo.py`. The dependencies for the state-space code should be imported as follows:
```python
import numpy as np
import muan.control.state_space_controller as ssc
import muan.control.state_space_plant as ssp
import muan.control.state_space_observer as sso
import muan.control.state_space_scenario as sss
```
Then, define the system's continuous-time matrices with numpy. The constants in these matrices may be calculated or gathered empirically. In addition, define the system's `dt`, or timestep:
```python
A = np.asmatrix([[0, 1], [0, -.756]])
B = np.asmatrix([[0], [19.4]])
C = np.asmatrix([1, 0])
Q_noise = np.asmatrix([[1, 0], [0, 1]])
R_noise = np.asmatrix([[0.3]])
dt = .005
u_min = np.asmatrix([[-12.0]])
u_max = np.asmatrix([[12.0]])
x0 = np.asmatrix([[0], [0]])
```
Next, create the state space plant, controller, and observer. This is where the design comes in - you must choose between pole-placement and LQR for the controller, and pole-placement and Kalman gains for the observer.
```python
plant = ssp.state_space_plant(dt, x0, A, B, C, Q = Q_noise, R = R_noise)

# LQR Controller
Q = np.asmatrix([[.1, 0], [0, 10]])
R = np.asmatrix([[1]])
controller_lqr = ssc.lqr(plant, Q, R, u_min, u_max)

# Pole-placement controller
poles = [.98, .975]
controller_pp = ssc.placement(plant, poles, u_min, u_max)

# Kalman observer
observer_kalman = sso.kalman(plant)

# Pole-placement observer
poles = [.5, .3]
observer_pp = sso.placement(plant, poles)
```
The arguments to pole-placement and LQR are not an exact science - you should mess around with them until you get a good response out of the system. But how do you know how the system will respond? You must create a `state_space_scenario` object, which manages state-space simulations:
```python
xhat0 = x0
scenario = sss.state_space_scenario(plant, x0, controller, observer, xhat0, "flywheel")
```
In this scenario, `x0` is the initial actual position of the system while `xhat0` is the initial estimated position of the system. `"flywheel"` is the name of the system, which will later be used in writing files. We can run the scenario by defining a goal as a function of time and calling `run`:
```python
def goal(t):
  goal_vel = 100
  return np.asmatrix([goal_vel*t, goal_vel])
time_to_run = 10
scenario.run(goal, time_to_run)
```
Now we can run the program with `bazel run <target>`. It should display some graphs of the system's state and control signal over time. Adjust the parameters for the controller and the observer until the system's response is acceptable.

Finally, we are ready to write the matrices to C++ code. First, set up a new bazel rule (in the same build file):
```bazel
genrule(
  name = 'flywheel_genrule',
  srcs = [],
  tools = [':flywheel_python'],
  cmd = '$(location :flywheel_python)',
  outs = ['flywheel_constants.h', 'flywheel_constants.cpp']
)
```
This rule will run the `flywheel_python` rule to generate two files, `flywheel_constants.h` and `flywheel_constants.cpp`. However, currently `flywheel_python` is not set up to output to a file! To fix this, we just need to call `scenario.write`:
```python
...
scenario.write('flywheel_constants.h', 'flywheel_constants.cpp')
```
However, this doesn't quite work with bazel. Bazel expects the files to be written to long, complicated locations that change computer-to-computer. To fix this, change the `cmd` line in the `BUILD` file to read:
```bazel
  cmd = '$(location :flywheel_python) $(location flywheel_constants.h) $(location flywheel_constants.cpp)'
```
The program will now be run with the locations of the files to write as arguments. Now, in `flywheel.py`, we can write:
```python
# At the top of the file
import sys
...
if len(sys.argv) == 3:
  # The output files were specified
  scenario.write(sys.argv[1], sys.argv[2])
```
This, finally, will output the files to the correct locations. However, the program is still graphing when you try to build! Let's fix this:
```python
# Remove the line that reads "scenario.run(goal, time_to_run)"
if len(sys.argv) == 3:
  # The output files were specified
  scenario.write(sys.argv[1], sys.argv[2])
else:
  # No outputs were specified, so we can assume we just want to graph things
  scenario.run(goal, time_to_run)
```
