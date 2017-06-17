# Debugging Using `gdbserver`

When debugging a segfault or other issue causing your code to crash, using a debugger can be *very* useful. However, it can be a bit difficult to run a debugger on the robot code. This doc will show you how to debug robot code running on the robot using gdbserver.

To do this, we will be running the robot code binary on the RoboRIO, but running the debugger on your computer.

First, compile the robot code with debug flags on. This gives the debugger information that it needs in order to give you useful information!

```
bazel run --cpu=roborio //cYEAR:deploy -c dbg --strip=never -- --team TEAM
```

Where `YEAR` is the year of the code that you're running, and `TEAM` is the team number of the robot that you're deploying to.

Next, SSH into the RoboRIO, and run the following commands:
```
kill $(pidof autostart)
gdbserver localhost:2345 path/to/robot/code/binary/frc1678
```

Where `path/to/robot/code/binary/frc1678` is the actual path to the binary. Usually, you will need to cd into the `/home/lvuser/robot-code/` directory and then run `cYEAR/frc1678`, where `YEAR` is the year of the code that you're running.

This starts the robot code, and waits for your debugger to connect to it. To connect the debugger, run the following command locally on your computer:

```
arm-frc-linux-gnueabi-gdb path/to/robot/code/binary/frc1678
```

Again, `path/to/robot/code/binary/frc1678` should be the real path to the binary on your computer. If you're in the `robot-code` directory, this should be something like `bazel-bin/cYEAR/frc1678`.

Once you've done that, you should be at a prompt that says `(gbd) `. Type the following to connect to the robot:

```
target remote roborio-TEAM-frc.local:2345
set sysroot remote:/
```

Once you've done that, you can type `continue` to run the robot code. If it crashes, you can type `bt` to get a printout of where it crashed. You can also examine the contents of variables, set breakpoints, and do many other great things that are beyond the scope of this tutorial (but easily findable by searching "gdb tutorial" on google).

Do note that if you set a breakpoint, motors can continue to run when the control loop pauses, so make sure to unplug any motors that you don't want to spin out of control indefinitly before you enable the robot with a breakpoint on.

Good luck, and go squash some bugs!
