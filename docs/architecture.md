# Architecture

This page describes the way that our code acts at a very high level.

> The architecture of a software system defines that system in terms of computational components and interactions among those components.

Our architecture has a few main goals:

* Make automated testing of code easy
  * Anything that includes WPILib functions cannot be compiled on 64bit computers, so we want to avoid calling WPILib functions in most of our code.
* Have clearly defined ownership
  * We want it to be clear what parts of our code have access to what other parts.
* Allow all controllers to be realtime, while being able to communicate with non-realtime threads
  * We want all of the controllers in our code to run in a bounded amount of time. However, there are some things that cannot (and don't need to) complete in a bounded amount of time, such as reading things from the network, or getting joystick values. Because of this, we need a good way to pass things between threads, so that the realtime threads can access values that they need from the non-realtime threads

## Subsystems

We split all of the mechanical components of our robot into subsystems. We usually have a drivetrain subsystem and superstructure subsystem, since most of the components on the superstructure are logically linked. Each subsystem owns all of the mechanisms associated with it.

For example, here are the subsystems and mechanisms for c2017:

* Drivetrain
* Superstructure
  * ground ball intake
  * ground gear intake
  * shooter
  * magazine
  * climber

The superstructure is responsible for reading the goals for all of the subsystems, fixing any incompatible goals (for example, we don't want to try to be shooting and climbing at the same time, since they're run off the same motors), and passing through inputs and outputs. Usually, we will have an `Update()` function for each subsystem, which will read all of the goals and inputs, call `Update()` on each of the mechanisms, and write their output to a queue.

Each mechanism has 4 [protobufs](protobuf_notes.md) associated with it - a goal, input, output, and status:

* Goal
  * The goal is what we want the mechanism to be doing. For example, on a shooter, we might have the RPM of the shooter wheel and a boolean for if we want to be firing right now in the goal.
* Input
  * The input is the sensor that the mechanism requires. This includes encoders, proximity sensors, hall effect sensors, and current draw readings.
* Output
  * The output is a voltage or boolean for a solenoid, usually.
* Status
  * The status is anything else that we might care about or want to log.

## Threads

We have quite a few different threads running:

* Subsystem runner
  * This is the main realtime thread, which runs all of the logic and controllers.
* Citrus robot
  * This receives control packets (which contain joystick data, among other things). It runs at a very slow and inconsistent rate, so we don't do anything that needs to be realtime in it.
* Logging
  * This reads from every queue and writes the output to a CSV file on the robot.
* CAN Wrapper
  * This deals with outputting CAN messages, since writing to CAN is not realtime.
