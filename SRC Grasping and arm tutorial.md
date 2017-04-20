This tutorial aims to explain how to control the R5’s arm and grasping.  This tutorial assumes you have already setup your system for SRC, and can successfully run the SRC qualifiers and/or the main competition.  If this isn’t true, please see the previous [tutorials](https://bitbucket.org/osrf/srcsim/wiki/tutorials) on how to do this.

This tutorial is broken into two parts.  In the first part, we’ll be going over how to use the keyboard_teleop SRC node to control the arm and hand of the R5.  In the second part, we’ll be going over how to publish your own transforms to send to the robot.

# Part 1 - Controlling the robot with the keyboard_teleop node

The SRC ships with a ROS node that allows you to teleop the R5 from the keyboard.  This is called the keyboard_teleop node.  Let’s start up the SRC simulation, and start up the keyboard_teleop node:

In one terminal, fire up the simulation; note that this tutorial uses qual2.launch, but you can use any other launch file that you choose. (IMPORTANT NOTE: the launch file *must* have the grasping arg set to true!)

    $ roslaunch srcsim qual2.launch init:=true

The simulation will launch and will slowly lower the R5 robot to the ground.  Once the harness has released, the robot is free to move.  The arm and hand position should look something like the picture below.