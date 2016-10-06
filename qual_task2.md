# Overview

SRC qualification 2 is a locomotion and basic manipulation task. The qualification loads with R5 standing in front of a doorway with a closed door and a red button. 

You must walk close to the red button, press it, and cross the doorway once the door opens. Score is based on your ability to press the button, walking through the doorway, where R5 must walk one meter beyond the door without falling, and the time required to complete the task.

There is a line in front of the initial position of R5. When you cross this starting line your task will start. At one meter after the doorway there is a second line. Crossing this finish line will finish the task.

# 2D image processing

Please, refer to the [qualification 1 documentation](https://bitbucket.org/osrf/srcsim/wiki/qual_task1) for a description of how to process your camera images.

# Walking

The DRCSim API (TBD) provides a set of ROS messages that can be used to control the walking of your robot. In this tutorial, we'll use a helper script to demonstrate how to start a multi-step behavior. Once your robot is ready standing in front of the doorway, execute the following command:


```
#!c++

rosrun ihmc_ros_diagnostics boxStep.py
```

The robot should start walking a few steps. 


# Arm control

# Upload your log file

