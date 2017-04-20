# Controlling the hand

## Starting up the simulation

The SRC ships with a ROS node that allows you to teleop the R5 from the keyboard.  This is called the keyboard_teleop node.  Let’s start up the SRC simulation, and start up the keyboard_teleop node:

In one terminal, fire up the simulation; note that this tutorial uses qual2.launch, but you can use any other launch file that you choose. (IMPORTANT NOTE: the launch file *must* have the grasping arg set to true!)

    $ roslaunch srcsim qual2.launch init:=true

The simulation will launch and will slowly lower the R5 robot to the ground.  Once the harness has released, the robot is free to move.  The arm and hand position should look something like the picture below.

![src-armtut-01-default-position.png](https://bitbucket.org/repo/xEbAAe/images/844246496-src-armtut-01-default-position.png)

At this point, we want to make sure that the grasping controller is active.  In another terminal, run:

    $ rostopic list | grep hand_position_controller

You should see the /left_hand_position_controller/command and the /right_hand_position_controller/command topics there.  If you do not, go back into your launch file and make sure that the grasping_init arg is set to true.

## Keyboard control

Now in another terminal launch the keyboard_teleop node:

    $ ./src/srcsim/scripts/keyboard_teleop.py # FIXME: this needs to get installed somewhere

When you first launch the keyboard_teleop.py node, you’ll get some information on how to control the robot using the keyboard, similar to what is below.

            Keyboard Teleop for Space Robotics Challenge 0.1.0
            Copyright (C) 2017 Open Source Robotics Foundation
            Released under the Apache 2 License
            --------------------------------------------------
            Left arm joints: q, W, e, R
                             a, S, D
                Reset: t or T

            Right arm joints: u, i, o, p
                              j, k, l
                Reset: y or Y

            Left hand joints: 5, $, #, @, !
                Reset: ` or ~

            Right hand joints: 6, 7, 8, 9, 0
                Reset: - or _

            Head angles:
                Roll: x
                Pitch: c
                Yaw: v
                Reset all head angles: g or G

            Neck joints: b, n, m
                Reset all neck joints: h or H

            The shown characters turn the joints in positive direction.
            The opposite case turns the joints in negative direction.
            For numbers the uppercase character is based on an English layout.

            Walking controls:
                Walk: f lowercase meaning forward and uppercase backwards
                Rotating on the spot: z lowercase is counterclockwise, uppercase is clockwise
            ?: Print this menu
            <TAB>: Print all current joint values
            <ESC>: Quit


As can be seen above, there are controls for the left and right arms, left and right hands, the head, the neck, and for walking.  This tutorial will focus just on the hands.

Let's put the arms of the robot in the "home" position by pressing 't' and then 'y'.  With that done, the robot should look like below.

![src-armtut-02-arm-home-position.png](https://bitbucket.org/repo/xEbAAe/images/56993020-src-armtut-02-arm-home-position.png)

Now, let's control one of the fingers on the R5.  Press the '8' key approximately 10 times, which should cause the index finger to curl up.  When you are done, the finger should look approximately like the following picture.

![src-armtut-05-right-finger.png](https://bitbucket.org/repo/xEbAAe/images/3417929898-src-armtut-05-right-finger.png)

Other keys on the keyboard do similar things.  For reference, the keys are mapped as follows:

Keys | Action
---- | ------
5/%  | Left thumb pitch
$/4  | Left thumb roll
#/3  | Left curl/uncurl index finger
@/2  | Left curl/uncurl middle finger
!/1  | Left curl/uncurl ring finger
`/~  | Left reset hand to home position
6/^  | Right thumb pitch
7/&  | Right thumb roll
8/*  | Right curl/uncurl index finger
9/(  | Right curl/uncurl middle finger
0/)  | Right curl/uncurl ring finger
-/_  | Right reset hand to home position

## Watching the messages

At this point, we have a basic idea of how to control the hand with the keyboard.  Let's go a little bit further and look at what exactly the keyboard_teleop node publishes when you hit a key.


Assuming you followed the tutorial above, you should have the simulation running in a gazebo window, and another terminal in which keyboard_teleop is running.  To start with, make sure the keyboard_teleop terminal is active, then press '`' and '-' to reset the hands to the home position.  Open up another terminal, and in that terminal run:

    $ rostopic echo /right_hand_position_controller/command


This will print out all of the messages that are being printed on the `/right_hand_position_controller/command` topic.  Now that you have that, go back to the terminal window running keyboard_teleop, and press '6' once.  Switching back to the terminal window containing the rostopic echo command, you should see this:

    layout: 
      dim: 
        - 
          label: fingers
          size: 5
          stride: 5
      data_offset: 0
    data: [0.1, 0.0, 0.0, 0.0, 0.0]
    ---

There's a couple of things to note about this message.  First, the size describes how many elements are in the data array, five in this case.  Second, the data array is all 0.0 except for the 0th element.  There you see the keyboard_teleop program has requested a position of 0.1, so the controller will attempt to drive to that position for that joint (thumb pitch).

By using the command:

    $ rostopic type /right_hand_position_controller/command

We see that the /right_hand_position_controller/command topic is of type `std_msgs/Float64MultiArray`.  By using the command:

    $ rosmsg show std_msgs/Float64MultiArray

We can see that a `std_msgs/Float64MultiArray` looks like:

    std_msgs/MultiArrayLayout layout
      std_msgs/MultiArrayDimension[] dim
        string label
        uint32 size
        uint32 stride
      uint32 data_offset
    float64[] data

For the hand, the data array must have 5 elements, one to control each finger (with two for the thumb).  The array elements map to the fingers (and the keyboard keys) as follows:

Element | Finger      | Keyboard
------- | ------      | --------
   0    | Thumb pitch | 5 (left), 6 (right)
   1    | Thumb roll  | $ (left), 7 (right)
   2    | Index       | # (left), 8 (right)
   3    | Middle      | @ (left), 9 (right)
   4    | Ring        | ! (left), 0 (right)

Thus, to control the hand from your own code, you should do the following:

1.  Construct a message of type `std_msgs/Float64MultiArray`.
1.  Fill in the size and stride of the MultiArray dimension as 5.
1.  Set the data_offset of the MultiArray to 0.
1.  Fill in the positions for each of the fingers.
1.  Publish the message to the `/right_hand_position_controller/command` or `/left_hand_position_controller/command` topic.