# Controlling the hand

The SRC ships with a ROS node that allows you to teleop the R5 from the keyboard.  This is called the keyboard_teleop node.  Let’s start up the SRC simulation, and start up the keyboard_teleop node:

In one terminal, fire up the simulation; note that this tutorial uses qual2.launch, but you can use any other launch file that you choose. (IMPORTANT NOTE: the launch file *must* have the grasping arg set to true!)

    $ roslaunch srcsim qual2.launch init:=true

The simulation will launch and will slowly lower the R5 robot to the ground.  Once the harness has released, the robot is free to move.  The arm and hand position should look something like the picture below.

![src-armtut-01-default-position.png](https://bitbucket.org/repo/xEbAAe/images/844246496-src-armtut-01-default-position.png)

At this point, we want to make sure that the grasping controller is active.  In another terminal, run:

    $ rostopic list | grep hand_position_controller

You should see the /left_hand_position_controller/command and the /right_hand_position_controller/command topics there.  If you do not, go back into your launch file and make sure that the grasping_init arg is set to true.

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