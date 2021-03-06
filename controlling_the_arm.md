# Controlling the arm

## Starting up the simulation

The SRC ships with a ROS node that allows you to teleop the R5 from the keyboard.  This is called the keyboard_teleop node.  Let’s start up the SRC simulation, and start up the keyboard_teleop node.

In one terminal, fire up the simulation; note that this tutorial uses qual2.launch, but you can use any other launch file that you choose. Be sure to set both `init` and `grasping_init` to true.

    $ roslaunch srcsim qual2.launch init:=true grasping_init:=true

The simulation will launch and will slowly lower the R5 robot to the ground.  Once the harness has released, the robot is free to move.  The arm and hand position should look something like the picture below.

![src-armtut-01-default-position.png](https://bitbucket.org/repo/xEbAAe/images/844246496-src-armtut-01-default-position.png)

At this point, we want to make sure that the grasping controller is active.  In another terminal, run:

    $ rostopic list | grep hand_position_controller

You should see the `/left_hand_position_controller/command` and the `/right_hand_position_controller/command` topics there.  If you do not, make sure that you set the `grasping_init` argument to true when launching.

## Keyboard control

Now in another terminal fetch the keyboard_teleop node:

    $ wget https://bitbucket.org/osrf/srcsim/raw/default/scripts/keyboard_teleop.py

Now launch the keyboard_teleop node:

    $ python keyboard_teleop.py

When you first launch the `keyboard_teleop.py` node, you’ll get some information on how to control the robot using the keyboard, similar to what is below.

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


As can be seen above, there are controls for the left and right arms, left and right hands, the head, the neck, and for walking.  This tutorial will focus just on the arms.

Let’s control the arms of the robot.  Make sure the terminal with the keyboard_teleop node is in focus, and then send the arms to their “home” position by pressing `t` and then `y`.  With that done, the robot should look like below.

![src-armtut-02-arm-home-position.png](https://bitbucket.org/repo/xEbAAe/images/56993020-src-armtut-02-arm-home-position.png)

Next, let’s rotate the arms around the shoulder joint 90 degrees.  Press the `u` key approximately 15 times, then press the `q` key approximately 15 times.  When you are done, the shoulder should have rotated 90 degrees, and look like below.

![src-armtut-03-shoulder-rotation.png](https://bitbucket.org/repo/xEbAAe/images/3541547728-src-armtut-03-shoulder-rotation.png)

Now, let’s rotate the shoulders back to the original position by doing the opposite movement. That is achieved using the upper case keys `U` (`Shift`+`u`) and `Q` (`Shift`+`q`) about 15 times.  The robot should be approximately back at the “home” position.

Similarly, we can control the elbow joint on the arms by using the `r` and the `p` keys.  Press the `r` key about 15 times and the `p` key about 15 times to make the elbows go to approximately 90 degrees.  The robot should look like below.

![src-armtut-04-elbow-90-degrees.png](https://bitbucket.org/repo/xEbAAe/images/1554513140-src-armtut-04-elbow-90-degrees.png)

Additionally, we can control the hands using the keyboard.  Hit `t` and `y` to reset the arms to the home position.  Then control the index finger on the right hand by hitting the `8` key about 10 times.  The robot should look like below.

![src-armtut-05-right-finger.png](https://bitbucket.org/repo/xEbAAe/images/3737807227-src-armtut-05-right-finger.png)

Other keys on the keyboard do similar things.  For reference, the keys are mapped as follows:

Keys | Action
---- | ------
**left arm** |
q/Q  | Positive/negative rotation of the left arm joint around the shoulder (`leftShoulderPitch`)
W/w  | Positive/negative rotation of the left arm shoulder joint (`leftShoulderRoll`)
e/E  | Positive/negative rotation of the left upper arm (`leftShoulderYaw`)
R/r  | Positive/negative bend of the left elbow (`leftElbowPitch`)
a/A  | Positive/negative rotation of the left lower arm (`leftForearmYaw`)
S/s  | Positive/negative X offset of the left wrist (`leftWristRoll`)
D/d  | Positive/negative Y offset of the left wrist (`leftWristPitch`)
t/T  | Reset left arm to home position
**right arm** |
u/U  | Positive/negative rotation of the right arm joint around the shoulder (`rightShoulderPitch`)
i/I  | Positive/negative rotation of the right arm joint around the shoulder (`rightShoulderRoll`)
o/O  | Positive/negative rotation of the right arm upper arm (`rightShoulderYaw`)
p/P  | Positive/negative bend of the right elbow (`rightElbowPitch`)
j/J  | Positive/negative rotation of the right lower arm (`rightForearmYaw`)
k/K  | Positive/negative X offset of the right wrist (`rightWristRoll`)
l/L  | Positive/negative Y offset of the right wrist (`rightWristPitch`)
y/Y  | Reset right arm to home position
**left hand** |
5/% | left thumb roll
$/4 | left thumb pitch
#/3 | left index finger
@/2 | left middle finger
!/1 | left ring finger
`/~ | Reset left hand to home position
**right hand** |
6/^ | right thumb roll
7/& | right thumb pitch
8/* | right index finger
9/( | right middle finger
0/) | left ring finger
-/_ | Reset right hand to home position
**head angles** |
x/X | roll
c/C | pitch
v/V | yaw
g/G | Reset head to home position
**neck joints** |
b/B | lower neck pitch
n/N | neck yaw
m/M | upper neck pitch
h/H | Reset neck to home position

## Watching the messages

At this point, we have a basic idea on how to control the arm using the keyboard.  Let's go a little bit further and look at what exactly the keyboard_teleop node publishes when you hit a key.

Assuming you followed the tutorial above, you should have the simulation running in a gazebo window, and another terminal in which keyboard_teleop is running.  To start with, make sure the keyboard_teleop terminal is active, then press `t` and `y` to reset the arms to the home position.  Open up another terminal, and in that terminal run:

    $ rostopic echo /ihmc_ros/valkyrie/control/arm_trajectory

This will print out all of the messages that are being printed on the `/ihmc_ros/valkyrie/control/arm_trajectory` topic.  Now that you have that, go back to the terminal window running keyboard_teleop, and press `j` once.  Switching back to the terminal window containing the rostopic echo command, you should see this:

    robot_side: 1
    joint_trajectory_messages: 
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.1
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
      - 
        trajectory_points: 
          - 
            time: 1.0
            position: 0.0
            velocity: 0.0
            unique_id: 0
        unique_id: 0
    execution_mode: 0
    previous_message_id: 0
    unique_id: -1
    ---

There are a few things to notice about this message.  First, `robot_side` describes whether we are controlling the left (0) or the right (1) side of the robot.  Second, there is a list of `joint_trajectory_messages`; one for each joint that can be controlled in the arm.  Third, notice that all of the positions and velocities are 0.0, with the exception of the 4th trajectory (counting from 0).  There you see that we have requested position 0.1, so the controller will attempt to drive to that position for that joint (lower arm rotation).

By using the command:

    $ rostopic type /ihmc_ros/valkyrie/control/arm_trajectory

We can see that the `/ihmc_ros/valkyrie/control/arm_trajectory` topic is of type [`ihmc_msgs/ArmTrajectoryRosMessage`](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/ArmTrajectoryRosMessage.msg).  By using the command:

    $ rosmsg show ihmc_msgs/ArmTrajectoryRosMessage

We can see that an `ihms_msgs/ArmTrajectoryRosMessage` looks like:

    uint8 LEFT=0
    uint8 RIGHT=1
    uint8 OVERRIDE=0
    uint8 QUEUE=1
    uint8 robot_side
    ihmc_msgs/OneDoFJointTrajectoryRosMessage[] joint_trajectory_messages
      ihmc_msgs/TrajectoryPoint1DRosMessage[] trajectory_points
        float64 time
        float64 position
        float64 velocity
        int64 unique_id
      int64 unique_id
    uint8 execution_mode
    int64 previous_message_id
    int64 unique_id

For the arm, the `joint_trajectory_messages` array must have 7 elements, one for each degree of freedom on the arm.  The array elements map to the joints (and the keyboard keys) as follows:

Element | Joint               | Keyboard
------- | -----               | --------
   0    |  Shoulder rotation  | q (left), u (right)
   1    |  Shoulder joint     | W (left), i (right)
   2    |  Upper arm rotation | e (left), o (right)
   3    |  Elbow joint        | R (left), p (right)
   4    |  Lower arm rotation | a (left), j (right)
   5    |  Wrist X            | S (left), k (right)
   6    |  Wrist Y            | D (left), l (right)

Thus, to control the arm from your own code, you should do the following:

1.  Construct a message of type `ihms_msgs/ArmTrajectoryRosMessage`.
2.  Set the unique_id of the message to -1.
3.  Set the robot_side of the message to 0 to control the left, or 1 to control the right.
4.  Create an array of 7 elements in the joint_trajectory_elements.
5.  Fill in the position of each degree of freedom.
6.  Publish the message to the `/ihmc_ros/valkyrie/control/arm_trajectory` topic.