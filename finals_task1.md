# Overview

Task 1 consists of the following checkpoints:

* Checkpoint 1: Move within 1 meter from the communication dish
* Checkpoint 2: Move the communication dish to the correct pitch and yaw angles
* Checkpoint 3: Walk into Task 1's finish box

All checkpoints must be completed within 30 minutes.

## Practice run

We provide an example world file which only contains task 1 for convenience
when practicing.

An example step-by-step of how to run task 1:

#### Setup

1. Remember to source this file for every new terminal shell:

    ```
    source /opt/nasa/indigo/setup.bash
    ```

1. On a terminal, run a world which only contains task 1:

    ```
    roslaunch srcsim unique_task1.launch init:="true"
    ```

1. On a new terminal, listen to task updates:

    ```
    rostopic echo /srcsim/finals/task
    ```

    You shouldn't see anything yet, because the task has not started.


#### Checkpoint 1

1. Once you're setup your controllers and are ready to perform the task, on a
   new terminal, call a service which starts task 1's first checkpoint:

    ```
    rosservice call /srcsim/finals/start_task 1 1
    ```

    You should see messages on the previous terminal, like this:

    ```
    ---
    task: 1
    current_checkpoint: 1
    checkpoints_completion: []
    start_time:
      secs: 65
      nsecs:  21000000
    elapsed_time:
      secs: 3
      nsecs: 609000000
    timed_out: False
    finished: False
    ---
    ```

1. Perform checkpoint 1: move within 1 meter from the satellite dish.

1. Once you reach the satellite dish, the task message will be updated with the
time checkpoint 1 was complete. You'll see something like this:

    ```
    ---
    task: 1
    current_checkpoint: 2
    checkpoints_completion:
      -
        secs: 95
        nsecs: 21000000
    start_time:
      secs: 65
      nsecs:  21000000
    elapsed_time:
      secs: 30
      nsecs: 000000000
    timed_out: False
    finished: False
    ---
    ```

#### Checkpoint 2

We're now performing checkpoint 2: Move the communication dish to the
correct pitch and yaw angles. This consists of rotating the handles on the
satellite until the correct angles are achieved.

1. Conveniently, the satellite reports its current and target angles through a
ROS topic. On a new terminal, start listening to the satellite updates:


    ```
    rostopic echo /task1/checkpoint2/satellite
    ```

1. You'll see messages like:

    ```
    ---
    target_pitch: -0.2
    target_yaw: 1.0
    current_pitch: -0.0306981597753
    current_yaw: 0.238853778623
    pitch_correct_now: False
    yaw_correct_now: False
    pitch_completed: False
    yaw_completed: False
    ---
    ```

1. As the robot moves the handles, you'll see the numbers for `current_pitch`
and `current_yaw` change. These values are given in radians and correspond to
the satellite's angles, not the angles of the handle whcih the robot is moving.

1. Once `current_pitch` is within a tolerance (currently using 5 degrees)
distance from `target_pitch`, `pitch_correct_now` will change to `True`. The
same goes for `yaw_correct_now`.

1. After `pitch_correct_now` is true for a target number of seconds
(now using 5 s), `pitch_completed` becomes True. Make sure the handle stays in
place, because if the pitch value moves away from the target, `pitch_completed`
goes back to False. The same applies for yaw.

1. When both `pitch_completed` and `yaw_completed` are true for the same time
(i.e. both values have been correct for over 5 seconds at the same time),
checkpoint 2 is completed.

1. You should see a message like this on the task update, which includes the
time of checkpoint 2's completion:


    ```
    ---
    task: 1
    current_checkpoint: 3
    checkpoints_completion:
      -
        secs: 95
        nsecs: 21000000
      -
        secs: 115
        nsecs: 21000000
    start_time:
      secs: 65
      nsecs:  21000000
    elapsed_time:
      secs: 50
      nsecs: 000000000
    timed_out: False
    finished: False
    ---
    ```

#### Checkpoint 3

1. Checkpoint 3 consists of moving to the finish box. Make sure we're in this
checkpoint by looking at the `current_checkpoint` field of the task status
message.

1. Walk to the finish box.

1. The task should be completed.


## Timeout

Make sure you complete all the checkpoints within 30 minutes.

## Skipping checkpoints

It's possible to skip checkpoints at any time during practice or during the
final competition.

Beware that failing to complete a checkpoint will cost you points. Also note
that it is not possible to restart a checkpoint or to go back to an earlier
checkpoint.

To skip to a checkpoint, simply use the task service to choose a new checkpoint
and the robot will be teleported to a position where it can start that
checkpoint from.

For example, to start practice in front of the satellite (skip checkpoint 1),
call the start task service with task 1, checkpoint 2:

    ```
    rosservice call /srcsim/finals/start_task 1 2
    ```

You must explicitly skip checkpoints which you don't wish to complete. For
example, if you wish to walk directly to the finish box (checkpoint 3) and
won't move the satellite handles, you must call:

    ```
    rosservice call /srcsim/finals/start_task 1 3
    ```

Otherwise, it will never be registered that you reached the finish box.

