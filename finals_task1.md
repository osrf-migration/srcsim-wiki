# Overview

Task 1 consists of the following checkpoints:

* **Checkpoint 1**: Move within 1 meter from the communication dish
* **Checkpoint 2**: Move the communication dish **either** to the correct pitch or the correct yaw angle
* **Checkpoint 3**: Move the communication dish to **both** correct pitch and yaw angles
* **Checkpoint 4**: Walk into Task 1's finish box

All checkpoints must be completed within 30 minutes.

![t1.png](https://bitbucket.org/repo/xEbAAe/images/1480766054-t1.png)

### Updates

The tutorial below has been updated to reflect the updates.

##### New on SRCSim 0.4.0

* There's now a start box.
* The satellite control panel colors have changed.
* Clockwise rotations increase angles.

##### New on SRCSim 0.5.0

* There are 4 checkpoints instead of 3 (previous checkpoint 2 has been split into 2 and 3)

##### New on SRCSim 0.6.0

* The `init` parameter is no longer needed, Val is always de-harnessed.
* New [Task](https://bitbucket.org/osrf/srcsim/raw/default/msg/Task.msg) message structure
* New [Score](https://bitbucket.org/osrf/srcsim/raw/default/msg/Score.msg) message
* It's possible to restart a checkpoint

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
    roslaunch srcsim unique_task1.launch
    ```

1. Wait until the robot is detached from the harness before beginning. You'll
   know when you're ready when you see the `[Msg] [Harness] Detached` message
   on the terminal.

1. On a new terminal, listen to task updates:

    ```
    rostopic echo /srcsim/finals/task
    ```

    You shouldn't see anything yet, because the task has not started.

1. On another new terminal, listen to score updates:

    ```
    rostopic echo /srcsim/finals/score
    ```

    You shouldn't see anything yet, because the task has not started.

#### Start task

You'll start the task inside a start walkway. Time only starts counting as you
leave the start walkway.

![t1start.png](https://bitbucket.org/repo/xEbAAe/images/1585779460-t1start.png)

1. Once you've setup your controllers and are ready to perform the task, on a
   new terminal, call a service to start task 1's first checkpoint:

    ```
    rosservice call /srcsim/finals/start_task 1 1
    ```

1. You'll see the following message on the console:

        [Msg] Task [1] - Started: time will start counting as you leave the box.
        [Msg] Started box contains plugin [task1/start]

1. The task message will start being published. You'll see it says the current
   checkpoint is zero (i.e. no checkpoint) and the start time is zero.

        task: 1
        current_checkpoint: 0
        checkpoint_durations: []
        checkpoint_penalties: []
        start_time:
          secs: 0
          nsecs:  0
        elapsed_time:
          secs: 0
          nsecs: 0
        timed_out: False
        finished: False

1. You'll also see that the score message says zero score:

        checkpoint_durations: []
        checkpoint_penalties: []
        score: 0
        total_completion_time:
          secs: 0
          nsecs: 0

1. The moment you step out of the start box, you'll see messages on the
   previous terminal, such as:

        task: 1
        current_checkpoint: 1
        checkpoint_durations: []
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 65
          nsecs:  21000000
        elapsed_time:
          secs: 3
          nsecs: 609000000
        timed_out: False
        finished: False

    You can see that you're now attempting checkpoint 1, you have a start and
    elapsed time, and the first penalty says zero. The time penalty for the
    checkpoint will be increased for each time you restart it, and if you skip it.

    There will always be one more penalty than duration, because durations are added
    for each completed / skipped checkpoint, and penalties also includes the current
    checkpoint.

    You should also see messages like this on the console:

        [Msg] Task [1] - Checkpoint [1] - Started (65 21000000)
        [Msg] Started box contains plugin [task1/checkpoint1]

#### Checkpoint 1

Start checkpoint 1: *move the robot within 1 meter from the satellite dish*

![t1_a.png](https://bitbucket.org/repo/xEbAAe/images/2053972142-t1_a.png)

1. Perform checkpoint 1: use your controllers to move towards the satellite dish.

1. Once you reach the satellite dish, the task message will be updated with the
time it took to complete checkpoint 1. You'll see something like this:

        task: 1
        current_checkpoint: 2
        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 65
          nsecs:  21000000
        elapsed_time:
          secs: 30
          nsecs: 000000000
        timed_out: False
        finished: False

    Your score should have increased to 1:

        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        score: 1
        total_completion_time:
          secs: 30
          nsecs: 0

    There will also be a confirmation message on the console telling you that
    checkpoint 2 has started:

        [Msg] Stopped box contains plugin [task1/checkpoint1]
        [Msg] Task [1] - Checkpoint [1] - Completed (95 21000000)
        [Msg] Task [1] - Checkpoint [2] - Started (95 21000000)
        [Msg] Started satellite plugin

#### Checkpoint 2

We're now performing checkpoint 2: _Move the communication dish **either** to the
correct pitch or the correct yaw angle_. This consists of rotating the handles on the
satellite until the a correct angle is achieved.

![t1_b.png](https://bitbucket.org/repo/xEbAAe/images/3163083520-t1_b.png)

The friction on the valve joints will be randomized for each world. The ranges are as follows:

Joint | Min friction | Max friction | Min damping | Max damping
----- | ------------ | ------------ | ----------- | -----------
Wheel valves | 2.0 | 3.0 | 0.0 | 0.0

1. Conveniently, the satellite reports its current and target angles through a
ROS topic. On a new terminal, start listening to the satellite updates:

    ```
    rostopic echo /task1/checkpoint2/satellite
    ```

1. You'll see messages like:

        target_pitch: -0.2
        target_yaw: 1.0
        current_pitch: -0.0306981597753
        current_yaw: 0.238853778623
        pitch_correct_now: False
        yaw_correct_now: False
        pitch_completed: False
        yaw_completed: False

1. As the robot moves the handles, you'll see the numbers for `current_pitch`
and `current_yaw` change. These values are given in radians and correspond to
the satellite dish's angles, not the angles of the handle which the robot is
moving. The ratio between the handles and the dish movement is unknown to the
robot.

1. Once `current_pitch` is within a tolerance (currently using 5 degrees)
distance from `target_pitch`, `pitch_correct_now` will change to `True`. The
same goes for `yaw_correct_now`.

1. After `pitch_correct_now` is true for a target number of seconds
(now using 5 s), `pitch_completed` becomes True and checkpoint 2 is complete.
The same happens for yaw. You can complete either of them for this checkpoint
to be considered completed.

1. You should see a message like this on the task update, which includes the
time of checkpoint 2's completion:

        task: 1
        current_checkpoint: 3
        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 65
          nsecs:  21000000
        elapsed_time:
          secs: 50
          nsecs: 000000000
        timed_out: False
        finished: False

    The console will have a message like this:

        [Msg] Task [1] - Checkpoint [2] - Completed (115 21000000)
        [Msg] Task [1] - Checkpoint [3] - Started (115 21000000)

#### Checkpoint 3

We're now performing checkpoint 3: _Move the communication dish to **both**
correct pitch and yaw angles_.

1. Make sure you're still listening to the satellite message as before:

    ```
    rostopic echo /task1/checkpoint2/satellite
    ```

1. Manipulate the handle corresponding to the incomplete checkpoint so until that
checkpoint is also completed for 5 seconds.

    > Make sure not to dislocate the previously completed valve by mistake, because
      you'll need both to be correct for this checkpoint to be completed.

1. When both `pitch_completed` and `yaw_completed` are true for the same time
(i.e. both values have been correct for over 5 seconds at the same time),
checkpoint 3 is completed.

1. You should see a message like this on the task update, which includes the
time of checkpoint 3's completion:


        task: 1
        current_checkpoint: 4
        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 65
          nsecs:  21000000
        elapsed_time:
          secs: 80
          nsecs: 000000000
        timed_out: False
        finished: False

    The console will have a message like this:

        [Msg] Task [1] - Checkpoint [3] - Completed (135 21000000)
        [Msg] Task [1] - Checkpoint [4] - Started (135 21000000)
        [Msg] Started box contains plugin [task1/checkpoint4]

#### Checkpoint 4

You're now performing checkpoint 4: *Go to the finish box*.

![task1_d.png](https://bitbucket.org/repo/xEbAAe/images/4283297297-task1_d.png)

1. Walk to the finish box.

1. Task 1 should be completed. You'll see a message like this:

        [Msg] Stopped box contains plugin [task1/checkpoint4]
        [Msg] Task [1] - Checkpoint [4] - Completed (145 210000000)
        [Msg] Task [1] finished.

1. You'll get one last task message from task 1 as follows:

        task: 1
        current_checkpoint: 5
        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
          -
            secs: 10
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 65
          nsecs:  21000000
        elapsed_time:
          secs: 90
          nsecs: 000000000
        timed_out: False
        finished: True

    It says "checkpoint 5", which is the total number of checkpoints + 1, which
    means the task is over. Note also how `finished` is true.

1. If you completed all checkpoints without any restarts or skips, your score
   message should say you have 10 points (1+2+3+4), and the total completion time
   is the sum of all checkpoints:

        checkpoint_durations:
          -
            secs: 30
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
          -
            secs: 20
            nsecs: 00000000
          -
            secs: 10
            nsecs: 00000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        score: 10
        total_completion_time:
          secs: 80
          nsecs: 0

## Timeout

Make sure you complete all checkpoints in task 1 within 30 minutes.

## Other resources

* [API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)
* [Skipping checkpoints](https://bitbucket.org/osrf/srcsim/wiki/skip_summary)
* [Practice versus competition](https://bitbucket.org/osrf/srcsim/wiki/practice_vs_competition)
* [Random world generator](https://bitbucket.org/osrf/srcsim/wiki/world_generator)
