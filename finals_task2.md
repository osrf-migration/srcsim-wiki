# Overview

Task 2 consists of the following checkpoints:

* **Checkpoint 1**: Pick up the solar panel
* **Checkpoint 2**: Place solar panel within reach of the power cable
* **Checkpoint 3**: Deploy solar panel by pressing a button
* **Checkpoint 4**: Pick up the power cable
* **Checkpoint 5**: Plug the power cable into the solar panel
* **Checkpoint 6**: Walk into Task 2's finish box

All checkpoints must be completed within 1 hour.

![t2_a.png](https://bitbucket.org/repo/xEbAAe/images/191484822-t2_a.png)

### Updates

The tutorial below has been updated to reflect the updates.

##### New on SRCSim 0.6.0

* The cable's shape has been changed
* The array is 0.2 m higher
* The `init` parameter is no longer needed, Val is always de-harnessed.
* New [Task](https://bitbucket.org/osrf/srcsim/raw/default/msg/Task.msg) message structure
* New [Score](https://bitbucket.org/osrf/srcsim/raw/default/msg/Score.msg) message
* It's possible to restart a checkpoint

## Practice run

We provide an example world file which only contains task 2 for convenience
when practicing.

An example step-by-step of how to run task 2:

#### Setup

1. Remember to source this file for every new terminal shell:

    ```
    source /opt/nasa/indigo/setup.bash
    ```

1. On a terminal, run a world which only contains task 2:

    ```
    roslaunch srcsim unique_task2.launch
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

![t2start.png](https://bitbucket.org/repo/xEbAAe/images/2175615210-t2start.png)

1. Once you've setup your controllers and are ready to perform the task, on a
   new terminal, call a service to start task 2's first checkpoint:

    ```
    rosservice call /srcsim/finals/start_task 2 1
    ```

1. You'll see the following message on the console:

        [Msg] Task [1] - Skipped (35 236000000)
        [Msg] Task [2] - Started: time will start counting as you leave the box.
        [Msg] Started box contains plugin [task2/start]

1. The task message will start being published. You'll see it says the current
   checkpoint is zero (i.e. no checkpoint) and the start time is zero.

        task: 2
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

        task: 2
        current_checkpoint: 1
        checkpoint_durations: []
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 0
          nsecs:   1000000
        timed_out: False
        finished: False

    You can see that you're now attempting checkpoint 1, you have a start and
    elapsed time, and the first penalty says zero. The time penalty for the
    checkpoint will be increased for each time you restart it, and if you skip it.

    There will always be one more penalty than duration, because durations are added
    for each completed / skipped checkpoint, and penalties also includes the current
    checkpoint.

    You should also see messages like this on the console:

        [Msg] Task [2] - Checkpoint [1] - Started (35 236000000)
        success: True

#### Checkpoint 1

We'll start on checkpoint 1: *Pick up the solar panel*.

![t2_b.png](https://bitbucket.org/repo/xEbAAe/images/1005836777-t2_b.png)

1. Perform checkpoint 1: use your controllers to move towards the Mars Explorer
and lift the solar panel. The moment Val picks up the solar panel, you'll see
a message on the console:

        [Msg] Model [solar_panel] started touching [valkyrie] at 39 832000000 seconds

1. Once the panel has been lifted for 0.5 seconds, the task message will be
time it took to complete checkpoint 1. You'll see something like this:

        task: 2
        current_checkpoint: 2
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
        checkpoint_penalties:
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 5
          nsecs: 518000000
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

        [Msg] Model [solar_panel] touched [valkyrie] exclusively for 0 500000000 seconds
        [Msg] Stopped touch plugin [task2/checkpoint1]
        [Msg] Task [2] - Checkpoint [1] - Completed (40 332000000)
        [Msg] Task [2] - Checkpoint [2] - Started (40 332000000)
        [Msg] Started box contains plugin [task2/checkpoint2]

#### Checkpoint 2

We're now performing checkpoint 2: *Place solar panel within reach of the power
cable*.

![t2_c.png](https://bitbucket.org/repo/xEbAAe/images/296043003-t2_c.png)

1. Perform checkpoint 2: Place the solar panel anywhere on top of the solar
array.

1. Once the panel reaches the region on top of the solar array, you'll see task
messages telling you the checkpoint is completed, such as:

        task: 2
        current_checkpoint: 3
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
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
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 5
          nsecs: 520000000
        timed_out: False
        finished: False

    There will be messages on the console confirming that checkpoint 2 is
    complete, and we're now on checkpoint 3:

        [Msg] Task [2] - Checkpoint [2] - Completed (40 756000000)
        [Msg] Task [2] - Checkpoint [3] - Started (40 756000000)
        [Msg] Stopped box contains plugin [task2/checkpoint2]
        [Msg] Started solar panel plugin

#### Checkpoint 3

We're now performing checkpoint 3: *Deploy solar panel by pressing a button*.

![t2_d.png](https://bitbucket.org/repo/xEbAAe/images/223290231-t2_d.png)

1. Perform checkpoint 3: Press the button on top of the solar panel so it
is deployed.

1. The panel will unfold and you'll see the task message updating:

        task: 2
        current_checkpoint: 4
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
          -
            secs: 2
            nsecs: 186000000
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
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 8
          nsecs: 438000000
        timed_out: False
        finished: False

    And the terminal messages will tell us we are on the next checkpoint:

        [Msg] Solar panel is open
        [Msg] Task [2] - Checkpoint [3] - Completed (42 942000000)
        [Msg] Task [2] - Checkpoint [4] - Started (42 942000000)
        [Msg] Started touch plugin [task2/checkpoint4]

#### Checkpoint 4

You're now performing checkpoint 4: *Pick up the power cable*.

![t2cp4_2.png](https://bitbucket.org/repo/xEbAAe/images/80016327-t2cp4_2.png)

1. Perform checkpoint 4: Lift the tip of the cable. The moment Val picks it up,
you'll see a message:

        [Msg] Model [solar_panel_cable] started touching [valkyrie] at 43 665000000 seconds

1. Once the cable has been held for 0.5 seconds, the task message will be updated to show checkpoint 4 has been completed:

        task: 2
        current_checkpoint: 5
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
          -
            secs: 2
            nsecs: 186000000
          -
            secs: 1
            nsecs: 223000000
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
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 10
          nsecs: 695000000
        timed_out: False
        finished: False

    There will also be feedback on the console:

        [Msg] Model [solar_panel_cable] touched [valkyrie] exclusively for 0 500000000 seconds
        [Msg] Stopped touch plugin [task2/checkpoint4]
        [Msg] Task [2] - Checkpoint [4] - Completed (44 165000000)
        [Msg] Task [2] - Checkpoint [5] - Started (44 165000000)

#### Checkpoint 5

You're now performing checkpoint 5: *Plug the power cable into the solar panel*.

![t2cp5_4.png](https://bitbucket.org/repo/xEbAAe/images/567550910-t2cp5_4.png)

1. Perform checkpoint 5: Touch the blue tip of the cable onto the blue outlet
on the solar panel. The moment they touch you'll see a message like:

        [Msg] Plug started touching outlet at 56 483000000 seconds

1. Keep them in continuous contact for 3 seconds, and they will get stuck. The
task message is updated:

        task: 2
        current_checkpoint: 6
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
          -
            secs: 2
            nsecs: 186000000
          -
            secs: 1
            nsecs: 223000000
          -
            secs: 15
            nsecs: 318000000
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
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 32
          nsecs: 221000000
        timed_out: False
        finished: False

    And the terminal notifies the completion:

        [Msg] The cable plug has been fixed to the outlet
        [Msg] Task [2] - Checkpoint [5] - Completed (59 483000000)
        [Msg] Task [2] - Checkpoint [6] - Started (59 483000000)
        [Msg] Started box contains plugin [task2/checkpoint6]

#### Checkpoint 6

You're now performing checkpoint 5: *Go to the finish box*.

![t2cp6.png](https://bitbucket.org/repo/xEbAAe/images/3175127162-t2cp6.png)

1. Checkpoint 6 consists of moving to the finish box.

1. Walk to the finish box.

1. Task 2 should be completed! The task message is updated:

        task: 2
        current_checkpoint: 7
        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
          -
            secs: 2
            nsecs: 186000000
          -
            secs: 1
            nsecs: 223000000
          -
            secs: 15
            nsecs: 318000000
          -
            secs: 7
            nsecs: 883000000
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
          -
            secs: 0
            nsecs: 0
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 32
          nsecs: 221000000
        timed_out: False
        finished: True

    It says "checkpoint 7", which is the total number of checkpoints + 1, which
    means the task is over. Note also how `finished` is true.

1. If you completed all checkpoints without any restarts or skips, your score
   message should say you have 21 points (1+2+3+4+5+6), and the total completion time
   is the sum of all checkpoints:

        checkpoint_durations:
          -
            secs: 5
            nsecs: 518000000
          -
            secs: 0
            nsecs: 442000000
          -
            secs: 2
            nsecs: 186000000
          -
            secs: 1
            nsecs: 223000000
          -
            secs: 15
            nsecs: 318000000
          -
            secs: 7
            nsecs: 883000000
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
          -
            secs: 0
            nsecs: 0
        score: 21
        total_completion_time:
          secs: 80
          nsecs: 0

    Note that during the competition you'll be performing task 2 immediately
    after task 1, so by the time you finish it, you might have an even higher
    score.

    And you'll see a message like this:

        [Msg] Stopped box contains plugin [task2/checkpoint6]
        [Msg] Task [2] - Checkpoint [6] - Completed (67 366000000)
        [Msg] Task [2] finished.

## Timeout

Make sure you complete all checkpoints in task 2 within 1 hour.

## Other resources

* [API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)
* [Skipping checkpoints](https://bitbucket.org/osrf/srcsim/wiki/skip_summary)
* [Practice versus competition](https://bitbucket.org/osrf/srcsim/wiki/practice_vs_competition)
* [Random world generator](https://bitbucket.org/osrf/srcsim/wiki/world_generator)
