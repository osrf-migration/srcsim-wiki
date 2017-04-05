# Overview

** Not released yet **

Task 2 consists of the following checkpoints:

* **Checkpoint 1**: Pick up the solar panel
* **Checkpoint 2**: Place solar panel within reach of the power cable
* **Checkpoint 3**: Deploy solar panel by pressing a button
* **Checkpoint 4**: Pick up the power cable
* **Checkpoint 5**: Plug the power cable into the solar panel
* **Checkpoint 6**: Walk into Task 2's finish box

All checkpoints must be completed within 1 hour.

![t2over.png](https://bitbucket.org/repo/xEbAAe/images/3846487934-t2over.png)

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
    roslaunch srcsim unique_task2.launch init:="true"
    ```

1. Wait to see the `Init: Done` message on the terminal before beginning.

1. On a new terminal, listen to task updates:

    ```
    rostopic echo /srcsim/finals/task
    ```

    You shouldn't see anything yet, because the task has not started.

#### Start task

You'll start the task inside a start walkway. Time only starts counting as you
leave the start walkway.

![t2start.png](https://bitbucket.org/repo/xEbAAe/images/2175615210-t2start.png)

1. Once you've setup your controllers and are ready to perform the task, on a
   new terminal, call a service to start task 1's first checkpoint:

    ```
    rosservice call /srcsim/finals/start_task 2 1
    ```

1. You'll see the following message on the console:

        [Msg] Task [2] - Started: time will start counting as you leave the box.
        [Msg] Started box contains plugin [task2/start]

1. The moment you step out of the start box, you'll see messages on the
   previous terminal, such as:

        task: 2
        current_checkpoint: 1
        checkpoints_completion: []
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 0
          nsecs:   1000000
        timed_out: False
        finished: False

    You should also see messages like this on the console:

        [Msg] Task [1] - Skipped (35 236000000)
        [Msg] Task [2] - Checkpoint [1] - Started (35 236000000)
        success: True

#### Checkpoint 1

We'll start on checkpoint 1: *Pick up the solar panel*.

![task_2_b.png](https://bitbucket.org/repo/xEbAAe/images/3973924048-task_2_b.png)

1. Perform checkpoint 1: use your controllers to move towards the Mars Explorer
and lift the solar panel. The moment Val picks up the solar panel, you'll see
a message on the console:

        [Msg] Model [solar_panel] started touching [valkyrie] at 39 832000000 seconds

1. Once the panel has been lifted for 0.5 seconds, the task message will be
updated with the time checkpoint 1 was complete. You'll see something like this:

        task: 2
        current_checkpoint: 2
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 5
          nsecs: 518000000
        timed_out: False
        finished: False

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

![task_2_c](https://bitbucket.org/repo/xEbAAe/images/2001505714-task_2_c.png)

1. Perform checkpoint 2: Place the solar panel anywhere on top of the solar
array.

1. Once the panel reaches the region on top of the solar array, you'll see task
messages telling you the checkpoint is completed, such as:

        task: 2
        current_checkpoint: 3
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
          -
            secs: 40
            nsecs: 756000000
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

![task_2_d.png](https://bitbucket.org/repo/xEbAAe/images/22174731-task_2_d.png)

1. Perform checkpoint 3: Press the button on top of the solar panel so it
is deployed.

1. The panel will unfold and you'll see the task message updating:

        task: 2
        current_checkpoint: 4
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
          -
            secs: 40
            nsecs: 756000000
          -
            secs: 42
            nsecs: 942000000
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
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
          -
            secs: 40
            nsecs: 756000000
          -
            secs: 42
            nsecs: 942000000
          -
            secs: 44
            nsecs: 165000000
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
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
          -
            secs: 40
            nsecs: 756000000
          -
            secs: 42
            nsecs: 942000000
          -
            secs: 44
            nsecs: 165000000
          -
            secs: 59
            nsecs: 483000000
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
        current_checkpoint: 0
        checkpoints_completion:
          -
            secs: 40
            nsecs: 332000000
          -
            secs: 40
            nsecs: 756000000
          -
            secs: 42
            nsecs: 942000000
          -
            secs: 44
            nsecs: 165000000
          -
            secs: 59
            nsecs: 483000000
          -
            secs: 67
            nsecs: 366000000
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 32
          nsecs: 221000000
        timed_out: False
        finished: True

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