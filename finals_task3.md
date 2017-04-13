# Overview

** Not released yet **

Task 3 consists of the following checkpoints:

* **Checkpoint 1**: Climb the stairs
* **Checkpoint 2**: Open the door
* **Checkpoint 3**: Pass through the door
* **Checkpoint 4**: Pick up the leak detector
* **Checkpoint 5**: Find the leak
* **Checkpoint 6**: Pick up the leak repair tool
* **Checkpoint 7**: Repair the leak
* **Checkpoint 8**: Walk into Task 3's finish box

All checkpoints must be completed within 2 hours.

![t3_a.png](https://bitbucket.org/repo/xEbAAe/images/3877616181-t3_a.png)

## Practice run

We provide an example world file which only contains task 3 for convenience
when practicing.

An example step-by-step of how to run task 3:

#### Setup

1. Remember to source this file for every new terminal shell:

    ```
    source /opt/nasa/indigo/setup.bash
    ```

1. On a terminal, run a world which only contains task 3:

    ```
    roslaunch srcsim unique_task3.launch init:="true"
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

![t3_b.png](https://bitbucket.org/repo/xEbAAe/images/1499553971-t3_b.png)

1. Once you've setup your controllers and are ready to perform the task, on a
   new terminal, call a service to start task 3's first checkpoint:

    ```
    rosservice call /srcsim/finals/start_task 3 1
    ```

1. You'll see the following message on the console:

        [Msg] Task [1] - Skipped (35 236000000)
        [Msg] Task [2] - Skipped (35 236000000)
        [Msg] Task [3] - Started: time will start counting as you leave the box.
        [Msg] Started box contains plugin [task3/start]

1. The moment you step out of the start box, you'll see messages on the
   previous terminal, such as:

        task: 3
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

        [Msg] Task [3] - Checkpoint [1] - Started (35 236000000)
        [Msg] Started box contains plugin [task3/checkpoint1]

#### Checkpoint 1

We'll start on checkpoint 1: *Climb the stairs*.

![t3_c.png](https://bitbucket.org/repo/xEbAAe/images/1178427936-t3_c.png)

1. Perform checkpoint 1: climb to the top of the stairs.

1. When Val reaches the top, you'll see something like this:

        task: 3
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

        [Msg] Stopped box contains plugin [task3/checkpoint1]
        [Msg] Task [3] - Checkpoint [1] - Completed (40 332000000)
        [Msg] Task [3] - Checkpoint [2] - Started (40 332000000)

#### Checkpoint 2

We're now performing checkpoint 2: *Open the door*.

Turn valve to unlock | Push door open
----- | -----
![t3_d.png](https://bitbucket.org/repo/xEbAAe/images/3460481447-t3_d.png) | ![t3_j.png](https://bitbucket.org/repo/xEbAAe/images/3274264517-t3_j.png)

1. The door starts locked. To unlock it, first turn the door valve a full
rotation counter-clockwise.

1. When the full rotation is completed, you'll see the message:

    [Msg] Task [3] - Checkpoint [2] - Door unlocked

1. Now push the door until it is completely open (90 degrees from the initial
pose).

1. Once the door is completely open, the checkpoint is complete. You'll see:

        task: 3
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

        [Msg] Task [3] - Checkpoint [2] - Completed (40 756000000)
        [Msg] Task [3] - Checkpoint [3] - Started (40 756000000)
        [Msg] Started box contains plugin [task3/checkpoint3]

#### Checkpoint 3

We're now performing checkpoint 3: *Pass through the door*.

1. Perform checkpoint 3: walk all the way into the habitat. This means you
must reach the large grey area to the right of the door.

1. The checkpoint will be complete the moment you enter the large module.
You'll then see:

        task: 3
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

        [Msg] Task [3] - Checkpoint [3] - Completed (42 942000000)
        [Msg] Task [3] - Checkpoint [4] - Started (42 942000000)
        [Msg] Started touch plugin [task3/checkpoint4]

#### Checkpoint 4

You're now performing checkpoint 4: *Pick up the leak detector*.

1. Perform checkpoint 4: Lift the leak detector from the table. The moment
Val picks it up you'll see:

        [Msg] Model [air_leak_detector] started touching [valkyrie] at 43 665000000 seconds

1. Once the detector has been held for 0.5 seconds, the task message will be
updated to show checkpoint 4 has been completed:

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

        [Msg] Model [air_leak_detector] touched [valkyrie] exclusively for 0 500000000 seconds
        [Msg] Stopped touch plugin [task3/checkpoint4]
        [Msg] Task [3] - Checkpoint [4] - Completed (44 165000000)
        [Msg] Task [3] - Checkpoint [5] - Started (44 165000000)

#### Checkpoint 5

You're now performing checkpoint 5: *Find the leak*.

The leak is located on the wall to the left of the entrance. It is somewhere
between 0.8 m and 1.4 m from the ground.

The detector publishes messages on a topic with the value it is currently reading.
The value ranges from 0 to 1, where 0 means far away and 1 means close to
the detector's antena.

The value emitted by the detector is based on a frustum coming out of the antena:

* Whenever the leak is out of the frustum, the detector reports 0.01.
* Whenever the leak is within the frustum, the detector reports a value equal to
`f ^ d`, where `f` is a constant, and `d` is the distance from the center of
the leak to the point on the antena located at the base of the frustum.

1. On a new terminal, start listening to the detector's readings:

        rostopic echo /task3/checkpoint5/leak

1. At first, you'll see readings like this:

        ---
        value: 0.01
        ---

1. Get close to the wall and point the detector at it.

1. Move the detector around until you get readings indicating that the leak is
within the frustum (reading above `0.01`).
Once the leak is within the frustum, this checkpoint is complete:

        task: 3
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

        [Msg] Task [3] - Checkpoint [5] - Completed (59 483000000)
        [Msg] Task [3] - Checkpoint [6] - Started (59 483000000)
        [Msg] Started touch plugin [task3/checkpoint6]

#### Checkpoint 6

You're now performing checkpoint 5: *Pick up the leak repair tool*.

1. Pick up the leak patch tool from the table. The moment Val touches it
you'll see:


        [Msg] Model [leak_patch_tool] started touching [valkyrie] at 66 866000000 seconds

1. After 0.5 s of continuous contact, the checkpoint is complete:

        task: 3
        current_checkpoint: 7
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
        finished: False

    And you'll see a message like this:

        [Msg] Task [3] - Checkpoint [6] - Completed (67 366000000)
        [Msg] Task [3] - Checkpoint [7] - Started (67 366000000)

#### Checkpoint 7

You're now performing checkpoint 7: *Repair the leak*.

1. Carry the patch tool back to where the leak is.

1. Press the tool's tip against the leak.

1. Press the button on the tool.

1. Keep the tool pressed against the leak, and the button pressed, for 2 s and
the checkpoint will be complete!

        task: 3
        current_checkpoint: 8
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
          -
            secs: 77
            nsecs: 938000000
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 32
          nsecs: 221000000
        timed_out: False
        finished: False

    And you'll see a message like this:

        [Msg] Task [3] - Checkpoint [7] - Completed (77 938000000)
        [Msg] Task [3] - Checkpoint [8] - Started (77 938000000)

#### Checkpoint 8

You're now performing checkpoint 8: *Go to the finish box*.

1. Checkpoint 8 consists of moving to the finish box located on the long module.

1. Walk to the finish box.

1. Task 3 should be completed! The task message is updated:

        task: 0
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
          -
            secs: 77
            nsecs: 938000000
          -
            secs: 83
            nsecs: 710000000
        start_time:
          secs: 35
          nsecs: 236000000
        elapsed_time:
          secs: 32
          nsecs: 221000000
        timed_out: False
        finished: True

    And you'll see a message like this:

        [Msg] Stopped box contains plugin [task3/checkpoint8]
        [Msg] Task [3] - Checkpoint [6] - Completed (67 366000000)
        [Msg] Task [3] finished.

## Timeout

Make sure you complete all checkpoints in task 3 within 2 hours.

## Other resources

* [API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)
* [Skipping checkpoints](https://bitbucket.org/osrf/srcsim/wiki/skip_summary)
* [Practice versus competition](https://bitbucket.org/osrf/srcsim/wiki/practice_vs_competition)
* [Random world generator](https://bitbucket.org/osrf/srcsim/wiki/world_generator)