# Overview

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

![task3_opt.gif](https://bitbucket.org/repo/xEbAAe/images/1336454180-task3_opt.gif)

### Updates

The tutorial below has been updated to reflect the updates.

##### New on SRCSim 0.6.0

* The table is 0.2 m higher
* The `init` parameter is no longer needed, Val is always de-harnessed.
* New [Task](https://bitbucket.org/osrf/srcsim/raw/default/msg/Task.msg) message structure
* New [Score](https://bitbucket.org/osrf/srcsim/raw/default/msg/Score.msg) message
* It's possible to restart a checkpoint

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
    roslaunch srcsim unique_task3.launch
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

1. The task message will start being published. You'll see it says the current
   checkpoint is zero (i.e. no checkpoint) and the start time is zero.

        task: 3
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

        task: 3
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

        [Msg] Task [3] - Checkpoint [1] - Started (35 236000000)
        [Msg] Started box contains plugin [task3/checkpoint1]

#### Checkpoint 1

We'll start on checkpoint 1: *Climb the stairs*.

![t3_c.png](https://bitbucket.org/repo/xEbAAe/images/1178427936-t3_c.png)

1. Perform checkpoint 1: climb to the top of the stairs.

1. When Val reaches the top, you'll see something like this:

        task: 3
        current_checkpoint: 2
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
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

    There will also be a confirmation message on the console telling you that
    checkpoint 2 has started:

        [Msg] Stopped box contains plugin [task3/checkpoint1]
        [Msg] Task [3] - Checkpoint [1] - Completed (40 332000000)
        [Msg] Task [3] - Checkpoint [2] - Started (40 332000000)

#### Checkpoint 2

We're now performing checkpoint 2: *Open the door*.

The friction of the door and valve hinges will fall within the following ranges for each world:

Joint | Min friction | Max friction | Min damping | Max damping
----- | ------------ | ------------ | ----------- | -----------
Valve | 1.0 | 1.6 | 0.01 | 0.05
Door hinge | 10.0 | 20.0 | 1.0 | 4.0


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
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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

        [Msg] Task [3] - Checkpoint [2] - Completed (40 756000000)
        [Msg] Task [3] - Checkpoint [3] - Started (40 756000000)
        [Msg] Started box contains plugin [task3/checkpoint3]

#### Checkpoint 3

We're now performing checkpoint 3: *Pass through the door*.

![t3_f.png](https://bitbucket.org/repo/xEbAAe/images/1182017986-t3_f.png)

1. Perform checkpoint 3: walk all the way into the habitat. This means you
must reach the large grey area to the right of the door.

1. The checkpoint will be complete the moment you enter the large module.
You'll then see:

        task: 3
        current_checkpoint: 4
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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

        [Msg] Task [3] - Checkpoint [3] - Completed (42 942000000)
        [Msg] Task [3] - Checkpoint [4] - Started (42 942000000)
        [Msg] Started touch plugin [task3/checkpoint4]

#### Checkpoint 4

You're now performing checkpoint 4: *Pick up the leak detector*.

![t3_g.png](https://bitbucket.org/repo/xEbAAe/images/2530120514-t3_g.png)

1. Perform checkpoint 4: Lift the leak detector from the table. The moment
Val picks it up you'll see:

        [Msg] Model [air_leak_detector] started touching [valkyrie] at 43 665000000 seconds

1. Once the detector has been held for 0.5 seconds, the task message will be
updated to show checkpoint 4 has been completed:

        task: 2
        current_checkpoint: 5
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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

        [Msg] Model [air_leak_detector] touched [valkyrie] exclusively for 0 500000000 seconds
        [Msg] Stopped touch plugin [task3/checkpoint4]
        [Msg] Task [3] - Checkpoint [4] - Completed (44 165000000)
        [Msg] Task [3] - Checkpoint [5] - Started (44 165000000)

#### Checkpoint 5

You're now performing checkpoint 5: *Find the leak*.

![t3_h.png](https://bitbucket.org/repo/xEbAAe/images/2959862727-t3_h.png)

See the **Leak** section below for detailed information about the leak detection.

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
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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

        [Msg] Task [3] - Checkpoint [5] - Completed (59 483000000)
        [Msg] Task [3] - Checkpoint [6] - Started (59 483000000)
        [Msg] Started touch plugin [task3/checkpoint6]

#### Checkpoint 6

You're now performing checkpoint 6: *Pick up the leak repair tool*.

![t3_l.png](https://bitbucket.org/repo/xEbAAe/images/84766533-t3_l.png)

1. Pick up the leak patch tool from the table. The moment Val touches it
you'll see:


        [Msg] Model [leak_patch_tool] started touching [valkyrie] at 66 866000000 seconds

1. After 0.5 s of continuous contact, the checkpoint is complete:

        task: 3
        current_checkpoint: 7
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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

    And you'll see a message like this:

        [Msg] Task [3] - Checkpoint [6] - Completed (67 366000000)
        [Msg] Task [3] - Checkpoint [7] - Started (67 366000000)

#### Checkpoint 7

You're now performing checkpoint 7: *Repair the leak*.

![t3_i.png](https://bitbucket.org/repo/xEbAAe/images/749924072-t3_i.png)

1. Carry the patch tool back to where the leak is.

1. Press the tool's tip against the leak.

1. Press the button on the tool.

1. Keep the tool pressed against the leak, and the button pressed, for 2 s and
the checkpoint will be complete!

        task: 3
        current_checkpoint: 8
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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
          -
            secs: 10
            nsecs: 572000000
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

    And you'll see a message like this:

        [Msg] Task [3] - Checkpoint [7] - Completed (77 938000000)
        [Msg] Task [3] - Checkpoint [8] - Started (77 938000000)

#### Checkpoint 8

You're now performing checkpoint 8: *Go to the finish box*.

![t3_m.png](https://bitbucket.org/repo/xEbAAe/images/3879188980-t3_m.png)

1. Checkpoint 8 consists of moving to the finish box located on the long module.

1. Walk to the finish box.

1. Task 3 should be completed! The task message is updated:

        task: 3
        current_checkpoint: 9
        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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
          -
            secs: 10
            nsecs: 572000000
          -
            secs: 5
            nsecs: 772000000
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

    It says "checkpoint 9", which is the total number of checkpoints + 1, which
    means the task is over. Note also how `finished` is true.

1. If you completed all checkpoints without any restarts or skips, your score
   message should say you have 36 points (1+2+3+4+5+6+7+8), and the total completion
   time is the sum of all checkpoints:

        checkpoint_durations:
          -
            secs: 5
            nsecs: 096000000
          -
            secs: 0
            nsecs: 424000000
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
          -
            secs: 10
            nsecs: 572000000
          -
            secs: 5
            nsecs: 772000000
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
          -
            secs: 0
            nsecs: 0
          -
            secs: 0
            nsecs: 0
        score: 36
        total_completion_time:
          secs: 48
          nsecs: 474000000

    Note that during the competition you'll be performing task 3 immediately
    after tasks 1 and 2, so by the time you finish it, you might have an even higher
    score.

    And you'll see a message like this:

        [Msg] Stopped box contains plugin [task3/checkpoint8]
        [Msg] Task [3] - Checkpoint [6] - Completed (67 366000000)
        [Msg] Task [3] finished.

## Timeout

Make sure you complete all checkpoints in task 3 within 2 hours.

## Leak

This section contains more detailed information about the leak location and
detection.

#### Leak location

During the competition, teams will not know the exact location of the leak
and must find it using the leak detector. However, the region where the leak
can be located is known, so the robot should only need to search that area.

It is known that the leak is located on the wall to the left of the entrance.
For each world, the leak is randomly placed somewhere between 0.835 m and
1.713 m from the ground.

The blue rectangle below marks the region where the leak can be (the rectangle
is not present in the worlds):

![wall3.png](https://bitbucket.org/repo/xEbAAe/images/2099664621-wall3.png)

![wall2.png](https://bitbucket.org/repo/xEbAAe/images/60609230-wall2.png)

For debugging purposes during practice, you can visualize the leak as follows:

1. On the left panel's world tab, scroll down until you find the `leak` model
1. Right-click `leak` and choose `View->Collisions`
1. The leak will show as an orange sphere on the wall, like this:

![leak1.png](https://bitbucket.org/repo/xEbAAe/images/2011524842-leak1.png)

Teams will not be able to visualize the leak during the finals.

#### Leak detection

The leak can be detected using the Air Leak Detector tool. The detector
publishes messages on ROS topic `/task3/checkpoint5/leak` with the value it is
currently reading.

The value ranges from 0 to 1, where the higher the number is, the closer the leak
is to the detector's antenna.

The value emitted is based on the following frustum coming out of the antenna.
Checkpoint 5 is completed when the leak is within this frustum.

![antenna.png](https://bitbucket.org/repo/xEbAAe/images/744960137-antenna.png)

Frustum dimensions | _
-- | --
Near plane distance from the origin of the tool | 0.2 m
Far plane distance from the origin of the tool | 0.5 m
Horizontal field of view | PI / 9 rad
Aspect ratio | 1

* Whenever the leak is out of the frustum, the detector reports 0.01
* Whenever the leak is within the frustum, the detector reports a value equal to
`f ^ d`, where `f` is a constant, and `d` is the distance from the center of
the leak to the point on the antenna located at the base of the frustum.

Some example readings:

0.01 | 0.021 | 0.574
---- | ----- | ----
![fru1.png](https://bitbucket.org/repo/xEbAAe/images/4206459929-fru1.png) | ![fru2.png](https://bitbucket.org/repo/xEbAAe/images/4064879223-fru2.png) | ![fru3.png](https://bitbucket.org/repo/xEbAAe/images/364842782-fru3.png)

#### Leak patching

To fix the leak, the tip of the patch tool must be physically pressed against the leak
and maintain continuous contact with it for a given time.

If you turn on collision visualization for the tool, you can see the collision shape
on the tool tip. (Right-click tool -> View -> Collisions)

If you turn on contact visualization, you can see a blue sphere when there is contact.
(On the top menu, choose View -> Contacts).

The robot's camera can't see collisions or contacts.

Collision visuals | Contact visual | Robot's camera view
----------------- | -------------- | -------------------
![patch2.png](https://bitbucket.org/repo/xEbAAe/images/4231633387-patch2.png) | ![patch1.png](https://bitbucket.org/repo/xEbAAe/images/2783548609-patch1.png) | ![patch3.png](https://bitbucket.org/repo/xEbAAe/images/390289498-patch3.png)



## Other resources

* [API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)
* [Skipping checkpoints](https://bitbucket.org/osrf/srcsim/wiki/skip_summary)
* [Practice versus competition](https://bitbucket.org/osrf/srcsim/wiki/practice_vs_competition)
* [Random world generator](https://bitbucket.org/osrf/srcsim/wiki/world_generator)
