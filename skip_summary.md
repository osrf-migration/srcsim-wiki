## Skipping checkpoints

It's possible to skip checkpoints at any time during practice or during the
final competition.

Beware that failing to complete a checkpoint will cost you points. Also note
that it is not possible to go back to an earlier checkpoint.

You can restart a checkpoint as many times as you wish, but beware that you'll
receive an increasing time penalty each time you do so.

### How to skip

To skip to a checkpoint, simply use the task service to choose a new checkpoint
and the robot will be teleported to a position where it can start that
checkpoint from. The configuration of other objects may also be changed,
depending on the checkpoint.

For example, if you want to practice task 2 checkpoint 3 (press button on solar
panel), call the start task service with task 2, checkpoint 3:

    rosservice call /srcsim/finals/start_task 2 3

The result will be:

* Task 1 is completely skipped
* Task 2, checkpoint 1 is skipped
* Task 2, checkpoint 2 is skipped
* Task 2, checkpoint 3 is **not** skipped - this is the checkpoint you're currently
  trying to complete.

### Harness (from srcsim_0.6.0)

When the robot is teleported, it is placed on a harness which will carefully lower
the robot to the ground. You can subscribe to the [Harness](https://bitbucket.org/osrf/srcsim/raw/default/msg/Harness.msg) message to be notified of when the robot has been fully detached:

    rostopic echo /srcsim/finals/harness

### Must skip

You must explicitly skip checkpoints which you don't wish to complete. For
example, if you wish to walk directly to the finish box on task 1 (checkpoint 4)
and won't move the satellite handles, you must call:

    rosservice call /srcsim/finals/start_task 1 4

Just walking directly to the box without skipping to that checkpoint will **not
register the checkpoint as complete**.

### "Unskippable" checkpoints

It's not possible to skip directly to certain checkpoints. For example, it's not
possible to skip to task 2 checkpoint 2 (place solar panel on the array),
because that requires that you complete checkpoint 1 first (pick up solar panel).
Your options are:

1. Skip both checkpoints 1 and 2 with:

        rosservice call /srcsim/finals/start_task 2 3

1. Complete checkpoint 1 (pick up solar panel), and then skip checkpoint 2 with
the call above. You'll receive points for checkpoint 1.

### Restarting the current checkpoint

At any moment, you can restart the current checkpoint by calling the start task
service with that checkpoint's number. When you do so:

* The robot will be re-harnessed back to the beginning of the checkpoint
* Other objects might be rearranged according to the checkpoint
* You'll receive a time penalty

Make sure you wait for the harness to be detached as explained above.

### Summary

This is a summary of goals and skip behaviors for each checkpoint.

The last column of the table describes what happens when you **skip past** a
checkpoint. Note that this is different from **skipping to** that checkpoint.

Task | Checkpoint | Goal | Skip
---- | ---------- | ---- | ----
1 | 1 | Go to satellite | Robot is teleported to be in front of the satellite
1 | 2 | Rotate either handle to correct position | Robot is teleported to be in front of the satellite
1 | 3 | Rotate both handles to correct positions | Robot is teleported to be in front of the satellite
1 | 4 | Go to 1st finish box | Robot is teleported to be in the middle of the first finish box
2 | 1 | Lift solar panel | This checkpoint can't be skipped by itself
2 | 2 | Place solar panel on array | Robot is teleported to be in front of the array, panel is teleported to be on top of the array
2 | 3 | Press solar panel button | Robot is teleported to be in front of the array, solar panel opens
2 | 4 | Lift power cable | This checkpoint can't be skipped by itself
2 | 5 | Plug power cable | Robot is teleported to be in front of the array (nothing changes from 2/4)
2 | 6 | Go to 2nd finish box | Robot is teleported to be in the middle of the second finish box
3 | 1 | Climb stairs | Robot is teleported to be on top of the stairs, in front of the door
3 | 2 | Open door | Robot is teleported to be in front of the door, door opens
3 | 3 | Pass through door | Robot is teleported to be after the door
3 | 4 | Lift detector | This checkpoint can't be skipped by itself
3 | 5 | Find the leak | Robot is teleported to be after the door
3 | 6 | Lift repair tool | This checkpoint can't be skipped by itself
3 | 7 | Repair leak | Robot is teleported to be after the door
3 | 8 | Go to 3rd finish box | Program ends