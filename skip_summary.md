## Summary of behavior when skipping checkpoints

### Goal vs Skip

The last column of the table describes what happens when a checkpoint is skipped. Note that this is different from "skipping to" that checkpoint. For example, if you start a round and **skip to task 2, checkpoint 3**, i.e. `rosservice call /srcsim/finals/start_task 2 3`:

* Task 1 is completely skipped
* Task 2, checkpoint 1 is skipped
* Task 2, checkpoint 2 is skipped
* Task 2, checkpoint 3 is **not** skipped - this is the checkpoint you're currently trying to complete.

### "Unskippable" checkpoints

TODO: Add note about checkpoints which can't be skipped 

Task | Checkpoint | Goal | Skip
---- | ---------- | ---- | ----
1 | 1 | Go to satellite | Robot is teleported to be in front of the satellite
1 | 2 | Rotate handles to correct positions | Robot is teleported to be in front of the satellite (nothing changes from 1/1)
1 | 3 | Go to 1st finish box | Robot is teleported to be in the middle of the first finish box
2 | 1 | Lift solar panel | This checkpoint can't be skipped.
2 | 2 | Place solar panel on array | Robot is teleported to be in front of the array, panel is teleported to be on top of the array
2 | 3 | Press solar panel button | Robot is teleported to be in front of the array, solar panel opens
2 | 4 | Lift power cable | This checkpoint can't be skipped.
2 | 5 | Plug power cable | Robot is teleported to be in front of the array (nothing changes from 2/4)
2 | 6 | Go to 2nd finish box | Robot is teleported to be in the middle of the second finish box
3 | 1 | Climb stairs | Robot is teleported to be on top of the stairs
3 | 2 | Open door | Robot is teleported to be in front of the door, door opens
3 | 3 | Pass through door | Robot is teleported to be after the door
3 | 4 | Lift detector | This checkpoint can't be skipped.
3 | 5 | Find the leak | Robot is teleported to be near the leak
3 | 6 | Lift repair tool | This checkpoint can't be skipped.
3 | 7 | Repair leak | Robot is teleported to be near the leak
3 | 8 | Go to 3rd finish box | Program ends