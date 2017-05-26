# Overview

Each run consists of 3 tasks and each task is composed of a different number of
checkpoints:

### [Task 1](https://bitbucket.org/osrf/srcsim/wiki/finals_task1)

* **Checkpoint 1**: Move within 1 meter from the communication dish
* **Checkpoint 2**: Move the communication dish **either** to the correct pitch or the correct yaw angle
* **Checkpoint 3**: Move the communication dish to **both** correct pitch and yaw angles
* **Checkpoint 4**: Walk into Task 1's finish box

### [Task 2](https://bitbucket.org/osrf/srcsim/wiki/finals_task2)

* **Checkpoint 1**: Retrieve the solar panel from the rover
* **Checkpoint 2**: Place the solar panel close to the power cable
* **Checkpoint 3**: Press the button on the solar panel
* **Checkpoint 4**: Pick up the power cable
* **Checkpoint 5**: Plug the power cable into the solar panel
* **Checkpoint 6**: Walk into Task 2's finish box

### [Task 3](https://bitbucket.org/osrf/srcsim/wiki/finals_task3)

* **Checkpoint 1**: Climb the stairs
* **Checkpoint 2**: Open the door
* **Checkpoint 3**: Pass through the door
* **Checkpoint 4**: Pick up the leak detector
* **Checkpoint 5**: Find the leak
* **Checkpoint 6**: Pick up the leak repair tool
* **Checkpoint 7**: Repair the leak
* **Checkpoint 8**: Walk into Task 3's finish box

## Quickstart

1. Run a world which contains all tasks:

        source /opt/nasa/indigo/setup.bash
        roslaunch srcsim unique.launch init:="true"


1. Start the first checkpoint of the first task:

    ```
    rosservice call /srcsim/finals/start_task 1 1
    ```

    * The service arguments correspond to the task id and the checkpoint id
    respectively.

    * When skipping to a checkpoint, the robot will be placed in a way as if
      it had completed the previous checkpoint.

    * Once a checkpoint has been completed, the next checkpoint is available.

    * Similarly, once a task has been completed, the next task is available.
    Beware that each task has a timeout.

    * Teams may choose to skip checkpoints (and lose points). At any time, you may
    skip to a checkpoint of a specific task using the service above. It's not
    possible to go back to an earlier checkpoint.

1. Listen to task updates:

    ```
    rostopic echo /srcsim/finals/task
    ```

    * Once a task starts, you will get information about its progress on this
    topic.

Click on a task above to see specific instructions for each task.

### Other resources

* [API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)
* [Skipping checkpoints](https://bitbucket.org/osrf/srcsim/wiki/skip_summary)
* [Practice versus competition](https://bitbucket.org/osrf/srcsim/wiki/practice_vs_competition)
* [Random world generator](https://bitbucket.org/osrf/srcsim/wiki/world_generator)
