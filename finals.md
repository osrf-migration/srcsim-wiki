# Overview

Each run consists of 3 tasks and each task is composed of a different number of
checkpoints:

### [Task 1](https://bitbucket.org/osrf/srcsim/wiki/finals_task1)

* **Checkpoint 1**: Move within 1 meter from the communication dish
* **Checkpoint 2**: Move the communication dish to the correct pitch and yaw angles (consists of 2 checkpoints for scoring purposes)
* **Checkpoint 3**: Walk into Task 1's finish box

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

    * Once a checkpoint has been completed, the next checkpoint is available.

    * Similarly, once a task has been completed, the next task is available.
    Beware that each task has a timeout.

    * Teams may choose to skip checkpoints (and lose points). At any time, you may
    skip to a checkpoint of a specific task using the service above. It's not
    possible to restart the current checkpoint or to go back to an earlier
    checkpoint.

1. Listen to task updates:

    ```
    rostopic echo /srcsim/finals/task
    ```

    * Once a task starts, you will get information about its progress on this
    topic.

Click on a task above to see specific instructions for each task.

For a complete list of the available interfaces, check the
[API documentation](https://bitbucket.org/osrf/srcsim/wiki/api)

## Random world generator

** Not released yet **

The worlds provided during practice are similar to the worlds which will be used
during the competition, but not exactly the same. It's a good idea to practice
with different world setups to be prepared for the finals. A handy script which
can be used to generate randomized worlds is provided for teams, you can use it
as follows:

1. Make sure you have a recent version of ruby installed (recommended > 2.2)

        ruby --version

1. In case you have an old version, you can upgrade as follows:

        sudo apt-get install software-properties-common python-software-properties
        sudo apt-add-repository ppa:brightbox/ruby-ng
        sudo apt-get update
        sudo apt-get install ruby2.2 ruby2.2-dev

1. Create a directory to hold the scripts and world files:

        mkdir ~/src_finals_worlds
        cd ~/src_finals_worlds

1. Copy the necessary scripts:

        wget https://bitbucket.org/osrf/srcsim/raw/default/worlds/unique.world.erb
        wget https://bitbucket.org/osrf/srcsim/raw/default/worlds/satellite.erb

1. Generate a world with all 3 tasks as follows. Each time this command is run,
   a different world will be generated.

        erb unique.world.erb > unique.world

    * From time to time, an invalid world may be generated. This can be quickly
      spotted by models constantly shaking or flying around. When this happens,
      just generate a new world.*

1. You can also generate worlds with each of the tasks separately, by passing
   the task number as an argument:

        erb t=1 unique.world.erb > unique_task1.world

1. To use one of the generated worlds, you must set the `USE_CUSTOM_WORLD`
   environment variable to true:

        export USE_CUSTOM_WORLD=1

1. You also need to set another environment variable with the full path to the
   world file you wish to use, for example:

        export CUSTOM_WORLD_PATH=~/src_finals_worlds/unique.world

1. Obs: These commands will only set the environment variables for the current
   terminal. If you wish to set the variables for every new terminal, you can
   do:

        echo "export USE_CUSTOM_WORLD=1" >> ~/.bashrc
        echo "export CUSTOM_WORLD_PATH=~/src_finals_worlds/unique.world" >> ~/.bashrc

1. Now you can use the usual launch file, and that will use the custom world:

        roslaunch srcsim unique.launch init:="true"


