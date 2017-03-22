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

    *From time to time, an invalid world may be generated. This can be quickly
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

    *Obs: These commands will only set the environment variables for the current
     terminal. If you wish to set the variables for every new terminal, you can
     do:*

        echo "export USE_CUSTOM_WORLD=1" >> ~/.bashrc
        echo "export CUSTOM_WORLD_PATH=~/src_finals_worlds/unique.world" >> ~/.bashrc

1. Now you can use the usual launch file, and that will use the custom world:

        roslaunch srcsim unique.launch init:="true"
