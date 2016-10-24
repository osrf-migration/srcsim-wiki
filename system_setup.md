# Setting up a system to run SRCSIM #

This tutorial will walk you through the setup required to make a computer ready to run SRCSIM. For more information about recommended and minimum requirements see the [System Requirements](https://bitbucket.org/osrf/srcsim/wiki/system_requirements) page.

## Install Dependencies


1. Install Gazebo7

    1. Add the Gazebo7 repository

        ```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        ```

    1. Download the signing key

        ```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        ```

    1. Update the apt database

        ```
sudo apt-get update
        ```

    1. Finally, install gazebo7

        ```
sudo apt-get install gazebo7
        ```

1. Install ROS Indigo

    1. Add the ROS repository

        ```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list'
        ```

    1. Download the signing key

        ```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
        ```

    1. Update the apt database

        ```
sudo apt-get update
        ```

    1. Install the "Desktop" packages for ROS Indigo

        ```
sudo apt-get install ros-indigo-desktop
        ```

    1. Initialize the rosdep database

        ```
sudo rosdep init; rosdep update
        ```

    1. Install the Gazebo7 version of the Gazebo-ROS compatibility packages

        ```
sudo apt-get install ros-indigo-gazebo7-ros-pkgs
        ```

## Install SRC Sim

### Binary install


1. Add the SRCSim packages repository

    ```
sudo sh -c 'echo "deb http://52.53.157.231/src trusty main" > /etc/apt/sources.list.d/srcsim.list'
    ```

1. Add the key SRCSim uses to sign the packages:

    ```
wget -O - http://52.53.157.231/src/src.key | sudo apt-key add -
    ```

### Source Install

This strategy is only recommended for a few cases:

- You want to test a change you're going to propose to the SRCSIM source code.
- There are no SRCSIM binaries available for your system.

1. Set up a Catkin Workspace

    ```
mkdir -p ~/srcsim_ws/src
cd ~/srcsim_ws
    ```

1. Get the SRC Sim source code from bitbucket.

    ```
cd ~/srcsim_ws/src
hg clone https://bitbucket.org/osrf/srcsim
    ```

1. Before starting the build, you need to source the ROS `setup.bash` file:

    ```
source /opt/ros/indigo/setup.bash
    ```

1. Start the build using the catkin build tool

    ```
cd ~/srcsim_ws
catkin_make install
    ```

1. Source the installed setup bash file before building your competition solution and running srcsim

      ``` 
source ~/srcsim_ws/install/setup.bash
    ```