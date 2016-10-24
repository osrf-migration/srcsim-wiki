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

### Setting Up a Catkin Workspace ###

Since the SRCSIM packages are [catkin](http://wiki.ros.org/catkin) packages, we'll start by setting up a [catkin workspace](http://www.ros.org/reps/rep-0128.html).
First create a folder for the workspace, something like this:

```
mkdir -p ~/srcsim_ws/src
cd ~/srcsim_ws
```

The `src` folder will contain the source code you want to build.

### Getting the Source Code ###

Next you will need to get the SRCSIM source code.
You can either "clone" the source using `mercurial` or you can extract an archive of the source that you downloaded from BitBucket.
To clone the source:

```
cd ~/srcsim_ws/src
hg clone https://bitbucket.org/osrf/srcsim
```

If you would prefer to download the source as an archive:

```
cd ~/srcsim_ws/src
wget https://bitbucket.org/osrf/srcsim/get/default.zip
unzip default.zip
rm default.zip
```

### Building the SRCSIM Packages ###

Before starting the build, you need to source the ROS `setup.bash` file:

```
source /opt/ros/indigo/setup.bash
```

This sets up your shell to build on top of ROS.
There is also a `setup.zsh` if you are using that shell.

Next start the build using the `catkin` build tool called `catkin_make`:

```
cd ~/srcsim_ws
catkin_make install
```

This will build the SRCSIM packages and install them to a local folder, in this case `~/srcsim_ws/install` next to the `src` folder.

This folder will also have a `setup.bash` file in it that you can source before building your competition packages:

``` 
source ~/srcsim_ws/install/setup.bash

```
