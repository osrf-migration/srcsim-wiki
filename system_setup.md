# Setting up a system to run SRCSIM #

This tutorial will walk you through the setup required to make a computer ready to run SRCSIM.
In order to run SRCSIM your computer will need a discrete graphics card and will need to be running Ubuntu Desktop 14.04 Trusty.
It is possible to run on a different OS and OS version, but it is not recommended.
For more information about recommended and minimum requirements see the [System Requirements](https://bitbucket.org/osrf/srcsim/wiki/system_requirements) page.

## Installing Gazebo7 ##

SRCSIM requires Gazebo7 to be installed, for which instructions can be found here:

http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Alternativeinstallation:step-by-step

The "alternative installation" steps in the above link will be summarized here.
You could use the "one-liner" installation, but it would not guarantee that you get Gazebo7.

### Setup Ubuntu to Install Packages from the Open Source Robotics Foundation (OSRF) ###

First you need to add OSRF's package repository to your system so it can install packages from there.
Do so by running this command:


```
sudo sh -c '
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
    > /etc/apt/sources.list.d/gazebo-stable.list'
```

Next you need to add the signing key to your computer so the packages can be verified:

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### Installing Gazebo ###

First update the `apt-get` database since you added the OSRF packages repository to the sources list:

```
sudo apt-get update
```

Finally, install `gazebo7`:

```
sudo apt-get install gazebo7
```

## Installing ROS Indigo ##

Next you need to install ROS Indigo, which SRCSIM uses to provide the competition interface.
Instructions to do this can be found here:

http://wiki.ros.org/indigo/Installation/Ubuntu

These instructions will be summarized next.

### Setup Ubuntu to Install Packages from ROS ###

First add the ROS packages repository to `apt-get`'s list of sources:

```
sudo sh -c '
  echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list'
```

Then add the key ROS uses to sign the packages:

```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
```

### Installing ROS Desktop ###

ROS has a few "variants" which contain commonly used components, e.g. "Robot" is a small set of things that you'd run on a deployed robot, "Desktop" is a superset of "Robot" that also contains tools you might use on a workstation, and "Desktop-Full" additionally has perception algorithms and simulators.
Normally ROS Indigo uses Gazebo2, but since SRCSIM uses Gazebo7 you will need to avoid installing "Desktop-Full" and instead install the "Desktop" set of packages and then manually install the Gazebo7 version of the Gazebo-ROS compatibility packages afterwards.

First update `apt-get`'s database since you added the ROS repositories to the list of sources:

```
sudo apt-get update
```

Then install the "Desktop" packages for ROS Indigo:

```
sudo apt-get install ros-indigo-desktop
```

To finish the ROS Indigo installation, initialize the `rosdep` database:

```
sudo rosdep init; rosdep update
```

### Installing Gazebo-ROS Compatibility Packages ###

Now install the Gazebo7 version of the Gazebo-ROS compatibility packages:

```
sudo apt-get install ros-indigo-gazebo7-ros-pkgs
```

## Installing the SRCSIM packages from Binaries ##

Not available yet.

## Alternative: Building the SRCSIM packages from Source ##

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
