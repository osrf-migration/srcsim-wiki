# Setting up a system to run SRCSim #

This tutorial will walk you through the setup required to make a computer ready to run SRCSim. For more information about recommended and minimum requirements see the [System Requirements](https://bitbucket.org/osrf/srcsim/wiki/system_requirements) page.

## Binary installation

* Install the Gazebo package repository

    1. Add the Gazebo7 repository

        ```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        ```

    1. Download the signing key

        ```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        ```

* Install the SRCSim package repository

    1. Add the SRCSim repository

        ```
sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" \
    > /etc/apt/sources.list.d/src-latest.list'
        ```

    1. Download the signing key

        ```
wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
        ```

* Update the `apt` database

```
sudo apt-get update
```

* Install the "SRCSim" package and all its dependencies

```
sudo apt-get install srcsim
```

* Initialize the rosdep database

```
sudo rosdep init; rosdep update
```

* Install Java 8
   
```
sudo apt-add-repository -y ppa:openjdk-r/ppa
sudo apt-get update
sudo apt-get install -y openjdk-8-jdk
```
 

* Update your `JAVA_HOME` environment variable

```
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
```

* Change ownership of `ihmc_ros_java_adapter`. This ROS package requires to write some files in its installation directory at runtime. We're working on a fix for this issue. In the meantime, please change the ownership of this directory to your user.

```
sudo chown -R $USER:$USER /opt/ros/indigo/share/ihmc_ros_java_adapter
```

* Create IHMC networking ini file in `/opt/ros/indigo/share/ihmc_valkeryie_ros/configurations/IHMCNetworkParameters.ini` with the following content (you will need to edit the file using `sudo`:


```
#!c++

rosURI=http\://localhost\:11311
logger=localhost
robotController=localhost
networkManager=localhost

#don't log any cameras
loggedCameras=
```

* Configure your users and groups with the following:

```
#!c++

sudo groupadd ros
sudo groupadd pgrimaging
sudo adduser vanguard
sudo usermod -a -G ros $USER
sudo usermod -a -G dialout $USER
sudo usermod -a -G pgrimaging $USER
sudo usermod -a -G sudo vanguard
sudo usermod -a -G ros vanguard 
```

* Download all the required Gazebo models

```
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
rm /tmp/default.tar.gz
```

* Pre-build ihmc_ros_java_adapter

```
roslaunch ihmc_valkyrie_ros valkyrie_warmup_graddle_cache.launch
```

## Test your installation

* Open a new terminal and type:

```
roslaunch srcsim qual2.launch init:=true walk_test:=true
```

You should see your robot appear into the scene. After a few seconds, the robot should approach the ground, switch to high level control, detach from the virtual harness and start walking.

Here's a [video](https://vimeo.com/188873182) of the expected behavior.

