# Setting up a system to run SRCSim #

This tutorial will walk you through the setup required to make a computer ready to run SRCSim. For more information about recommended and minimum requirements see the [System Requirements](https://bitbucket.org/osrf/srcsim/wiki/system_requirements) page.

## Binary installation

* Install the Gazebo package repository

    1. Add the Gazebo7 repository. Open a new terminal and type:

        ```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        ```

    1. Download the signing key

        ```
wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
        ```

* Install the SRCSim package repository

    1. Add the SRCSim repository

        ```
sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" \
    > /etc/apt/sources.list.d/src-latest.list'
        ```

    1. Download the signing keys

        ```
wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
        ```

        ```
wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
        ```

* Update the `apt` database

```
sudo apt-get update
```

* Install the SRCSim package and all its dependencies

```
sudo apt-get install srcsim
```

* Update your `JAVA_HOME` environment variable

```
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
```

* Add `IS_GAZEBO` environment variable:

```
echo 'export IS_GAZEBO=true' >> ~/.bashrc
```


* Change ownership of `ihmc_ros_java_adapter`. This ROS package requires to write some files in its installation directory at runtime. We're working on a fix for this issue. In the meantime, please change the ownership of this directory to your user.

```
sudo chown -R $USER:$USER /opt/ros/indigo/share/ihmc_ros_java_adapter
```

* Copy the IHMC networking `ini` file 


```
#!c++

mkdir -p ${HOME}/.ihmc
curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini
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

**Logout from your current session and log in to make sure that all these changes are in place.**

* Download all the required Gazebo models

```
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
rm /tmp/default.tar.gz
```

* Pre-build `ihmc_ros_java_adapter`. Open a new terminal and run:

```
roslaunch ihmc_valkyrie_ros valkyrie_warmup_graddle_cache.launch
```

## Test your installation

* Open a new terminal and type:

```
source /opt/nasa/indigo/setup.bash
roslaunch srcsim qual2.launch init:=true walk_test:=true
```

You should see your robot appear into the Gazebo scene. After a few seconds, the robot should approach the ground, switch to high level control, detach from the virtual harness and start walking.

Here's a [video](https://vimeo.com/188873182) of the expected behavior.