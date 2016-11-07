# Setting up a system to run SRCSim

This tutorial will walk you through the setup required to make a computer ready to run SRCSim. For more information about recommended and minimum requirements see the [System Requirements](https://bitbucket.org/osrf/srcsim/wiki/system_requirements) page.


1. Install the Gazebo package repository

    1. Add the Gazebo7 repository. Open a new terminal and type:

        ```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        ```

    1. Download the signing key

        ```
wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
        ```

1. Install the SRCSim package repository

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

1. Update the `apt` database

    ```
sudo apt-get update
    ```

1. Install the SRCSim package and all its dependencies

    ```
sudo apt-get install srcsim
    ```

1. Update your `JAVA_HOME` environment variable

    ```
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
    ```

1. Add `IS_GAZEBO` environment variable:

    ```
echo 'export IS_GAZEBO=true' >> ~/.bashrc
    ```


1. Change ownership of `ihmc_ros_java_adapter`. This ROS package requires to write some files in its installation directory at runtime. We're working on a fix for this issue. In the meantime, please change the ownership of this directory to your user.

    ```
sudo chown -R $USER:$USER /opt/ros/indigo/share/ihmc_ros_java_adapter
    ```

1. Copy the IHMC networking `ini` file 


    ```
mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini
    ```

1. Increase real-time scheduling priority for current user (rtprio), which is required by the IHMC controller. Add current user to ros group:

    ```
sudo bash -c 'echo "@ros    -       rtprio      99" > /etc/security/limits.d/ros-rtprio.conf'
    ```

    ```
sudo groupadd ros
    ```

    ```
sudo usermod -a -G ros $USER
    ```

    **Logout from your current session and log in to make sure that all these changes are in place.**

1. Download the deployed Valkyrie controller

    ```
wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
    ```

    ```
tar -xvf /tmp/valkyrie_controller.tar.gz -C $HOME
    ```

    ```
rm /tmp/valkyrie_controller.tar.gz
    ```

1. Download all the required Gazebo models

    ```
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
    ```

    ```
mkdir -p $HOME/.gazebo/models
    ```

    ```
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
    ```

    ```
rm /tmp/default.tar.gz
    ```

1. Pre-build `ihmc_ros_java_adapter`. Open a new terminal and run:

    ```
source /opt/nasa/indigo/setup.bash
    ```

    ```
roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch
    ```

## Test your installation

1. Open a new terminal and type:

    ```
source /opt/nasa/indigo/setup.bash
    ```

    ```
roslaunch srcsim qual2.launch init:=true walk_test:=true
    ```

You should see your robot appear into the Gazebo scene. After a few seconds, the robot should approach the ground, switch to high level control, detach from the virtual harness and start walking.

Here's a [video](https://vimeo.com/188873182) of the expected behavior.

Also, the following error message is often seen in the console. This is a severe warning when controlling a real robot but is expected when running a simulation on PC hardware.

~~~
[ERROR] [1478200728.329368274, 2.971000000]: Nov 03, 2016 7:18:48 PM us.ihmc.valkyrieRosControl.ValkyrieAffinity <init>
[ERROR] [1478200728.329423149, 2.971000000]: SEVERE: WARNING: Hyper-Threading is enabled. Expect higher amounts of jitter
~~~