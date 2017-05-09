Here you'll find instructions on how to use [CloudSim](https://cloudsim.io) for practice and for the competition.

**NOT YET RELEASED**

# Constellation structure

![src_cloud.png](https://bitbucket.org/repo/xEbAAe/images/2411928160-src_cloud.png)

Teams will use the CloudSim interface to launch a "constellation" of machines on Amazon Web Services for practice and for the final competition, as depicted above.

* The dark gray area delimits what lives within AWS.
* Each light grey area represents an AWS instance / S3 bucket:
    * The **Field Computer** is an EC2 instance (probably g2.2xlarge). Teams will have SSH access during practice, and traffic-controlled VPN access during the competition.
    * The **Simulator** is another EC2 instance (TBD). Teams will have SSH access during practice and no access during the competition.
    * The **S3 bucket** is where log files will be stored for scoring. Each team will be given read access to their own bucket.
* Teams are able to run code in each blue area:
    * The **team code** inside the field computer has direct access to the IHMC and SRCSim ROS topics on Gazebo.
    * The **Operator Control Unit (OCU)** is connected to the team code in the field computer through a traffic-controlled VPN network.
* All the other areas (green):
    * **Gazebo**: Runs the Gazebo server (gzserver), the robot controller and the competition plugins
    * **Firewall**: limits traffic between Gazebo and team code so Gazebo topics and some ROS topics are not available.
    * **Traffic shaper**: Limits traffic between Team Code and OCU
    * **Data log**: saves competition logs during the finals and uploads it to an S3 bucket once the simulation is over.

# One-time setup

To use CloudSim, you'll need to:

1. Create a [CloudSim](https://cloudsim.io) account
1. Host your field computer code on a [GitHub](https://github.com/) repository (public or private)

### Sign-up into CloudSim

1. Sign up with a Google account at https://cloudsim.io
1. Email `cloudsim@osrfoundation.org` to request access, stating:
    * the email you used to sign up 
    * the name of your team. 
1. Wait to hear confirmation, then log out and back in.

> Only the first user of a team needs to send an email to OSRF. After one user has been added, they can add other CloudSim users to the team as long as they've created accounts.

### Github repository

#### Repository structure

TODO: Explain the required repository structure and offer an example repo

#### Deploy key

If your software is held in a private Github repository, you'll need to provide us a Github deploy key. See [this Github help page](https://developer.github.com/guides/managing-deploy-keys/#deploy-keys) for more information on deploy keys. You do not need to provide a Github deploy key if you are using a public repository.

To setup the deploy key: 

1. On your machine, run 

        ssh-keygen -t rsa -b 4096

      **Leave the passphrase empty**

1. Import the public key in your Github repo by going to your repository, selecting `Settings` > `Deploy Keys` and clicking on `Add deploy key` to add the public key that you just generated.

Once that's done, all CloudSim needs is the private key to be able to clone your repository and deploy it in the field computer instance.

# Practice instructions

![cstutorial.png](https://bitbucket.org/repo/xEbAAe/images/3141748960-cstutorial.png)

### Login to cloudsim.io

1. Log in to https://cloudsim.io
1. Go to the SRC Competition Kiosk

### Start an SRC constellation

1. Click on the round gray button (with a + inside) to start a new constellation. You must supply:

    * the Github repository URI containing your code, for example (**Note the .git in the end**):
        * public repo: `https://github.com/username/reponame.git` (use https://)
        * private repo: `git://github.com/username/reponame.git` (use git://)
    * the Github deploy private key for that repo (private repos only). See the "One-time setup" section above to learn how to set up the deploy key. 

1. Click on continue to launch the machine instances on the cloud.

1. A new tab in the application will come up showing the two instances: the Simulator (top), and the Field Computer (bottom).

1. **Important:** You must wait until both machines are provisioned before using them. This might take a few minutes.
    1. Wait for the instance's public IP to show up on the interface
    1. Once they show up, click on both of them, this opens 2 new tabs on your browser
    1. At first, nothing will show on the tab. Refresh those tabs from time to time until you see something like the image below, indicating that the instance is up and running.

        ![cssimsim.png](https://bitbucket.org/repo/xEbAAe/images/3281358634-cssimsim.png)

1. In the Simulator instance, there is a `Run` widget that allows you to run 5 different SRC worlds. Click on the `>1` button to launch the first world inside the Simulator instance. **Note: Currenlty only >1 is active for practice. We'll add the 4 other different SRC worlds soon**

    > During practice, you can ssh into the sim instance to monitor the simulation process. The `>1` button launches a docker container with srcsim. So inside the sim machine instance, you can type `docker ps` to make sure the process has been started. You can also execute an interactive bash shell on the running docker container by doing `docker exec -it gazebo_run bash`

### Launch your software on field computer

The Field Computer instance is where your code will be deployed. The `Run` widget has a `START` button, that once triggered, will clone your repo, build the docker image as per your Dockerfile, and run the docker container. 

See the Field Computer Docker Example section for an example of what a Dockerfile looks like.

Click on the `START` button to launch your software. 

> During practice, you can ssh into the field computer instance to check if your docker image has been built successfully. In the terminal, typing `docker images` should show a `ros` image and a `fcomputer` image. If not, please be patient and wait a couple of minutes. Once ready, pressing the `START` button will run a container, called `team_container` , based on the `fccomputer` image.


### VPN Access

You can setup a VPN connection with the field computer by downloading the VPN keys available in the Field computer instance widget. Extract the files and start `openvpn`:

~~~
tar xvzf client_vpn.tar.gz
sudo openvpn --config openvpn.conf
~~~

You should now have a `tap0` network interface with the ip address `192.168.2.150`. The field computer is located at `192.168.2.10`.

### SSH Access

During practice, you'll be able to download the ssh keys for both instances by clicking on the `SSH` button in the `Practice mode` widget. Unzip the file and ssh in as the `ubuntu` user, e.g.

~~~ 
unzip keys.zip
ssh -i cloudsim.pem ubuntu@55.55.55.222
~~~

### Stop the Round

Once you're done with the round:

1. Click on the `STOP` button in the Simulator instance to stop the simulation
1. Click on the `STOP` button in the Field computer instance to stop the docker container running your code.

### Team budget

Each team will have a limited budget of hours that can be used on CloudSim for practice. 

* You can check your budget by clicking on your username on the top right corner.
* Each constellation consists of 2 machine instances (simulator and field computer), so the hours are subtracted in pairs.
    i.e. Running a constellation up for 1 hour will deduct 2 hours from your budget.
* Teams are able to launch as many constellations at the same time as they wish. But note that this is being taken from your budget.
* There are no partial hours. So having one machine instance up for 5 minutes counts as 1 hour. Likewise, having the machine instance up for 65 minutes will count as 2 hours. Use your time wisely.


### Field Computer Docker Example 

Here's a github repo with an example Dockerfile that shows the necessary dependencies to communicate with the simulator.

https://github.com/scpeters/src_field_computer_test

This simple example currently doesn't do much when the container is launched. A script is available inside the docker container that, when executed, will repeatedly play back a rosbag file which makes the robot move its footsteps. 

The docker image is built when launching the field computer. So if you have many packages to install or software that need to be built from source, please give it some time for the docker image to be ready before pressing the `START` button on the field computer instance to run the docker container. 

Note that the `ROS_MASTER_URI` and `ROS_IP` environment variables are already be set for you so you should be able to see all the ROS topics inside the container.