Here you'll find instructions on how to use [CloudSim](https://cloudsim.io) for practice and for the competition.

**NOT YET RELEASED**

# One-time setup

To use CloudSim, you'll need to:

1. Create a [CloudSim](https://cloudsim.io) account
1. Host your field computer code on a GitHub repository (public or private)

### Sign-up into CloudSim

1. Sign up with a Google account at https://cloudsim.io
1. Email `cloudsim@osrfoundation.org` to request access, stating:
    * the email you used to sign up 
    * the name of your team. 
1. Wait to hear confirmation, then log out and back in.

> Only the first user of a team needs to send an email to OSRF. After one user has been added, they can add other CloudSim users to the team as long as they've created accounts.

### Github deploy key

If your software is held in a private Github repository, you'll need to provide us a Github deploy key. See [this Github help page](https://developer.github.com/guides/managing-deploy-keys/#deploy-keys) for more information on deploy keys. You do not need to provide a Github deploy key if you are using a public repository.

To setup the deploy key: 

1. On your machine, run 

        ssh-keygen -t rsa -b 4096

    Enter the passphrase or leave it empty

1. Import the public key in your Github repo by going to your repository, selecting `Settings` > `Deploy Keys` and clicking on `Add deploy key` to add the public key that you just generated.

Once that's done, all CloudSim needs is the private key to be able to clone your repository and deploy it in the field computer instance.

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

# Practice instructions

### Login to cloudsim.io

1. Log in to https://cloudsim.io
1. Click on the SRC link (on the left menu bar, under the Dashboard)

### Create an SRC constellation

Click on the round button (with a + inside) to create a new constellation. You must supply:

* the Github repository URI containing your code, for example `git://github.com/username/reponame.git`
* the Github deploy private key for that repo (private repos only). See the "One-time setup" section above to learn how to set up the deploy key. 

Click on continue to launch the machine instances on the cloud. 

### Start a round

There are two instances in a round: One for the Simulator (top), and the other for the Field Computer (bottom).

In the Simulator instance, there is a `Run` widget that allows you to run 5 different rounds of SRC simulation. Clicking on a numbered button here will launch the SRC simulator inside the Simulator instance. Before doing so, make sure the instances is up and running by clicking on the IP address link. A page will be displayed when the instance is ready, otherwise the link will not work.

Click on the `>1` button to launch the simulation. 

### Launch your software on Field computer

The Field Computer instance is where your code will be deployed. The `Run` widget has a `START` button, that once triggered, will clone your repo, build the docker image as per your Dockerfile, and run the docker container. 

Click on the `START` button to launch your software. 

### Stop the Round


Once you're done with the round, click on the `STOP` button in the Simulator instance to stop the simulation, and click on the `STOP` button in the Field computer instance to stop the docker container running your code.

### VPN Access

You can setup a VPN connection with the field computer by downloading the VPN keys available in the Field computer instance widget. Extract the files and start openvpn:

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