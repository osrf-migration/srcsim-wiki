Here you'll find instructions on how to use CloudSim for practice and for the competition.

# One-time setup

To use CloudSim, you'll need to:

1. Create a CloudSim account
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



# Login to cloudsim.io

1. Log in to https://cloudsim.io
1. Click on the SRC link (on the left menu bar, under the Dashboard)

# Create an SRC round

SRC machines are grouped into Rounds. Click on the round button (with a + inside) to create a new round. You must supply:

* the Github repository URI containing your code, for example `git://github.com/username/reponame.git`
* the Github deploy key for that repo (private repos only). See the "One-time setup" section above to learn how to set up the deploy key. 

Click on continue to launch the machine instances on the cloud. 

# Start a round

There are two instances in a round: One for the Simulator (top), and the other for the Field Computer (bottom).

In the Simulator instance, there is a `Run` widget that allows you to run 5 different rounds of SRC simulation. Clicking on a numbered button here will launch the SRC simulator inside the Simulator instance. Before doing so, make sure the instances is up and running by clicking on the IP address link. A page will be displayed when the instance is ready, otherwise the link will not work.

Click on the `>1` button to launch the simulation. 

Launch your software on Field computer
==

The Field Computer instance is where your code will be deployed. The `Run` widget has a `START` button, that once triggered, will clone your repo, build the docker image as per your Dockerfile, and run the docker container. 

Click on the `START` button to launch your software. 

Stop the Round
==

Once you're done with the round, click on the `STOP` button in the Simulator instance to stop the simulation, and click on the `STOP` button in the Field computer instance to stop the docker container running your code.


SSH Access
===

During practice, you'll be able to download the ssh keys for both instances by clicking on the `SSH` button in the `Practice mode` widget. Unzip the file and ssh in as the `ubuntu` user, e.g.

~~~ 
ssh -i cloudsim.pem ubuntu@55.55.55.222
~~~