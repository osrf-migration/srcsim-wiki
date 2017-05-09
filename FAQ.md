##Final Competition Process

Q. Teams have 24 hours to complete a run. Must those 24 hours be contiguous?

No.

Q. Can we complete competition runs at anytime time of day?

Yes

Q. Can multiple competition runs be completed simultaneously?

Yes

Q. Will the final competition be monitored?

OSRF staff will be on hand during the final competition.

Q. What do we do if we have a problem with the simulation?

Contact OSRF. Contact information will be provided prior to the competition.

Q. If a software crash occurs during competition will there be an avenue to request a rerun?

According the rules document, “ runs ended by software process crashes will not count if the crash occurred for reasons outside team control. If this type of event occurs, a team should contact the competition administrators, who will evaluate the situation and reset the competition from the beginning.” OSRF contact information will be provided prior to the competition.

Q. Regarding the judging for "style," does this mean Val should twirl her tools before shoving them in her holster? Or does it just mean we should avoid shortcuts which might not be realistic on the hardware?

According to the rules, “Subjective scoring will look for realism of R5 motions (i.e. not taking advantage of simulation physics), limiting of intentional damage to R5 and the environment to complete tasks successfully”

## Bandwidth, latency and Timing

Q. Will the latency be changed during a run?

No

Q. Are bandwidth limits in simulation time or wall time?

Wall time.

Q. Can we get some example tc commands that we can use to test the latency and bandwidth limits?

Issue #125 on Bitbucket has an example script.

Q. Is bandwidth limited or is a bucket-of-bits used instead?

According to the rules, bandwidth is limited.

Q. Do you know what the expected real time factor is going to be?

Not yet.

## Cloud Simulation

Q. Will we have permission to run custom scripts, and import software on the field computer?

Each team will have the ability to install and configure the field computer.

Q. Will the organizers alter the field computer in any way?

No

Q. Can we use ConstructSim, and are there any export control issues to be concerned about?

Construct Sim is not affiliated with the Space Robotics Challenge. The code released by OSRF is open source, and can be used accordingly.

Q. Is there an AMI for running srcsim in a docker container?

ami-20133540

## World Configuration and R5 Control

Q. The software is not ready for this Challenge. Additionally, current software release does not provide a stable grip for the robot.

All the tasks are available, grasping should be working, and CloudSim is available to teams.

Q. Does the R5 need to remain on the walkway, or is it permitted to walk on the terrain?

R5 does not need to stay on the walkway.

Q. Will there be anything that is randomized in the finals but currently not randomized by the world generator tool we have for practice?

All teams will compete in an identical world configuration.

Q. Which point of the hand is being aligned by the hand trajectory command? How do we access the hand frame?

Teams will have to do their own end point planning to control the hand.

Q. Will the slope of the desk with the wheels on change? Will the knob on the wheel be always in the same starting position?

The practice worlds are examples of what the final competition will look like, but the exact configuration will not be revealed ahead of time.

Q. Structure of final competition with regard to what software running is not clear. Have stated the “world” frame will not be available for competition but haven’t indicated the substitute. 

Please refer to issue #106 on bitbucket. This question is related to a misconfigured network, not the lack of a world frame.

Q. When R5 resets will she be standing up, or restarted in harness and lowered?

R5 will be re-harnessed and lowered.

Q. There are several topics that do not provide data. Can we get a list of topics that will be providing good data for us to use? Or a list of topics that we should not use or that may not be working at the competition? 

Topic use is left up to the decision of each team. There is no concept of a “good” topic.

Q.  Once initialized and lowered, it looks like R5 could, proceed through the checkpoints without intervention. Will you be doing the initialization and lowering, or do those commands need to come from the OCU? Is any other intervention or communication strictly REQUIRED with R5 or with the organizers?

Control of R5, outside of the controllers provided by the organizers, is left up to the competitor. All commands can come from the field computer, from the OCU, or a combination.