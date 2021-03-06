# Final's Schedule

* Start: June 13, 7am PDT
* End: June 16, 11:59pm PDT
* Goal: Complete all competition rounds
* Help line: ```src-support@osrfoundation.org```

OSRF will have personnel monitoring the support email 24/7 during the finals.

# How to complete competition rounds

1. Log into [CloudSim](https://cloudsim.io).

1. Request a constellation by sending an email to `src-support@osrfoundation.org`. The email must include your team name and the rounds you want to complete. Additionally, the email must come from the team lead. Here is an example email:

        Constellation request

        Team: Foo
        Rounds: 1, 2, 3
        Git URL: ...
        Git keys (for private repos): ...

1. Wait for the constellation to spin up in CloudSim.

1. Download your VPN keys, and establish you VPN connection.

1. In CloudSim, click the Play button for the first round.

1. Complete the active round.

1. Click the Stop button in Cloudsim.

1. Repeat for the other rounds in the constellation.

**Notes**

1. You can request multiple constellations if you want to run different rounds in parallel.
1. You must start using a constellation within an hour of requesting the constellation, otherwise it will be terminated
1. You can only complete each round once.
1. Logs are generated for each of your runs. These logs are available on S3 buckets using the AWS keys that have been provided to you.

# How to get help and request a reset

Send an email to ```src-support@osrfoundation.org``` with as much detail pertaining to your problem. If requesting a reset, please include your team name, round number, and a brief description of the problem that occurred.