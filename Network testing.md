This page describes how to use `tc`, a tool that can control latency and bandwidth, and `iperf` to test `tc`.

We will start with a simple example of `tc` and build to a full example that incorporates `cloudsim`.

# Step 1: Latency on local host

Let's start by trying to limit the bandwidth on our local machine's ethernet port. 

1. Find the name of your ethernet device, which is usually `eth0`, `eth1`, or something similar.

    * Run ifconfig to list your ethernet devices

        ```$ ifconfig```

    * You should see output similar to this


            eth0      Link encap:Ethernet  HWaddr 02:42:0a:00:00:02  
                      inet addr:10.0.0.2  Bcast:0.0.0.0  Mask:255.255.0.0
                      inet6 addr: fe80::42:aff:fe00:2/64 Scope:Link
                      UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
                      RX packets:108425 errors:0 dropped:0 overruns:0 frame:0
                      TX packets:54319 errors:0 dropped:0 overruns:0 carrier:0
                      collisions:0 txqueuelen:0 
                      RX bytes:116636572 (116.6 MB)  TX bytes:3959515 (3.9 MB)

            lo        Link encap:Local Loopback  
                      inet addr:127.0.0.1  Mask:255.0.0.0
                      inet6 addr: ::1/128 Scope:Host
                      UP LOOPBACK RUNNING  MTU:65536  Metric:1
                      RX packets:14375798 errors:0 dropped:0 overruns:0 frame:0
                      TX packets:14375798 errors:0 dropped:0 overruns:0 carrier:0
                      collisions:0 txqueuelen:1 
                      RX bytes:2534849336 (2.5 GB)  TX bytes:2534849336 (2.5 GB)

1. Apply a 500ms latency to your ethernet device (we will assume `eth0`) using `tc`

    ```
    sudo tc qdisc add dev eth0 root netem delay 500ms
    ```

1. Test the latency by running `ping google.com`. You should see a `time` that is roughly equivalent to 500 ms.

1. Make sure to clear your `tc` settings.

    ```
    sudo tc qdisc del dev eth0 root
    ```

# Step 2: Latency between local host and a local docker container

1. Run your favorite docker container. There are many helpful docker tutorials online.

1. Inside docker, run `iperf` server

    ```
    iperf -s
    ```

1. On your host system, run `iperf` client. We will assume that the docker container has an IP of 10.0.0.2 and your host has and IP 10.0.0.1. You can use `ifconfig` to determine your IP settings.

    ```
    iperf -c 10.0.0.2
    ```

1. You should see output on the server (docker) side similar to:


        ------------------------------------------------------------
        Server listening on TCP port 5001
        TCP window size: 85.3 KByte (default)
        ------------------------------------------------------------
        [  4] local 10.0.0.2 port 5001 connected with 10.0.0.1 port 58432
        [ ID] Interval       Transfer     Bandwidth
        [  4]  0.0-10.0 sec  52.4 GBytes  45.0 Gbits/sec

1. Now setup `tc` latency on your host system. Use `ifconfig` to determine the device name that maps to your docker network. In our case, the network device name is `br-f70d936ac68f`

    ```
    sudo tc qdisc add dev br-f70d936ac68f root netem delay 500ms
    ```

1. Re-run iperf to test this setting

    ```
    iperf -c 10.0.0.2
    ```

1. You should now see a reduction in bandwidth due to the increased latency

        [  5] local 10.0.0.2 port 5001 connected with 10.0.0.1 port 58550
        [  5]  0.0-17.5 sec  7.75 MBytes  3.71 Mbits/sec

# Step 3: Latency between local host and remote docker on CloudSim

1. Log into CloudSim and start a constellation.

1. Download the SSH keys to the Field Computer, and log into the field computer

    ```
    ssh -i cloudsim.pem ubuntu@FIELD_COMPUTER_IP_ADDRESS
    ```

1. Download the VPN keys, and establish your VPN connection to the field computer.

    ```
    sudo openvpn --config openvpn.config
    ````

1. Run add latency on the Field Computer using `tc`. We will apply latency control to the vpn device called `tap0`.

    ```
    sudo tc qdisc add dev tap0 root netem delay 500ms
    ```

1. Test using ping on your local computer

    ```
    ping 192.168.2.8
    ```

1. You can also test with `iperf`

    1. On the field computer: ```iperf -s```
    1. On your local computer: ```iperf -c 192.168.2.8```