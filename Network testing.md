This page describes how to use `tc`, a tool that can control latency, and `iperf` to test `tc`.

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