# Overview

SRC qualification 1 is an image processing task. The qualification loads with R5 standing in front of a console. Located on the console are a number of LEDs and screens, all of which are black. The large screen in the center of the console transitions from yellow to blue. This transition marks the start of the task.

One at a time, LEDs will turn on (red) then off after the center screen's transition from blue to white. An LED will stay on for five seconds. During this time, you must report the center location of the LED relative to R5's head frame. Score is based on the accuracy of the reported locations.

Yellow Console | Blue Console | Red LED
---------------|--------------|--------
![srcsim_qual1_console_yellow.jpg](https://bitbucket.org/repo/xEbAAe/images/4007635085-srcsim_qual1_console_yellow.jpg) | ![srcsim_qual1_console_blue.jpg](https://bitbucket.org/repo/xEbAAe/images/4006639120-srcsim_qual1_console_blue.jpg) | ![srcsim_qual1_console_red.jpg](https://bitbucket.org/repo/xEbAAe/images/1513381160-srcsim_qual1_console_red.jpg)

The end of the task is marked by the center screen transitioning from white to blue to black. At this point, no more LEDs will turn on, and it is safe to quite Gazebo and submit your results.