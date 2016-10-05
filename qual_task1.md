# Overview

SRC qualification 1 is an image processing task. The qualification loads with R5 standing in front of a console. Located on the console are a number of LEDs and screens, all of which are black. The large screen in the center of the console transitions from yellow to blue. This transition marks the start of the task.

One at a time, LEDs will turn on (red) then off after the center screen's transition from blue to white. An LED will stay on for five seconds. During this time, you must report the center location of the LED relative to R5's head frame. Score is based on the accuracy of the reported locations.

Yellow Console | Blue Console | Red LED
---------------|--------------|--------
![srcsim_qual1_console_yellow.jpg](https://bitbucket.org/repo/xEbAAe/images/4007635085-srcsim_qual1_console_yellow.jpg) | ![srcsim_qual1_console_blue.jpg](https://bitbucket.org/repo/xEbAAe/images/4006639120-srcsim_qual1_console_blue.jpg) | ![srcsim_qual1_console_red.jpg](https://bitbucket.org/repo/xEbAAe/images/1513381160-srcsim_qual1_console_red.jpg)

The end of the task is marked by the center screen transitioning from white to blue to black. At this point, no more LEDs will turn on, and it is safe to quite Gazebo and submit your results.

## 2D Image Processing

The first step is to find a red LED in a camera image. Camera data is available on the `/TBD` ROS topic. Subscribe to this topic by registering a callback. The callback will receive camera image data when the data is available.

The following snippet is an example subscription and callback. Note that this is for demonstration purposes only, and will not directly compile. Refer to the [ROS Publisher Subscriber Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) for more details on nodes, publishers, and subscribers.

```
void imageCallback(const sensor_msgs::ImageConstPtr &_msg)
{
  // Process data in the image message
}

ros::Subscriber sub = nodeHandle.subscribe("/TBD", 1000, imageCallback);
```

Make use of any image processing library, such as [OpenCV](http://opencv.org), to determine if an LED is on and where in the image the LED is located.

If we were to use OpenCV, the `imageCallback` function might look like following.

```
void imageCallback(const sensor_msgs::ImageConstPtr &_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &_e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to HSV
  cv::Mat hsv_image;
  cv:cvtColor(cv_ptr, hsv_image, cv::COLOR_BGR2HSV);

  // Threshold the HSV image to keep only red pixels
  cv::Mat lower_red_hue_range;
  cv::Mat upper_red_hue_range;
  cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
  cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

  // Combine the above two images
  cv::Mat red_hue_image;
  cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

  // Use the Hough transform to detect circles in the combined threshold image
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

  // Loop over all detected circles and outline them on the original image
  if(circles.size() == 0) std::exit(-1);
  for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
  {
    cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
    int radius = std::round(circles[current_circle][2]);
    
    std::cout << "Red LED located at " << center.x << " " << center.y << "\n";
  }
}

```

## Depth data

The previous step acquired the location of a red LED in 2D image space. Ultimately we need a 3D pose, which requires depth information. We can use either the stereo cameras on R5 or the spinning LIDAR to access depth information. This example will make use stereo camera data.

First, subscribe to the stereo camera data on the `/multisense_sl/camera/points` topic.

```
void stereoCallback(const sensor_msgs::PointCloud2ConstPtr &_msg)
{
}

ros::Subscriber stereoSub = nodeHandle.subscribe("/multisense/organized_image_points2", 1000, stereoCallback);
```

We can look up depth information using the `_msg.data` array inside the `stereoCallback` function. We are making an assumption that depth data exists for every pixel location.

```
void stereoCallback(const sensor_msgs::PointCloud2ConstPtr &_msg)
{
  int ledXPixelLocation = 100;
  int leftYPixelLocation = 200;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  pcl::fromROSMsg(*_msg, cloud);
  pcl::PointXYZ pt = cloud.at(ledYPixelLocation, ledXPixelLocation);

  std::cout << "The 3D location of LED[" << ledXPixelLocation << " " << ledYPixelLocation << "] is "
    << pt.x << " " << pt.y << " " << pt.z << std::endl;
}

```

## Reporting LED location

This qualification task requires us to upload a file that contains the 3D locations of each LED. The file to upload is generated for us based on data published to the `/src/qual1/led` topic. This topic expects a ROS `geometry_msgs/Vector3` message.

We will now create an appropriate message from the 3D location data, and publish this message on the `/src/qual1/led` topic.


```
ros::Publisher pub = nodeHandle.Advertise<geometry_msgs::Vector3>("/src/qual1/led");

msg.x = pt.x;
msg.y = pt.y;
msg.z = pt.z;

pub.Publish(msg);
```

## Upload your log file