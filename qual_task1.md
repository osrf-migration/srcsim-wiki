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

## Computing LED location in head frame

The location of each LED must be reported in R5's head frame. In the previous section, you wrote code to find an LED in a camera image. This is a 2D location that can be mapped to a 3D location if we know the pose of the camera.

We will use [TF](http://wiki.ros.org/tf/Tutorials) to perform the mapping from 2D image location to 3D.location.