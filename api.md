# SRC API

Competitors in the Space Robotics Challenge are expected to write a [ROS](ros.org) program to solve the challenge tasks. This page details information about the various ROS methods that a competitors may use to collect state information (eg. sensor data, joint states) and send control commands (e.g. foot placement).

Not listed on this page are standard ROS topics, such as `/tf`, which are not unique to the Space Robotics Challenge. If this is confusing, then a good place to start are the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials).

## ROS Services

## ROS Topics

The format of each item in the following list is `topic name` : `message type`.

* `/ihmc_ros/localization/pelvis_odom_pose_correction` : [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `/ihmc_ros/localization/pelvis_pose_correction` : [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
* `/ihmc_ros/valkyrie/control/abort_walking` : [ihmc_msgs/AbortWalkingRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/AbortWalkingRosMessage.msg)
* `/ihmc_ros/valkyrie/control/arm_desired_joint_accelerations` : [ihmc_msgs/ArmDesiredAccelerationsRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/ArmDesiredAccelerationsRosMessage.msg)
* `/ihmc_ros/valkyrie/control/arm_trajectory` : [ihmc_msgs/ArmTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/ArmTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/chest_trajectory` : [ihmc_msgs/ChestTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/ChestTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/end_effector_load_bearing` : [ihmc_msgs/EndEffectorLoadBearingRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/EndEffectorLoadBearingRosMessage.msg)
* `/ihmc_ros/valkyrie/control/foot_trajectory` : [ihmc_msgs/FootTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/footstep_list` : [ihmc_msgs/FootstepDataListRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootstepDataListRosMessage.msg)
* `/ihmc_ros/valkyrie/control/go_home` : [ihmc_msgs/GoHomeRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootstepDataListRosMessage.msg)
* `/ihmc_ros/valkyrie/control/hand_desired_configuration` : [ihmc_msgs/HandDesiredConfigurationRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootstepDataListRosMessage.msg)
* `/ihmc_ros/valkyrie/control/hand_trajectory` : [ihmc_msgs/HandTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootstepDataListRosMessage.msg)
* `/ihmc_ros/valkyrie/control/head_trajectory` : [ihmc_msgs/HeadTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/HeadTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/high_level_state` : [ihmc_msgs/HighLevelStateRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/HighLevelStateRosMessage.msg)
* `/ihmc_ros/valkyrie/control/low_level_control_mode` : [ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage](https://github.com/ihmcrobotics/ihmc_valkyrie_ros/blob/develop/msg/ValkyrieLowLevelControlModeRosMessage.msg)
* `/ihmc_ros/valkyrie/control/neck_desired_acceleration` : [ihmc_msgs/NeckDesiredAccelerationsRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/NeckDesiredAccelerationsRosMessage.msg)
* `/ihmc_ros/valkyrie/control/neck_trajectory` : [ihmc_msgs/NeckTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/NeckTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/pause_walking` : [ihmc_msgs/PauseWalkingRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/PauseWalkingRosMessage.msg)
* `/ihmc_ros/valkyrie/control/pelvis_height_trajectory` : [ihmc_msgs/PelvisHeightTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/PelvisHeightTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory` : [ihmc_msgs/PelvisOrientationTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/PelvisOrientationTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/pelvis_trajectory` : [ihmc_msgs/PelvisTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/PelvisTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/request_stop` : [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/ihmc_ros/valkyrie/control/stop_all_trajectories` : [ihmc_msgs/StopAllTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/StopAllTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/control/whole_body_trajectory` : [ihmc_msgs/WholeBodyTrajectoryRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/WholeBodyTrajectoryRosMessage.msg)
* `/ihmc_ros/valkyrie/output/behavior` : [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
* `/ihmc_ros/valkyrie/output/capturability/capture_point` : [ihmc_msgs/Point2dRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/Point2dRosMessage.msg)
* `/ihmc_ros/valkyrie/output/capturability/center_of_mass` : [geometry_msgs/Point32](http://docs.ros.org/api/geometry_msgs/html/msg/Point32.html)
* `/ihmc_ros/valkyrie/output/capturability/desired_capture_point` : [ihmc_msgs/Point2dRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/Point2dRosMessage.msg)
* `/ihmc_ros/valkyrie/output/capturability/is_in_double_support` : [std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)
* `/ihmc_ros/valkyrie/output/capturability/left_foot_support_polygon` : [ihmc_msgs/SupportPolygonRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/SupportPolygonRosMessage.msg)
* `/ihmc_ros/valkyrie/output/capturability/right_foot_support_polygon` : [ihmc_msgs/SupportPolygonRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/SupportPolygonRosMessage.msg)
* `/ihmc_ros/valkyrie/output/foot_force_sensor/left` : [geometry_msgs/WrenchStamped](http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html)
* `/ihmc_ros/valkyrie/output/foot_force_sensor/right` : [geometry_msgs/WrenchStamped](http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html)
* `/ihmc_ros/valkyrie/output/footstep_status` : [ihmc_msgs/FootstepStatusRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/FootstepStatusRosMessage.msg)
* `/ihmc_ros/valkyrie/output/high_level_state` : [ihmc_msgs/HighLevelStateRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/HighLevelStateRosMessage.msg)
* `/ihmc_ros/valkyrie/output/high_level_state_change` : [ihmc_msgs/HighLevelStateChangeStatusRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/HighLevelStateChangeStatusRosMessage.msg)
* `/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu` : [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* `/ihmc_ros/valkyrie/output/imu/pelvis_pelvisRearImu` : [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* `/ihmc_ros/valkyrie/output/imu/torso_leftTorsoImu` : [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* `/ihmc_ros/valkyrie/output/imu/upperNeckPitchLink_head_imu_sensor` : [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* `/ihmc_ros/valkyrie/output/joint_states` : [sensor_msgs/JointState](sensor_msgs/Imu)
* `/ihmc_ros/valkyrie/output/robot_motion_status` : [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)
* `/ihmc_ros/valkyrie/output/robot_pose` : [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `/ihmc_ros/valkyrie/output/walking_status` : [ihmc_msgs/WalkingStatusRosMessage](https://stash.ihmc.us/projects/ROS/repos/ihmc-ros-core/browse/ihmc_msgs/msg/WalkingStatusRosMessage.msg)