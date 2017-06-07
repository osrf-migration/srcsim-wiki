# SRC API

Competitors in the Space Robotics Challenge (SRC) are expected to write a
[ROS](http://ros.org) program that controls the R5 robot to solve each task.
This page contains information about the ROS interfaces that competitors may
use to collect state information (sensor data, joint states), send control
commands (foot placement) and manage the competition run (start / skip tasks).

Not listed on this page are standard ROS topics, such as `/tf`, which are not
unique to the SRC. If this is confusing, then a good place to start are the
[ROS tutorials](http://wiki.ros.org/ROS/Tutorials).

Topics act as named data pipes, where the data transmitted is a ROS message.
Information in this section was generated from the `rostopic list` and
`rostopic info <topic_name>` command line utilities. We encourage competitors
to use these tools during development.

### Robot interface

These are interfaces to receive sensor data and send commands to the robot.

Topic / service name | Message type
---------- | ------------
`/ihmc_ros/localization/pelvis_odom_pose_correction` | [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
`/ihmc_ros/localization/pelvis_pose_correction` | [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
`/ihmc_ros/valkyrie/control/abort_walking` | [ihmc_msgs/AbortWalkingRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/AbortWalkingRosMessage.msg)
`/ihmc_ros/valkyrie/control/arm_desired_joint_accelerations` | [ihmc_msgs/ArmDesiredAccelerationsRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/ArmDesiredAccelerationsRosMessage.msg)
`/ihmc_ros/valkyrie/control/arm_trajectory` | [ihmc_msgs/ArmTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/ArmTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/chest_trajectory` | [ihmc_msgs/ChestTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/ChestTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/end_effector_load_bearing` | [ihmc_msgs/EndEffectorLoadBearingRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/EndEffectorLoadBearingRosMessage.msg)
`/ihmc_ros/valkyrie/control/foot_trajectory` | [ihmc_msgs/FootTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/FootTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/footstep_list` | [ihmc_msgs/FootstepDataListRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/FootstepDataListRosMessage.msg)
`/ihmc_ros/valkyrie/control/go_home` | [ihmc_msgs/GoHomeRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/GoHomeRosMessage.msg)
~~/ihmc_ros/valkyrie/control/hand_desired_configuration~~ | [~~ihmc_msgs/HandDesiredConfigurationRosMessage~~](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HandDesiredConfigurationRosMessage.msg), this topic is not in use
`/ihmc_ros/valkyrie/control/hand_trajectory` | [ihmc_msgs/HandTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HandTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/head_trajectory` | [ihmc_msgs/HeadTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HeadTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/high_level_state` | [ihmc_msgs/HighLevelStateRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HighLevelStateRosMessage.msg)
`/ihmc_ros/valkyrie/control/low_level_control_mode` | [ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage](https://github.com/ihmcrobotics/ihmc_valkyrie_ros/blob/develop/msg/ValkyrieLowLevelControlModeRosMessage.msg)
`/ihmc_ros/valkyrie/control/neck_desired_acceleration` | [ihmc_msgs/NeckDesiredAccelerationsRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/NeckDesiredAccelerationsRosMessage.msg)
`/ihmc_ros/valkyrie/control/neck_trajectory` | [ihmc_msgs/NeckTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/NeckTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/pause_walking` | [ihmc_msgs/PauseWalkingRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/PauseWalkingRosMessage.msg)
`/ihmc_ros/valkyrie/control/pelvis_height_trajectory` | [ihmc_msgs/PelvisHeightTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/PelvisHeightTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory` | [ihmc_msgs/PelvisOrientationTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/PelvisOrientationTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/pelvis_trajectory` | [ihmc_msgs/PelvisTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/PelvisTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/request_stop` | [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
`/ihmc_ros/valkyrie/control/stop_all_trajectories` | [ihmc_msgs/StopAllTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/StopAllTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/control/whole_body_trajectory` | [ihmc_msgs/WholeBodyTrajectoryRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/WholeBodyTrajectoryRosMessage.msg)
`/ihmc_ros/valkyrie/output/behavior` | [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html)
`/ihmc_ros/valkyrie/output/capturability/capture_point` | [ihmc_msgs/Point2dRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/Point2dRosMessage.msg)
`/ihmc_ros/valkyrie/output/capturability/center_of_mass` | [geometry_msgs/Point32](http://docs.ros.org/api/geometry_msgs/html/msg/Point32.html)
`/ihmc_ros/valkyrie/output/capturability/desired_capture_point` | [ihmc_msgs/Point2dRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/Point2dRosMessage.msg)
`/ihmc_ros/valkyrie/output/capturability/is_in_double_support` | [std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)
`/ihmc_ros/valkyrie/output/capturability/left_foot_support_polygon` | [ihmc_msgs/SupportPolygonRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/SupportPolygonRosMessage.msg)
`/ihmc_ros/valkyrie/output/capturability/right_foot_support_polygon` | [ihmc_msgs/SupportPolygonRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/SupportPolygonRosMessage.msg)
`/ihmc_ros/valkyrie/output/foot_force_sensor/left` | [geometry_msgs/WrenchStamped](http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html)
`/ihmc_ros/valkyrie/output/foot_force_sensor/right` | [geometry_msgs/WrenchStamped](http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html)
`/ihmc_ros/valkyrie/output/footstep_status` | [ihmc_msgs/FootstepStatusRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/FootstepStatusRosMessage.msg)
`/ihmc_ros/valkyrie/output/high_level_state` | [ihmc_msgs/HighLevelStateRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HighLevelStateRosMessage.msg)
`/ihmc_ros/valkyrie/output/high_level_state_change` | [ihmc_msgs/HighLevelStateChangeStatusRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/HighLevelStateChangeStatusRosMessage.msg)
`/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu` | [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
`/ihmc_ros/valkyrie/output/imu/pelvis_pelvisRearImu` | [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
`/ihmc_ros/valkyrie/output/imu/torso_leftTorsoImu` | [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
`/ihmc_ros/valkyrie/output/imu/upperNeckPitchLink_head_imu_sensor` | [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
`/ihmc_ros/valkyrie/output/joint_states` | [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
`/ihmc_ros/valkyrie/output/robot_motion_status` | [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)
`/ihmc_ros/valkyrie/output/robot_pose` (becomes wrong after teleporting) | [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
`/ihmc_ros/valkyrie/output/walking_status` | [ihmc_msgs/WalkingStatusRosMessage](https://github.com/ihmcrobotics/ihmc_ros_core/blob/develop/ihmc_msgs/msg/WalkingStatusRosMessage.msg)
`/left_hand_position_controller/command` | [std_msgs/Float64MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html)
`/right_hand_position_controller/command` | [std_msgs/Float64MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html)

### Competition interface (finals)

These are the interfaces to interact with the flow of the competition.

For more detailed information, see the [finals](https://bitbucket.org/osrf/srcsim/wiki/finals) tutorials.

Topic name | Message / service | Description | Message definition
---------- | ----------------- | ----------- | ----------------
`/srcsim/finals/start_task` | Service | Start a specific task at a specific checkpoint. | [srcsim/StartTask.srv](https://bitbucket.org/osrf/srcsim/raw/default/srv/StartTask.srv)
`/srcsim/finals/task` | Message | Task status updates | [srcsim/Task.msg](https://bitbucket.org/osrf/srcsim/raw/default/msg/Task.msg)
`/srcsim/finals/score` | Message | Score updates | [srcsim/Score.msg](https://bitbucket.org/osrf/srcsim/raw/default/msg/Score.msg)
`/srcsim/finals/harness` | Message | Harness updates | [srcsim/Harness.msg](https://bitbucket.org/osrf/srcsim/raw/default/msg/Harness.msg)

#### Finals: Task 1

These are interfaces specific to task 1.

For more detailed information, see the [task 1](https://bitbucket.org/osrf/srcsim/wiki/finals_task1) tutorials.

Topic name | Message / service | Description | Message definition
---------- | ----------------- | ----------- | ----------------
`/task1/checkpoint2/satellite` | Message | Satellite status updates | [srcsim/Satellite.msg](https://bitbucket.org/osrf/srcsim/raw/default/msg/Satellite.msg)


#### Finals: Task 2

There are no interfaces specific to task 2.

For more detailed information, see the [task 2](https://bitbucket.org/osrf/srcsim/wiki/finals_task2) tutorials.

#### Finals: Task 3

These are interfaces specific to task 3.

For more detailed information, see the [task 3](https://bitbucket.org/osrf/srcsim/wiki/finals_task3) tutorials.

Topic name | Message / service | Description | Message definition
---------- | ----------------- | ----------- | ----------------
`/task3/checkpoint5/leak` | Message | Leak detector readings | [srcsim/Leak.msg](https://bitbucket.org/osrf/srcsim/raw/default/msg/Leak.msg)
`/task3/checkpoint5/leak_pose` | Message | Leak position expressed in the robot pelvis frame, only published if checkpoint is skipped | [geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html)