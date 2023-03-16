![](../images/banner.png)

# Overview

*stretch_core* provides the core ROS interfaces to the Stretch mobile manipulator. It includes the following:

Nodes:
 - `d435i_*.py` : various nodes to help use Stretch's 3D camera
 - `detect_aruco_markers.py`: node that detects and estimates the pose of ArUco markers, including the markers on the robot's body
 - `joint_trajectory_server.py`: action server to execute trajectories generated using planners like MoveIt2
 - `keyboard_*.py` : nodes that provides a keyboard interface to control the robot's joints
 - `stretch_driver.py`: node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1
 - `trajectory_components.py`: node defining classes for each joint to generate trajectory waypoints

Launch files:
 - `d435i_*.launch.py`: launches the d435i driver node with defined resolution
 - `keyboard_teleop.launch.py`: launches the node that allows teleoperating robot joints with keyboard
 - `rplidar.launch.py`: launches the RPLidar driver node
 - `stretch_aruco.launch.py`: launches the aruco detection node
 - `stretch_driver.launch.py`: launches the stretch driver node with defined configurations

Config:
 - `controller_calibration_head.yaml`: stores the backlash errors in the head servos
 - `laser_filter_params.yaml`: configures the filters that are used for laser scan filtering
 - `stretch_marker_dict.yaml`: stores information about ArUco markers known and assigned for use with Stretch

## API

### stretch_driver
This is the API documentation for the stretch_driver.py file in the stretch_core package of the stretch_ros2 repository. This driver provides an interface to communicate with the Stretch Robot from Hello Robot Inc.

#### Parameters
`broadcast_odom_tf`:
description: Whether to broadcast the odom TF
default_value='False'
choices=['True', 'False']

`mode`:
description: The mode in which the Stretch ROS driver commands the robot
default='position'
choices=['position', 'navigation', 'manipulation']

`fail_out_of_range_goal`:
description: Whether the motion action servers fail on out-of-range commands
default_value='True'
choices=['True', 'False']

`calibrated_controller_yaml_file`:
description: Path to the calibrated controller args file
default: stretch_core/config/controller_calibration_head.yaml

#### Published Topics
`/odom`: The odometry from the wheel encoders through a message of type nav_msgs.msg.Odometry
`/battery`: Publishes the battery state information through a message of type sensor_msgs.msg.BatteryState
`/is_homed`: Publishes whether the robot is homed through a message of type std_msgs.msg.Bool
`/mode`: Publishes the command mode the Stretch Driver is in through a message of type std_msgs.msg.String
`/imu_mobile_base`: Publishes IMU data from the Pimu through a message of type sensor_msgs.msg.Imu
`/magnetometer_mobile_base`: Publishes magnetometer data fromt he Pimu through a message of type sensor_msgs.msg.MagneticField
`/imu_wrist`: Publishes IMU data from the Wacc through a message of type sensor_msgs.msg.Imu
`/joint_states`: Publishes joint positions through a message of type sensor_msgs.msg.JointState

#### Subscribed Topics
`/cmd_vel`: Translational and rotational velocities through a message of type geometry_msgs.msg.Twist

#### Exposed Services
`/switch_to_position_mode`: change mode to position mode
`/switch_to_manipulation_mode`: change mode to manipulation mode
`/switch_to_navigation_mode`: change mode to navigation mode
`/stop_the_robot`: stop the robot
`/home_the_robot`: home the robot
`/runstop`: switches the robot to standby mode where it will ignore new commands

## Testing

Colcon is used to run the integration tests in the */test* folder. The command to run the entire suite of tests is:

```bash
$ cd ~/ament_ws
$ colcon test --packages-select stretch_core
```

Here are description of each test suite:

 - **test_services.py**: tests the ROS2 services within the stretch_driver node
 - **test_action_manipulation_mode.py**: tests the ROS2 FollowJointTrajectory action server's manipulation mode
 - **test_flake8.py and test_pep257.py**: linters that identify unrecommended code style

## License

For license information, please see the LICENSE files.
