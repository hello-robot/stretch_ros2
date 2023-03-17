## API Documentation Stretch Core

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

#### Exposed Action Servers
`/stretch_controller/follow_joint_trajectory`: action server to control the robot through joint trajectories