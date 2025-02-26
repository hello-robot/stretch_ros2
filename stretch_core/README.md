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
 - `usb_cam.py`: node that uses opencv python to access usb camera feed 

Launch files:
 - `d435i_*.launch.py`: launches the d435i driver node with defined resolution
 - `keyboard_teleop.launch.py`: launches the node that allows teleoperating robot joints with keyboard
 - `rplidar.launch.py`: launches the RPLidar driver node
 - `stretch_aruco.launch.py`: launches the aruco detection node
 - `stretch_driver.launch.py`: launches the stretch driver node with defined configurations
 - `navigation_camera.launch.py`: launches the usb cam node to access the stretch's overhead/navigation wide-angle camera 

Config:
 - `controller_calibration_head.yaml`: stores the backlash errors in the head servos
 - `laser_filter_params.yaml`: configures the filters that are used for laser scan filtering
 - `stretch_marker_dict.yaml`: stores information about ArUco markers known and assigned for use with Stretch

## API

### Nodes

### [stretch_driver](./stretch_core/stretch_driver.py)

#### Parameters

##### broadcast_odom_tf

If set to true, stretch_driver will publish an odom to base_link TF.

##### fail_out_of_range_goal

If set to true, motion action servers will fail on out-of-range commands.

##### mode

Can be set to `position`, `navigation`, and `trajectory` modes.

##### Streming Position Control
When Streaming position control is activated, publish goal Joint pose as a `Float64MultiArray`. Streaming position control can be activated and deactivated by calling the services `/activate_streaming_position` and `/deactivate_streaming_position`. The current activation state is published in topic `/is_streaming_position`.
This feature can be used in `position` and `navigation` modes. [More info on framing Goal pose array](https://github.com/hello-robot/stretch_ros2/pull/154#issue-2449787145).

##### calibrated_controller_yaml_file

Path to the calibrated controller args file

#### Published Topics

##### /mode ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html))

This topic publishes which mode the driver is in at 15hz. stretch_driver has a few modes that change how the robot is controlled. The modes are:

  - "position": The default and simplest mode. In this mode, you can control every joint on the robot using position commands. For example, the telescoping arm (whose range is from zero fully retracted to ~52 centimeters fully extended) would move to 25cm out when you send it a 0.25m position cmd <!-- [0.25m position cmd](../hello_helpers/README.md#movetoposepose-returnbeforedonefalse-customcontactthresholdsfalse-customfullgoalfalse)-->. Two kinds of position commands are available for the mobile base, translation and rotation, with joint names "translate_mobile_base" and "rotate_mobile_base", and these commands have no limits since the wheels can spin continuously.
   - Position commands are tracked in the firmware by a trapezoidal motion profile, and specifing the optional velocity and acceleration in [JointTrajectoryPoint](https://docs.ros2.org/latest/api/trajectory_msgs/msg/JointTrajectoryPoint.html) changes the shape of the trapezoid. <!-- More details about the motion profile [in the tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/follow_joint_trajectory/). -->
   - Position commands can be contact sensitive, which is helpful for manipulating objects in the world. For example, I could [open a cabinet](https://youtu.be/SXgj9be3PdM) by reaching out with the telescoping arm and detecting contact with the door. In order to specify contact thresholds for a position command, the optional effort in [JointTrajectoryPoint](https://docs.ros2.org/latest/api/trajectory_msgs/msg/JointTrajectoryPoint.html) is misused to mean a threshold for the effort the robot will apply while executing the position command.
   - Position commands can be preemptable, so you can issue a new position command before the previous one has finished and the robot will smoothly move to execute the latest command. This feature is helpful for scripts that use visual servo-ing or allow a user to teleop the robot.
   - The driver can be switched into "position" mode at any time using the [switch_to_position_mode service](#TODO).
 - "navigation": In this mode, every joint behaves identically to "position" mode except for the mobile base. The mobile base responds to velocity commands at a topic instead of position commands via the "translate_mobile_base" and "rotate_mobile_base" joints in the action server. You would publish [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) messages to the [/stretch/cmd_vel topic](#TODO). Since Twist messages are generalized to robots that can move with velocity in any direction, only the `Twist.linear.x` (translational velocity) and `Twist.angular.z` (rotational velocity) fields apply for differential drive mobile bases.
   - Velocity control of the base is a common way to move mobile robots around. For example, the [Navigation Stack](../stretch_nav2/README.md) is a piece of software that uses this mode to move the robot to different points in a map.
   - This mode has a few safety features to prevent Stretch from "running away" at the last commanded velocity if the node that was sending commands were to crash for whatever reason. The first is a 0.5 second timeout within the stretch_driver node, which means that if the driver doesn't receive a new Twist command within 0.5s, the base will be commanded to stop smoothly. The second is a 1 second timeout within the firmware of the wheels, which means that even if the ROS layer were to crash, the robot will still be commanded to stop abruptly within 1 second at the lowest layer. This low-level feature was added relatively recently to the firmware, so be sure to [update your firmware](https://github.com/hello-robot/stretch_firmware/blob/master/docs/tutorial_updating_firmware.md) to the latest version.
   - The driver can be switched into "navigation" mode at any time using the [switch_to_navigation_mode service](#TODO).
 - "trajectory": In this mode, every joint follows a trajectory that is modeled as a spline. The key benefit of this mode is control over the timing at which the robot achieves positions (a.k.a waypoints), enabling smooth and coordinated motion through a preplanned trajectory. [More details here](https://forum.hello-robot.com/t/creating-smooth-motion-using-trajectories/671).
 - "homing": This is the mode reported by the driver when a homing sequence has been triggered (e.g. through the [home_the_robot service](#hometherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml)). While this mode is active, no other commands will be accepted by the driver. After the robot has completed its 30 second homing sequence, it will return to the mode it was in before.
 - "stowing": This is the mode reported by the driver when a stowing sequence has been triggered (e.g. through the [stow_the_robot service](#stowtherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml)). While this mode is active, no other commands will be accepted by the driver. After the robot has completed its stowing sequence, it will return to the mode it was in before.
 - "runstopped": This is the mode reported by the driver when the robot is in runstop, either through the user pressing the [glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head or through the [runstop service](#runstop-stdsrvssetboolhttpsdocsrosorgennoeticapistdsrvshtmlsrvsetboolhtml). While this mode is active, no other commands will be accepted by the driver. After the robot has been taken out of runstop, it will return to the mode it was in before, or "position" mode if the driver was launched while the robot was runstopped.
 <!-- - "manipulation": **Deprecated**. This mode was previously available in ROS1 Melodic, but was removed when the driver was ported to ROS1 Noetic. The mode supported a virtual prismatic joint for the mobile base, allowing you to treat the mobile base as a joint that could move forwards/backwards 0.5m. It was removed because the mode had little utility for most users of the driver. -->

##### /battery ([sensor_msgs/BatteryState](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html))

This topic publishes Stretch's battery and charge status. Charging status, the `power_supply_status` field, is estimated by looking at changes in voltage readings over time, where plugging-in causes the voltage to jump up (i.e. status becomes 'charging') and pulling the plug out is detected by a voltage dip (i.e. status becomes 'discharging'). Estimation of charging status is most reliable when the charger is in SUPPLY mode (see [docs here](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/battery_maintenance_guide_re1/#charger) for how to change charging modes). Charging status is unknown at boot of this node. Consequently, the `current` field is positive at boot of this node, regardless of whether the robot is charging/discharging. After a charging state change, there is a ~10 second timeout where state changes won't be detected. Additionally, outlier voltage readings can slip past the filters and incorrectly indicate a charging state change (albeit rarely). Finally, voltage readings are affected by power draw (e.g. the onboard computer starts a computationally taxing program), which can lead to incorrect indications of charging state change. Stretch RE2s have a hardware switch in the charging port that can detect when a plug has been plugged in, regardless of whether the plug is providing any power. Therefore, this node combines the previous voltage-based estimate with readings from this hardware switch to make better charging state estimates on RE2s (effectively eliminating the false positive case where a computational load draws more power).

Since a battery is always present on a Stretch system, we instead misuse the `present` field to indicate whether a plug is plugged in to the charging port (regardless of whether it's providing power) on RE2 robots. This field is always false on RE1s. The unmeasured fields (e.g. charge in Ah) return a NaN, or 'not a number'.

##### /is_homed ([std_msgs/Bool](https://docs.ros2.org/latest/api/std_msgs/msg/Bool.html))

This topic publishes whether Stretch's encoders has been homed. If the robot isn't homed, joint states will be incorrect and motion commands won't be accepted by the driver. The [home_the_robot service](#hometherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml) can be used to home Stretch.

##### /is_runstopped ([std_msgs/Bool](https://docs.ros2.org/latest/api/std_msgs/msg/Bool.html))

This topic publishes whether Stretch is runstopped. When the robot is runstopped (typically by [pressing glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head), motion for all joints is stopped and new motion commands aren't accepted. This is a safety feature built into the firmware for the four primary actuators. It's also possible to runstop the robot programmatically using the [runstop service](#runstop-stdsrvssetboolhttpsdocsrosorgennoeticapistdsrvshtmlsrvsetboolhtml)

<!-- ##### /is_calibrated ([std_msgs/Bool](https://docs.ros2.org/latest/api/std_msgs/msg/Bool.html))

**Deprecated:** This topic has been renamed to [is_homed](#ishomed-stdmsgsboolhttpsdocsrosorgennoeticapistdmsgshtmlmsgboolhtml) because we often use the terms "calibrated" or "calibration" in context of [URDF calibrations](../stretch_calibration/README.md), whereas this topic returns whether the robot's encoders are homed. -->

#### Published Services

##### /home_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service will start Stretch's homing procedure, where every joint's zero is found. Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it's a 30-second procedure that must occur everytime the robot wakes up before you may send motion commands to or read correct joint positions from Stretch's prismatic and multiturn revolute joints. When this service is triggered, the [mode topic](https://docs.ros2.org/latest/api/std_msgs/msg/String.html) will reflect that the robot is in "homing" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "homing" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to home the robot include using the `stretch_robot_home.py` CLI tool from a terminal, or calling [`robot.home()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.home) from Stretch's Python API.

Runstopping the robot while this service is running will yield undefined behavior and likely leave the driver in a bad state.

##### /stow_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service will start Stretch's stowing procedure, where the arm is stowed into the footprint of the mobile base. This service is more convenient than sending a [follow joint trajectory command](#TODO) since it knows what gripper is installed at the end of arm and stows these additional joints automatically. When this service is triggered, the [mode topic](#mode-stdmsgsstringhttpsdocsrosorgennoeticapistdmsgshtmlmsgstringhtml) will reflect that the robot is in "stowing" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "stowing" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to stow the robot include using the `stretch_robot_stow.py` CLI tool from a terminal, or calling [`robot.stow()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.stow) from Stretch's Python API.

Runstopping the robot while this service is running will yield undefined behavior and likely leave the driver in a bad state.

##### /runstop ([std_srvs/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html))

This service can put Stretch into runstop or take Stretch out of runstop. It's common to put the robot into/out of runstop by pressing the [glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head (at which point the robot will beep and the button will be blinking to indicate that it's runstopped), and holding the button down for two seconds to take it out of runstop (the button will return to non-blinking). This service acts as a programmatic way to achieve the same effect. When this service is triggered, the [mode topic](#mode-stdmsgsstringhttpsdocsrosorgennoeticapistdmsgshtmlmsgstringhtml) will reflect that the robot is in "runstopped" mode, and after the robot is taken out of runstop, the driver will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "runstopped" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

<!-- ##### /calibrate_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

**Deprecated:** This service has been renamed to [home_the_robot](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html) because we often use the terms "calibrate" or "calibration" in context of [URDF calibrations](../stretch_calibration/README.md), whereas this service homes the robot's encoders. -->

### [keyboard_teleop](./stretch_core/keyboard_teleop.py)

#### Parameters
##### mapping_on

Parameter to enable various [FUNMAP](../stretch_funmap/stretch_funmap/funmap.py) mapping related capabilities like head and drive scan, global and local localization, aligning with a nearby cliff, and executing a motion until contact from stretch_funmap.

##### hello_world_on

Parameter to enable the [hello world](../stretch_demos/stretch_demos/hello_world.py) demo from stretch_demos.

##### open_drawer_on

Parameter to enable the [open drawer](../stretch_demos/stretch_demos/open_drawer.py) demo from stretch_demos.

##### clean_surface_on

Parameter to enable the [clean surface](../stretch_demos/stretch_demos/clean_surface.py) demo from stretch_demos.

##### grasp_object_on

Parameter to enable the [grasp object](../stretch_demos/stretch_demos/grasp_object.py) demo from stretch_demos.

##### handover_object_on

Parameter to enable the [handover object](../stretch_demos/stretch_demos/handover_object.py) demo from stretch_demos.

#### Subscribed Services
##### /funmap/trigger_head_scan ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger a head scan while running keyboard teleop along with FUNMAP.

##### /funmap/trigger_drive_to_scan ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger a drive scan while running keyboard teleop along with FUNMAP.

##### /funmap/trigger_global_localization ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger global localization while running keyboard teleop along with FUNMAP.

##### /funmap/trigger_local_localization ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger local localization while running keyboard teleop along with FUNMAP.

##### /funmap/trigger_align_with_nearest_cliff ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to enable Stretch to align to the nearest cliff while running keyboard teleop along with FUNMAP.

##### /funmap/trigger_reach_until_contact ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to enable Stretch to reach a surface with its arm and stop upon contact while running keyboard teleop along with FUNMAP. The capability uses Stretch's arm contact sesitivity to implement the reach until contact behavior.

##### /funmap/trigger_lower_until_contact ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to enable Stretch to lower its arm and stop upon contact while running keyboard teleop along with FUNMAP.
The capability uses Stretch's lift contact sesitivity to implement the lower until contact behavior.

##### /hello_world/trigger_write_hello ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the hello world demo while running keyboard teleop along with the hello world demo launch.

##### /open_drawer/trigger_open_drawer_down ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the open drawer demo while running keyboard teleop along with the open drawer demo launch.
This service is specifically useful when the hook needs to make contact with the drawer while the arm is descending.

##### /open_drawer/trigger_open_drawer_up ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the open drawer demo while running keyboard teleop along with the open drawer demo launch.
This service is specifically useful when the hook needs to make contact with the drawer while the arm is ascending.

##### /clean_surface/trigger_clean_surface ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the clean surface demo while running keyboard teleop along with the clean surface demo launch.

##### /clean_surface/trigger_grasp_surface ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the grasp object demo while running keyboard teleop along with the grasp object launch.

##### /handover_object/trigger_handover_object ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

This service client can be called to trigger the handover object demo while running keyboard teleop along with the handover object launch.

## Testing

Colcon is used to run the integration tests in the */test* folder. The command to run the entire suite of tests is:

```bash
$ cd ~/ament_ws
$ colcon test --packages-select stretch_core
```

You can run individual tests using the following command. Optionally, you can have the test runner (pytest) print to the console.

```bash
$ colcon test --packages-select stretch_core --pytest-args -k test_core_topics_published --event-handlers console_direct+
```

Here are description of each test suite:

 - **test_services.py**: tests the ROS2 services within the stretch_driver node
 - **test_action_trajectory_mode.py**: tests the ROS2 FollowJointTrajectory action server's trajectory mode
 - **test_flake8.py and test_pep257.py**: linters that identify unrecommended code style

## License

For license information, please see the LICENSE files.
