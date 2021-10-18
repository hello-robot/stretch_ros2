![](../images/banner.png)

## Overview

*stretch_core* provides the core ROS interfaces to the Stretch RE1 mobile manipulator from Hello Robot Inc. It includes the following nodes:

*stretch_driver* : node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1

*detect_aruco_markers* : node that detects and estimates the pose of ArUco markers, including the markers on the robot's body

*d435i_** : various nodes to help use the Stretch RE1's 3D camera

*keyboard_teleop* : node that provides a keyboard interface to control the robot's joints

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
