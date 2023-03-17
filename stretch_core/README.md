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
