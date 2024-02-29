![](./images/banner.png)

## ROS 2 Humble Development Branch

This is a development branch that we are using to port [stretch_ros](https://github.com/hello-robot/stretch_ros) to ROS 2 Humble, Python 3, and Ubuntu 22.04. We plan to begin shipping this version preinstalled on Stretch robots in the future. It is **not in a stable state**, since we are actively conducting development in this branch. Since we have performed limited testing, you may encounter unexpected behaviors. Also, the installation requires [Ubuntu 22.04](https://github.com/hello-robot/stretch_install/blob/master/docs/robot_install.md) on a second partition of your robot's hard drive.

We are beginning to use this port internally at Hello Robot to test it, improve it, and add new capabilities.

## Available support in ROS 2

 - Stretch Core (Partial)
    - Stretch driver
    - RPLidar driver
    - D435i driver
    - ReSpeaker driver
    - Aruco Detection
    - Keyboard teleop (known bugs)
 - Hello Helpers
 - Stretch Calibration
 - Stretch Description
 - Stretch Nav2
    - Mapping with slam_toolbox
    - Navigation with nav2
    - Simple Commander Python API
 - Stretch Deep Perception
 - Stretch Demos
 - Stretch FUNMAP
 - Stretch OctoMap
 - Stretch RTABMap

## Known Issues

No support for:
 - Stretch Dashboard
 - Stretch Gazebo

---

## Directories

The *stretch_ros2* repository holds ROS 2 related code for the Stretch mobile manipulator from Hello Robot Inc.

| Resource                                                     | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
[hello_helpers](hello_helpers/README.md) | Miscellaneous helper code used across the stretch_ros2 repository
[stretch_calibration](stretch_calibration/README.md) | Creates and updates calibrated URDFs for the Stretch mobile manipulator
[stretch_core](stretch_core/README.md) | ROS 2 drivers for Stretch
[stretch_deep_perception](stretch_deep_perception/README.md) | Demonstrations that use open deep learning models to perceive the world
[stretch_demos](stretch_demos/README.md) | Demonstrations of simple autonomous manipulation
[stretch_description](stretch_description/README.md) | Generate and export URDFs
[stretch_funmap](stretch_funmap/README.md) | Demonstrations of Fast Unified Navigation, Manipulation And Planning (FUNMAP)
[stretch_nav2](stretch_nav2/README.md) | Support for the ROS navigation stack Nav2, including slam_toolbox, AMCL and Simple Commander
[stretch_octomap](stretch_octomap/README.md) | Support for mapping using OctoMap: efficient probabilistic 3D Mapping based on Octrees
[stretch_rtabmap](stretch_rtabmap/README.md) | Support for mapping using Real-Time Appearance-Based Mapping (RTAB-Map)

## Licenses

This software is intended for use with the Stretch mobile manipulators, which are robots produced and sold by Hello Robot Inc. For further information, including inquiries about dual licensing, please contact Hello Robot Inc.

For license details for this repository, see the LICENSE files found in the directories. A summary of the licenses follows: 

Directory               | License
------------------------|--------------------------------------------------------------------------------------
hello_helpers           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_calibration     | [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html)
stretch_core            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_deep_perception | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_demos           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_description     | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/)
stretch_funmap          | [LGPLv3](https://www.gnu.org/licenses/lgpl-3.0.en.html)
stretch_nav2            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_octomap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_rtabmap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
