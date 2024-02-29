![](./images/banner.png)

## Getting Started

This repository holds ROS 2 Humble packages for the Stretch mobile manipulators from Hello Robot Inc. To get started with this code, take a look at the [Stretch ROS 2 Tutorials](https://docs.hello-robot.com/0.3/ros2/getting_started/). If you are porting code build on Stretch's ROS 1 Noetic packages, check out the [Migrating to ROS 2]((https://docs.hello-robot.com/0.3/ros2/TODO/) guide.

## Directories

Resource                                                     | Description
-------------------------------------------------------------|---------------------------------------------------------------------------------------------
[stretch_calibration](stretch_calibration/README.md)         | Creates **calibrated** URDFs for Stretch
[stretch_core](stretch_core/README.md)                       | ROS 2 drivers for Stretch
[stretch_deep_perception](stretch_deep_perception/README.md) | Demonstrations that use open deep learning models to perceive the world
[stretch_demos](stretch_demos/README.md)                     | Demonstrations of simple autonomous manipulation
[stretch_description](stretch_description/README.md)         | Description files for Stretch
[stretch_funmap](stretch_funmap/README.md)                   | Demonstrations of Fast Unified Navigation, Manipulation And Planning (FUNMAP)
[stretch_nav2](stretch_nav2/README.md)                       | Navigation stack Nav2, including slam_toolbox, AMCL and Simple Commander
[stretch_octomap](stretch_octomap/README.md)                 | Mapping using OctoMap: efficient probabilistic 3D Mapping based on Octrees
[stretch_rtabmap](stretch_rtabmap/README.md)                 | Navigation & Mapping using Real-Time Appearance-Based Mapping (RTAB-Map)
[hello_helpers](hello_helpers/README.md)                     | Miscellaneous helper code used across the stretch_ros2 repository

## Licenses

This software is intended for use with the Stretch mobile manipulators, which are robots produced and sold by Hello Robot Inc. For further information, including inquiries about dual licensing, please contact Hello Robot Inc.

For license details for this repository, see the LICENSE files found in the directories. A summary of the licenses follows: 

Directory               | License
------------------------|--------------------------------------------------------------------------------------
stretch_calibration     | [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html)
stretch_core            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_deep_perception | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_demos           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_description     | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/)
stretch_funmap          | [LGPLv3](https://www.gnu.org/licenses/lgpl-3.0.en.html)
stretch_nav2            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_octomap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_rtabmap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
hello_helpers           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
