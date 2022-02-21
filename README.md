![](./images/banner.png)

## ROS 2 Galactic Development Branch

This is a development branch that we are using to port [stretch_ros](https://github.com/hello-robot/stretch_ros) to ROS 2 Galactic, Python 3, and Ubuntu 20.04. We plan to begin shipping Ubuntu 20.04 with the ROS2 packages in this repo and the [Noetic ROS1 packages](https://github.com/hello-robot/stretch_ros/tree/dev/noetic) preinstalled on Stretch RE1 robots in the upcoming months. Currently, these packages are **not in a usable state**. It is also unstable, since we are actively conducting development in this branch. Since we have performed limited testing, you may encounter unexpected behaviors. Also, installation **requires Ubuntu 20.04 on a second partition** of your robot's hard drive.

We are beginning to use this port internally at Hello Robot to test it, improve it, and add new capabilities. We also anticipate that some customers will begin working with this development branch. If you wish to try it, please see the [installation guide](https://github.com/hello-robot/stretch_ros/blob/dev/noetic/install_noetic.md). **Both this branch and the installation guide are under active development. Please proceed with caution.**

**Please file issues here and ask general questions on the [forum](https://forum.hello-robot.com/).**

## Ported to ROS 2

 - Stretch Description
 - Stretch MoveIt Config
 - Stretch Core (Driver Node)

## Known Issues

No support for:

 - Stretch Calibration
 - Stretch Core (Everything Else)
 - Stretch Dashboard
 - Stretch Deep Perception
 - Stretch Demos
 - Stretch FUNMAP
 - Stretch Gazebo
 - Stretch Navigation
 - Stretch OctoMap
 - Stretch RTABMap
 - The deep perception demos won't work with a default installation, since they require OpenCV compiled with OpenVINO.
 - There is no support for the Respeaker Microphone Array.
 - There is no support for the Dexterous Wrist.

## Code Status & Development Plans

We intend to port every ROS1 package available for Stretch to ROS2. This table provides a high-level summary of the current state of the code. We're beginning by porting the lowest level packages first (e.g. Stretch Core, Stretch Description) and working towards the higher level packages (e.g. Nav2, Octomap). One notable exception is the 'stretch_moveit_config' package, which was ported in October 2021 to demo MoveIt2 + Stretch at ROS World 2021. See the README for instructions to run MoveIt2 planning/execution on Stretch RE1.

| Package                                                      | Status         | Notes                                                                  |
|--------------------------------------------------------------|----------------|------------------------------------------------------------------------|
| [hello_helpers](hello_helpers/README.md)                     | IN DEVELOPMENT | HelloNode class is ported to ROS2                                      |
| [stretch_calibration](stretch_calibration/README.md)         | NOT YET PORTED |                                                                        |
| [stretch_core](stretch_core/README.md)                       | IN DEVELOPMENT | stretch_driver node and launch file is ported to ROS2                  |
| [stretch_deep_perception](stretch_deep_perception/README.md) | NOT YET PORTED |                                                                        |
| [stretch_demos](stretch_demos/README.md)                     | NOT YET PORTED |                                                                        |
| [stretch_description](stretch_description/README.md)         | PORTED - GOOD  | Generate and export URDFs                                              |
| [stretch_funmap](stretch_funmap/README.md)                   | NOT YET PORTED |                                                                        |
| [stretch_gazebo](stretch_gazebo/README.md)                   | NOT YET PORTED | Ignition simulation in 'ros_world2021' branch                          |
| [stretch_moveit_config](stretch_gazebo/README.md)            | PORTED - GOOD  | Config files to use Stretch with the MoveIt2 Motion Planning Framework |
| [stretch_navigation](stretch_navigation/README.md)           | NOT YET PORTED |                                                                        |
| [stretch_octomap](stretch_octomap/README.md)                 | NOT YET PORTED |                                                                        |
| [stretch_rtabmap](stretch_rtabmap/README.md)                 | NOT YET PORTED |                                                                        |

---

## Directories

The *stretch_ros* repository holds ROS related code for the Stretch RE1 mobile manipulator from Hello Robot Inc. For an overview of the capabilities in this repository, we recommend you look at the following forum post: https://forum.hello-robot.com/t/autonomy-video-details


| Resource                                                     | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
[hello_helpers](hello_helpers/README.md) | Miscellaneous helper code used across the stretch_ros repository
[stretch_calibration](stretch_calibration/README.md) | Creates and updates calibrated URDFs for the Stretch RE1
[stretch_core](stretch_core/README.md) | Enables basic use of the Stretch RE1 from ROS
[stretch_deep_perception](stretch_deep_perception/README.md) | Demonstrations that use open deep learning models to perceive the world
[stretch_demos](stretch_demos/README.md) | Demonstrations of simple autonomous manipulation
[stretch_description](stretch_description/README.md) | Generate and export URDFs
[stretch_funmap](stretch_funmap/README.md) | Demonstrations of Fast Unified Navigation, Manipulation And Planning (FUNMAP)
[stretch_gazebo](stretch_gazebo/README.md) | Support for simulation of Stretch in the Gazebo simulator
[stretch_moveit_config](stretch_gazebo/README.md) | Config files to use Stretch with the MoveIt Motion Planning Framework
[stretch_navigation](stretch_navigation/README.md) | Support for the ROS navigation stack, including move_base, gmapping, and AMCL
[stretch_octomap](stretch_octomap/README.md) | Support for mapping using OctoMap: efficient probabilistic 3D Mapping based on Octrees
[stretch_rtabmap](stretch_rtabmap/README.md) | Support for mapping using Real-Time Appearance-Based Mapping (RTAB-Map)

## Licenses

This software is intended for use with the Stretch RE1 mobile manipulator, which is a robot produced and sold by Hello Robot Inc. For further information, including inquiries about dual licensing, please contact Hello Robot Inc.

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
stretch_gazebo          | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_moveit_config   | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_navigation      | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_octomap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_rtabmap         | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
