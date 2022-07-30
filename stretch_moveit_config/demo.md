![](../images/banner.png)

## Overview

MoveIt 2 is a whole-body motion planning framework for mobile manipulators that allows planning pose and joint goals in environments with and without obstacles. Stretch being a mobile manipulator is uniquely well-suited to utilize the planning capabilities of MoveIt 2 in different scenarios.

## Motivation

Stretch has a kinematically simple 3 DoF arm (+2 with DexWrist) that is suitable for pick and place tasks of varied objects. Its mobile base provides it with 2 additional degrees of freedom that afford it more manipulability and also the ability to move around freely in its environment. To fully utilize these capabilities, we need a planner that can plan for both the arm and the mobile base at the same time. With MoveIt 2 and ROS 2, it is now possible to achieve this, empowering users to plan more complicated robot trajectories in difficult and uncertain environments.

## Demo with Stretch Robot

### Installing ROS 2
By default, Stretch RE1 ships with Ubuntu 18.04, ROS Melodic, and Python2 packages. These steps will install a second operating system with Ubuntu 20.04, ROS Noetic, ROS 2 Galactic (where MoveIt 2 is supported), and Python3 packages.

1. A new firmware called protocol 1 (p1) for Stretch adds support for spline trajectories, a feature used by MoveIt2. Use the [firmware update guide](https://github.com/hello-robot/stretch_firmware/blob/master/tutorials/docs/updating_firmware.md) to install the new firmware.

2. Use the [Noetic installation instructions](https://github.com/hello-robot/stretch_ros/blob/dev/noetic/install_noetic.md) to create the second Ubuntu 20.04 partition, and create a ROS2 Galactic workspace with MoveIt 2 installed.

3. Boot into the new Ubuntu 20.04 partition, and edit the last few lines of the ~/.bashrc file to comment out sourcing of Noetic workspaces. When both versions of ROS are sourced, package conflicts arise. The result should look like:

```
#source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
#source ~/catkin_ws/devel/setup.bash
source ~/ament_ws/install/setup.bash
```

### Installing MoveIt 2 and Its Dependencies

4. Clone the [stretch_ros2 repository](https://github.com/hello-robot/stretch_ros2) in your workspace and switch to the “feature/hybrid_planning” branch within stretch_ros2 using the following command.

```
cd ~/ament_ws/src
git clone https://github.com/hello-robot/stretch_ros2.git
git checkout feature/hybrid_planning
```

4. a. In the absence of support for stretch_calibration in ROS 2 (porting in progress which will make this step redundant), we need to manually move the calibrated urdf file named stretch.urdf from the stretch_description/urdf directory in stretch_ros (in ROS Melodic) to its counterpart in stretch_ros2 (in ROS 2 Galactic).

4. b. We also need to move the controller_calibration_head.yaml file from the stretch_core/config directory in stretch_ros to its counterpart in stretch_ros2.
 
5. In a new terminal, collect all of the packages to build from source. Use the command below to automate collection of these packages. Download the stretch_moveit2.repos file or copy and paste the contents of the file from [here](https://github.com/hello-robot/stretch_ros2/blob/feature/hybrid_planning/stretch_moveit_config/stretch_moveit2.repos).

```
cd ~/ament_ws/src
vcs import < stretch_moveit2.repos
```

6. Download additional dependencies and build the workspace using these commands. This should install MoveIt 2 and everything else you need to run the tutorials!

```
cd ~/ament_ws
rosdep install --from-paths src --ignore-src -r --rosdistro galactic -y
cd ~/ament_ws
colcon build
source ~/ament_ws/install/setup.bash
```

### Planning with MoveIt 2 Using RViz
Before we proceed, it's always a good idea to home the robot first by running the following script so that we have the correct joint positions being published on the /joint_states topic. This is necessary for planning trajectories on Stretch with MoveIt.

```
stretch_robot_home.py
```

7. The easiest way to run MoveIt 2 on your robot is through RViz. With RViz you can plan, visualize, and also execute trajectories for various planning groups on your robot. To launch RViz with MoveIt 2, run the following command. (Press Ctrl+C in the terminal to terminate)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

8. Follow instructions in this [tutorial](https://github.com/hello-robot/stretch_ros2/blob/feature/hybrid_planning/stretch_moveit_config/rviz_demo.md) to plan and execute trajectories using the interactive markers in RViz.

Use the interactive markers to drag joints to desired positions or go to the manipulation tab in the Motion Planning pane to fine-tune joint values using the sliders. Next, click the 'Plan' button to plan the trajectory. If the plan is valid, you should be able to execute the trajectory by clicking the 'Execute' button. Below we see Stretch raising its arm without any obstacle in the way.

![WithoutObstacle](https://user-images.githubusercontent.com/97639181/162533340-dec4232b-617c-4b90-b4e1-a24fd3027baa.gif)

To plan with obstacles, you can insert objects like a box, cyclinder or sphere, in the planning scene to plan trajectories around the object. This can be done by adding an object using the Scene Objects tab in the Motion Planning pane. Below we see Stretch raising its arm with a flat cuboid obstacle in the way. The mobile base allows Stretch to move forward and then back again while raising the arm to avoid the obstacle.

![WithObstacle](https://user-images.githubusercontent.com/97639181/162533356-15955809-f21d-4181-a012-6bca3f48dfc4.gif)


### Planning with MoveIt 2 Using the MoveGroup C++ API

9. If you want to integrate MoveIt 2 into your planning pipeline and want greater control over its various functionalities, using the MoveGroup API is the way to go. Execute the launch file again and go through the comments in the node to understand what's going on. (Press Ctrl+C in the terminal to terminate)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```
10. Follow instructions in this [tutorial](https://github.com/hello-robot/stretch_ros2/blob/feature/hybrid_planning/stretch_moveit_config/movegroup_demo.md) to plan and execute trajectories using the MoveGroup C++ API.

![StowEdited](https://user-images.githubusercontent.com/97639181/166838248-cbfd537b-973e-4fb4-b60c-b5b3c111e02d.gif)

Read the comments for a breakdown of the code
