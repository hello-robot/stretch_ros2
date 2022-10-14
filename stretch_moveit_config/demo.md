![](../images/banner.png)

## Overview

MoveIt 2 is a whole-body motion planning framework for mobile manipulators that allows planning pose and joint goals in environments with and without obstacles. Stretch being a mobile manipulator is uniquely well-suited to utilize the planning capabilities of MoveIt 2 in different scenarios.

## Motivation

Stretch has a kinematically simple 3 DoF arm (+2 with DexWrist) that is suitable for pick and place tasks of varied objects. Its mobile base provides it with 2 additional degrees of freedom that afford it more manipulability and also the ability to move around freely in its environment. To fully utilize these capabilities, we need a planner that can plan for both the arm and the mobile base at the same time. With MoveIt 2 and ROS 2, it is now possible to achieve this, empowering users to plan more complicated robot trajectories in difficult and uncertain environments.

## Demo with Stretch Robot

### Installing ROS 2 on Ubuntu 20.04
By default, Stretch RE1 ships with Ubuntu 18.04, ROS Melodic, and Python2 packages. The following steps will install a second operating system with Ubuntu 20.04, ROS Noetic, ROS 2 Galactic (where MoveIt 2 is supported), and Python3 packages. If you already have an Ubuntu 20.04 partition on the robot set up with the correct ROS workspaces, skip to the next section.

If your robot is an RE1 and you have not upgraded the software stack recently, follow the steps in this [link](https://docs.hello-robot.com/0.2/stretch-install/docs/robot_install/) to perform a fresh robot installation. We recommend backing up important data before you begin.

### Installing MoveIt 2 and Its Dependencies

If you performed the above steps successfully, you don't need to follow the steps in this section as the install scripts take care of satisfying all dependencies you need to run MoveIt 2 on your robot. However, if you already had a stable ROS 2 installation running but were missing the most up to date workspaces, follow the steps in this [link](https://docs.hello-robot.com/0.2/stretch-install/docs/ros_workspace/#ubuntu-2004) to update your ROS workspaces. We recommend backing up important data before you begin as this step replaces your existing workspace with a new one.

### Planning with MoveIt 2 Using RViz
Before we proceed, it's always a good idea to home the robot first by running the following script so that we have the correct joint positions being published on the /joint_states topic. This is necessary for planning trajectories on Stretch with MoveIt.

```
stretch_robot_home.py
```

The easiest way to run MoveIt 2 on your robot is through RViz. With RViz you can plan, visualize, and also execute trajectories for various planning groups on your robot. To launch RViz with MoveIt 2, run the following command. (Press Ctrl+C in the terminal to terminate)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

Follow instructions in this [tutorial](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/moveit_rviz_demo.md) to plan and execute trajectories using the interactive markers in RViz.

Use the interactive markers to drag joints to desired positions or go to the manipulation tab in the Motion Planning pane to fine-tune joint values using the sliders. Next, click the 'Plan' button to plan the trajectory. If the plan is valid, you should be able to execute the trajectory by clicking the 'Execute' button. Below we see Stretch raising its arm without any obstacle in the way.

![WithoutObstacle](https://user-images.githubusercontent.com/97639181/162533340-dec4232b-617c-4b90-b4e1-a24fd3027baa.gif)

To plan with obstacles, you can insert objects like a box, cyclinder or sphere, in the planning scene to plan trajectories around the object. This can be done by adding an object using the Scene Objects tab in the Motion Planning pane. Below we see Stretch raising its arm with a flat cuboid obstacle in the way. The mobile base allows Stretch to move forward and then back again while raising the arm to avoid the obstacle.

![WithObstacle](https://user-images.githubusercontent.com/97639181/162533356-15955809-f21d-4181-a012-6bca3f48dfc4.gif)


### Planning with MoveIt 2 Using the MoveGroup C++ API

If you want to integrate MoveIt 2 into your planning pipeline and want greater control over its various functionalities, using the MoveGroup API is the way to go. Execute the launch file again and go through the comments in the node to understand what's going on. (Press Ctrl+C in the terminal to terminate)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

Follow instructions in this [tutorial](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/moveit_movegroup_demo.md) to plan and execute trajectories using the MoveGroup C++ API.

![StowEdited](https://user-images.githubusercontent.com/97639181/166838248-cbfd537b-973e-4fb4-b60c-b5b3c111e02d.gif)

Read the comments in the [code](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_moveit_config/src/movegroup_test.cpp) for a breakdown of the node.
