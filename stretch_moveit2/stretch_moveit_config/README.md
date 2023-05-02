![](../images/banner.png)

## Overview

*stretch_moveit_config* configures MoveIt 2 for Stretch. The MoveIt Motion Planning Framework makes whole body planning, manipulation, 3D perception, control/navigation, and more available on Stretch.

## Quickstart

Before proceeding, it's a good idea to home the robot.
```bash
stretch_robot_home.py
```

To launch RViz with MoveIt 2, run the following command. (Press Ctrl+C in the terminal to terminate):
```bash
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

Alternatively, if you want to integrate MoveIt2 into your planning pipeline and want greater control over its various functionalities, using the MoveGroup API is the way to go:
```bash
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

## Tutorials

For instructions on working with MoveIt 2 on Stretch, go through the tutorials [here](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/moveit_basics/).

MoveIt 2 Python API support is currently under development.
