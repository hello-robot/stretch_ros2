![](../images/banner.png)

## Overview

*stretch_moveit2* configures MoveIt2 for Stretch RE1. The MoveIt Motion Planning Framework makes whole body planning, manipulation, 3D perception, control/navigation, and more available on Stretch.


## Drag & Drop Rviz Planning

### Demo with Simulated Robot

To experiment with the planning capabilities of MoveIt2 on Stretch, you can run a demo _without_ Stretch hardware. The fake robot will be backed by [ros2_control controllers](https://github.com/ros-controls/ros2_controllers) that provide MoveIt2 with the action server required for planning, however, the 'execution' button in Rviz will not be able to execute plans that include mobile base motion.

```
ros2 launch stretch_moveit_config demo.launch.py
```

This will allow you to move the robot around using interactive markers and create plans between poses.

### Hardware Integration

TBD

## License

For license information, please see the LICENSE files.
