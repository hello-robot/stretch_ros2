![](../images/banner.png)

## Overview

*hello_helpers* mostly consists of the hello_helpers Python module. This module provides various Python files used across stretch_ros2 that have not attained sufficient status to stand on their own.

## Ported to ROS 2
 - `fit_plane.py`: Fits planes to 3D data
 - `gripper_conversion.py`: Used for converting measurements for the gripper
 - `hello_misc.py`: Various functions, including a helpful Python object with which to create ROS nodes (Partially)
 - `hello_ros_viz.py`: Various helper functions for visualizations using RViz

## Typical Usage

```
import hello_helpers.fit_plane as fp
```
```
import hello_helpers.hello_misc as hm
```
```
import hello_helpers.hello_ros_viz as hr
```

## License

For license information, please see the LICENSE files. 
