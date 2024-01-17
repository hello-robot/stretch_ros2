![](../images/banner.png)

## Overview

*stretch_description* provides materials for a [URDF](http://wiki.ros.org/urdf) kinematic model of the Stretch mobile manipulator.

## Quick View

    ros2 launch stretch_description display.launch.py

## Details

The *meshes directory* contains [STL mesh files](https://en.wikipedia.org/wiki/STL_(file_format)) representing the exterior geometry of various parts of the robot. 

The *urdf directory* contains [xacro files](http://wiki.ros.org/xacro) representing various parts of the robot that are used to generate the robot's URDF. 

stretch_ros2 expects a URDF with the name stretch.urdf to reside within the urdf directory. The file stretch.urdf serves as the URDF for the robot and must be generated. Typically, it is a calibrated urdf file for the particular Stretch robot being used. To generate this file, please read the documentation within stretch_ros2/stretch_calibration. 

The xacro_to_urdf.py will usually only be indirectly run as part of various scripts and launch files within stretch_ros2/stretch_calibration. 

Sometimes a stretch_uncalibrated.urdf file will reside with the urdf directory. This file is typically generated directly from the xacro files without any alterations. 

## Exporting a URDF

Sometimes a URDF is useful outside of ROS, such as for simulations and analysis. Running the *export_urdf.py* script in the urdf directory will export a full URDF model of the robot based on stretch.urdf. 

The exported URDF will be found within an exported_urdf directory. It is also copied to a directory for your specific robot found under ~/stretch_user. The exported URDF includes meshes and controller calibration YAML files. The exported URDF can be visualized using stretch_urdf_show.py, which is part of the stretch_body Python code. 

## Changing the Tool

If you want to generate a new URDF for Stretch (e.g. after attaching a new tool to Stretch), you can:

1. Edit the [`stretch_description.xacro`](./urdf/stretch_description.xacro) file in the Stretch Description package.

   Include the tool you want in the `stretch_description.xacro`. By default, it will include `stretch_gripper.xacro`. Simply comment out
   the old `<xacro:include ...` lines and paste in new ones for your tool. Example code for the other supported tools can be found by
   looking at the files matching `stretch_description_<tool-name>.xacro`. To add your own custom tool, copy your xacro/mesh files into
   the urdf/meshes folders in the Stretch Description package.

1. In a terminal run

   ```
   ros2 run stretch_calibration update_urdf_after_xacro_change
   ```

1. Rebuild the workspace

   ```
   cd ~/ament_ws
   colcon build
   ```

This will create a calibrated URDF called `stretch.urdf` using the most recent calibration parameters. Rviz will now show the new URDF.

## License and Patents

Patents are pending that cover aspects of the Stretch RE1 mobile manipulator.

For license information, please see the LICENSE files. 
