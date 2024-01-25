![](../images/banner.png)

## Overview

*stretch_description* provides assets to create a [URDF](http://wiki.ros.org/urdf) kinematic model of the Stretch mobile manipulator.

## Quick View

To compile the XACROs into a URDF and view it in Rviz, run:

```
ros2 launch stretch_description display.launch.py
```

## Details

The *meshes directory* contains [STL mesh files](https://en.wikipedia.org/wiki/STL_(file_format)) representing the exterior geometry of various parts of the robot. 

The *urdf directory* contains [xacro files](http://wiki.ros.org/xacro) representing various parts of the robot that are used to generate the robot's URDF. 

Stretch's ROS2 packages expects a URDF with the name "stretch.urdf" to reside within the urdf directory. The file "stretch.urdf" serves as the URDF for the robot and must be generated. Typically, it is a calibrated urdf file, unique to the particular Stretch robot being used. This file should already exist on your robot. To regenerate this file, please read the documentation within the stretch_calibration package.

## Exporting a URDF

Sometimes a URDF is useful outside of ROS, such as for simulations and analysis. To export the URDF, run:

```
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
./export_urdf.py
```

Normal output will look like:

```
$ ./export_urdf.sh
Moving previous exported_urdf to exported_urdf_previous...
Creating new exported URDF directories...
Copying the meshes/URDF files to the exported URDF...
Replacing the mesh ROS prefix with relative prefixes...
Copying D435i mesh to the exported URDF...
Copying controller params to the exported URDF...
Copying license to exported URDF...
Copying the exported URDF to the fleet directory...

DONE!
```

The exported URDF will be copied to your robot's calibration data directory (at "~/stretch_user/stretch-yyy-xxxx/exported_urdf/"). It will include the calibrated URDF, meshes, and calibrated controller parameters. Once the URDF has been exported, it can be visualized using a command line tool called `stretch_urdf_viz`.

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
