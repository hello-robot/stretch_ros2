![](../images/banner.png)

## Overview

*stretch_description* is the ROS interface to the [URDF](http://wiki.ros.org/urdf) kinematic model of the Stretch mobile manipulator. The assets that build up a URDF (e.g. mesh files and XACROs) are compiled and copied into this package. These assets can be found in the [Stretch URDF](https://github.com/hello-robot/stretch_urdf) repository.

## Quick View

To view the URDF in Rviz, run:

```
ros2 launch stretch_description display.launch.py
```

## Details

The *meshes directory* contains [STL mesh files](https://en.wikipedia.org/wiki/STL_(file_format)) representing the exterior geometry of various parts of the robot. 

The *urdf directory* contains [xacro files](http://wiki.ros.org/xacro) representing various parts of the robot that are used to generate the robot's URDF. 

Stretch's ROS2 packages expects a URDF with the name "stretch.urdf" to reside within the urdf directory. The file "stretch.urdf" serves as the URDF for the robot and must be generated. Typically, it is a calibrated urdf file, unique to the particular Stretch robot being used. This file should already exist on your robot. To regenerate this file, please read the documentation within the stretch_calibration package.

## Updating the URDF

Your Stretch robot should come with an URDF pre-configurated in this repo. If not, you can update it by cloning [stretch_urdf](https://github.com/hello-robot/stretch_urdf) and running a tool to pull an updated version with the correct tooling. If the above step does not work because no URDF is present, then you should try doing this.

```
git clone https://github.com/hello-robot/stretch_urdf.git
python stretch_urdf/tools/stretch_urdf_ros_update.py
```

You will see terminal output like this:
```
$ python tools/stretch_urdf_ros_update.py 
For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

Robot Model Name = SE3
Robot Tool Name = eoa_wrist_dw3_tool_sg3
Found Stretch URDF files at = /home/hello-robot/.local/lib/python3.10/site-packages/stretch_urdf/SE3
Found ROS_DISTRO = humble
Stretch Description Package Path = /home/hello-robot/ament_ws/src/stretch_ros2/stretch_description
```
If this shows the correct path, follow the instructions in the script to download it.

After this, you will need to follow the instructions in [stretch_calibration](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_calibration) to get a calibrated URDF.

Lastly, rebuild the workspace for the new URDF to take effect.

```
cd ~/ament_ws
colcon build
```

## Exporting a URDF

Sometimes a URDF is useful outside of ROS, such as for simulations and analysis. To export the URDF, run:

```
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
./export_urdf.sh
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

## License and Patents

Patents are pending that cover aspects of the Stretch RE1 mobile manipulator.

For license information, please see the LICENSE files. 
