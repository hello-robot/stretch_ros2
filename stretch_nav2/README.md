![](../images/banner.png)

## Overview

The *stretch_nav2* package provides the standard ROS 2 navigation stack (Nav2) with its launch files. This package utilizes slam_toolbox and Nav2 to drive Stretch around a mapped space. Running this code will require the robot to be untethered. We recommend stowing the arm while running navigation on the robot.

## Quickstart

The first step is to map the space that the robot will navigate in. The `offline_mapping.launch.py` will enable you to do this. First, run:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the joystick (see instructions below for using a keyboard) to teleoperate the robot around. Avoid sharp turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, open a new terminal and run the following commands to save the map to the `stretch_user/` directory.

```bash
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

**NOTE**: The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

**Tip**: For a quick sanity check, you can inspect the saved map using a pre-installed tool called Eye of Gnome (eog) by running the following command:

```bash
eog ${HELLO_FLEET_PATH}/maps/<map_name>.pgm
```

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

A new RViz window should pop up with a `Startup` button in a menu at the bottom left of the window. Press the `Startup` button to kick-start all navigation related lifecycle nodes. Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location on the map does not match the robot's location in the real space. To correct this, from the top bar of Rviz, use `2D Pose Estimate` to lay an arrow down roughly where the robot is located in the real space. This gives an initial estimate of the robot's location to AMCL, the localization package. AMCL will better localize the robot once we pass the robot a `2D Nav Goal`.

In the top bar of Rviz, use `2D Nav Goal` to lay down an arrow where you'd like the robot to navigate. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

**Tip**: If navigation fails or the robot becomes unresponsive to subsequent goals through RViz, you can still teleoperate the robot using the Xbox controller.

### Teleop using a Joystick Controller

The launch files expose the launch argument "teleop_type". By default, this argument is set to "joystick", which launches joystick teleop in the terminal with the xbox controller that ships with Stretch RE1. The xbox controller utilizes a dead man's switch safety feature to avoid unintended movement of the robot. This is the switch located on the front left side of the controller marked "LB". Keep this switch pressed and translate or rotate the base using the joystick located on the right side of the xbox controller.

If the xbox controller is not available, the following commands will launch mapping and navigation, respectively, with keyboard teleop:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=keyboard
```
or
```bash
ros2 launch stretch_nav2 navigation.launch.py teleop_type:=keyboard map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

## License

For license information, please see the LICENSE files.
