![](../images/banner.png)

## Overview

The *stretch_rtabmap* package provides Real Time Appearance Based Mapping (RTABMap) features. RTABmap allows mapping the environment in 3D using the RGBD camera. This is desirable if the environment is cluttered and rife with obstacles that the 2D lidar might fail to catch. Additionally, RTABmap can also be used to localize the robot while navigating using the RGBD camera. Running the code will require the robot to be untethered. We recommend stowing the arm while running navigation on the robot.

## Quickstart

The first step is to map the space that the robot will navigate in. The `visual_mapping.launch.py` launch file enables you to do this. First, run:

```bash
ros2 launch stretch_rtabmap visual_mapping.launch.py
```

<p align="center">
  <img height=500 src="https://github.com/hello-robot/stretch_ros2/assets/97639181/3f01a470-5fd1-4e97-80dc-65ea39832afa"/>
</p>

Rviz will show the robot and the visual map that is being constructed. With the terminal open, use the joystick (see instructions below for using a keyboard) to teleoperate the robot around. Since mapping is being achieved visually using the RGBD camera, it is necessary to pan the head camera around to capture landmarks. Avoid sharp turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, open a new terminal and run the following commands to save the map to the `stretch_user/` directory. 

!!! note
    This step is necessary only if you want to save a 2D map for use in other applications. If not, RTABmap saves the 3D and 2D maps by default in its database directory `~/.ros/rtabmap.db`.

```bash
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

!!! note
    The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

!!! tip
    For a quick sanity check, you can inspect the saved map using a pre-installed tool called Eye of Gnome (eog) by running the following command:

```bash
eog ${HELLO_FLEET_PATH}/maps/<map_name>.pgm
```

Next, let's navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_rtabmap visual_navigation.launch.py
```

<p align="center">
  <img height=500 src="https://github.com/hello-robot/stretch_ros2/assets/97639181/b9cd7189-2d82-41b0-b5ba-4f12cac63c4b"/>
</p>

A new RViz window should pop and show the robot in the previously mapped space, however, it's likely that the robot's location on the map does not match the robot's location in the real space. To correct this, from the top bar of Rviz, use `2D Pose Estimate` to lay an arrow down roughly where the robot is located in the real space. This gives an initial estimate of the robot's location to RTABmap.

In the top bar of Rviz, use `2D Nav Goal` to lay down an arrow where you'd like the robot to navigate. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

!!! tip
    If navigation fails or the robot becomes unresponsive to subsequent goals through RViz, you can still teleoperate the robot using the Xbox controller.

### Teleop using a Joystick Controller or Keyboard

The launch files expose the launch argument "teleop_type". By default, this argument is set to "joystick", which launches joystick teleop in the terminal with the xbox controller that ships with Stretch RE1. The xbox controller utilizes a dead man's switch safety feature to avoid unintended movement of the robot. This is the switch located on the front left side of the controller marked "LB". Keep this switch pressed and translate or rotate the base using the joystick located on the right side of the xbox controller.

If the xbox controller is not available, after launching mapping or navigation, execute the following command in a new terminal to enable keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=stretch/cmd_vel
```

## License

For license information, please see the LICENSE files.
