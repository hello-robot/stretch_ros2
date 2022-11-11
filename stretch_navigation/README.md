![](../images/banner.png)

## Overview

The *stretch_navigation* package provides the standard ROS 2 navigation stack (Nav2) as three launch files. This package utilizes slam_toolbox and Nav2 to drive Stretch around a mapped space. Running this code will require the robot to be untethered.

## Quickstart

The first step is to map the space that the robot will navigate in. The `mapping.launch.py` will enable you to do this. First run:

```bash
ros2 launch stretch_navigation mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the joystick or a keyboard to teleoperate the robot around. Avoid sharp turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, you can run the following commands to save the map to `stretch_navigation/` and `stretch_user/`.

```bash
mkdir ~/ament_ws/src/stretch_ros2/stretch_navigation/maps
ros2 run nav2_map_server map_saver_cli -f ~/stretch_navigation/maps/<map_name>
```

If you want to preserve the generated map or use it across different ROS distributions, copy the files in the `stretch_user` directory.

```bash
mkdir -p ~/stretch_user/maps
cp ~/ament_ws/src/stretch_ros2/stretch_navigation/maps/* ~/stretch_user/maps
```

The `<map_name>` does not include an extension. Map_saver will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_navigation navigation.launch map:=~/ament_ws/src/stretch_ros2/stretch_navigation/maps/<map_name>.yaml
```

Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location in the map does not match the robot's location in the real space. In the top bar of Rviz, use 2D Pose Estimate to lay an arrow down roughly where the robot is located in the real space. AMCL, the localization package, will better localize our pose once we give the robot a 2D Nav Goal. In the top bar of Rviz, use 2D Nav Goal to lay down an arrow where you'd like the robot to go. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior: spinning around 180 degrees in place or backing up.

It is also possible to send 2D Pose Estimates and Nav Goals programatically. In your own launch file, you may include `navigation.launch` to bring up the navigation stack. Then, you can send `move_base_msgs::MoveBaseGoal` messages in order to navigate the robot programatically.

<!-- ### Running in Simulation

To perform mapping and navigation in the Gazebo simulation of Stretch, substitute the `mapping_gazebo.launch` and `navigation_gazebo.launch` launch files into the commands above. The default Gazebo environment is the Willow Garage HQ. Use the "world" ROS argument to specify the Gazebo world within which to spawn Stretch.

```bash
roslaunch stretch_navigation mapping_gazebo.launch gazebo_world:=worlds/willowgarage.world
``` -->

### Teleop using a Joystick Controller

The launch files expose the launch argument "teleop_type". By default, this argument is set to "joystick", which launches joystick teleop in the terminal with the xbox controller that ships with Stretch RE1. The xbox controller utilizes a dead man's switch safety feature to avoid unintended movement of the robot. This is the switch located on the front left side of the controller marked "LB". Keep this switch pressed and translate or rotate the base using the joystick located on the right side of the xbox controller.

If the xbox controller is not available, the following command will launch mapping with keyboard teleop:

```bash
ros2 launch stretch_navigation mapping.launch teleop_type:=keyboard
```

<!-- ### Using ROS Remote Master

If you have set up [ROS Remote Master](https://docs.hello-robot.com/untethered_operation/#ros-remote-master) for [untethered operation](https://docs.hello-robot.com/untethered_operation/), you can use Rviz and teleop locally with the following commands:

```bash
# On Robot
roslaunch stretch_navigation mapping.launch rviz:=false teleop_type:=none

# On your machine, Terminal 1:
rviz -d `rospack find stretch_navigation`/rviz/mapping.launch
# On your machine, Terminal 2:
roslaunch stretch_core teleop_twist.launch teleop_type:=keyboard # or use teleop_type:=joystick if you have a controller
``` -->

## License

For license information, please see the LICENSE files.
