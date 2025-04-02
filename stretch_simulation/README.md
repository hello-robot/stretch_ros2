# Stretch Simulation in ROS2

## Mujoco

This ROS 2 package uses the [`stretch_mujoco`](https://github.com/hello-robot/stretch_mujoco) repo to interface with Mujoco.

Run the following to start interacting with Stretch in Mujoco using ROS 2:

```
sh ./stretch_simulation/stretch_mujoco_driver/setup.sh

cd ~/ament_ws
source ./install/setup.bash
colcon build
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py 
```