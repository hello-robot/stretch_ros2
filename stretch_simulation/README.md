# Stretch Simulation in ROS2

Use this package to use ROS2 with Stretch in Mujoco.

## System Requirements

It is recommended to run this package on an Ubuntu 22.04 workstation with an Nvidia graphics card or a WSL2 environment with GPU acceleration. 

Minimum: 16GB of RAM. Recommended: 32GB of RAM.

This package is not supported on Metal (MacOS) at this time due to the lack of GPU acceleration and OpenGL 1.5+ support in Docker, and slow performance in UTM with a virtual machine.

## Nav2

First go through the [Getting Started](#getting-started) guide to set up your environment.

### Mapping

This section is similar to https://docs.hello-robot.com/0.3/ros2/navigation_stack/#mapping, but uses the Stretch simulation environment.

To map the simulated environment, run the following:

```shell
# Terminal 1: Slam Toolbox
ros2 launch stretch_nav2 online_async_launch.py use_sim_time:=true 
# Optional: Navigation bringup:
ros2 launch stretch_nav2 navigation.launch.py use_slam:=true use_sim_time:=true use_rviz:=true teleop_type:=none

# Terminal 2: Stretch Mujoco Driver
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true mode:=navigation

# Terminal 3: Keyboard Teleop
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/stretch/cmd_vel
```

To save your map, run:

```sh
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

### Navigation

To run navigation on a previously generated map, run:

```sh
# Terminal 1: Stretch Mujoco Driver
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true use_rviz:=false mode:=navigation

# Terminal 2: Navigation
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger

ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml use_sim_time:=true use_rviz:=true teleop_type:=none
```

You may want to dynamically reduce the cost_map inflation radius for most Robocasa environments:

```shell
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap  inflation_layer.inflation_radius

ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25
ros2 param set /local_costmap/local_costmap  inflation_layer.inflation_radius 0.25
```

## Stretch Drivers

Simulation Drivers interface with the simulator to read and write data.

Simulation Drivers mimic the StretchDriver in `stretch_core`, which talks to the real robot. 

You could display all the launch options available to the Stretch Mujoco Driver using: `ros2 launch stretch_simulation stretch_mujoco_driver.launch.py --show-args`:

```
    'broadcast_odom_tf':
        Whether to broadcast the odom TF. Valid choices are: ['True', 'False']
        (default: 'True')

    'fail_out_of_range_goal':
        Whether the motion action servers fail on out-of-range commands. Valid choices are: ['True', 'False']
        (default: 'False')

    'mode':
        The mode in which the ROS driver commands the robot. Valid choices are: ['position', 'navigation', 'trajectory', 'gamepad']
        (default: 'position')

    'use_rviz':
        One of: ['true', 'false']
        (default: 'true')

    'use_mujoco_viewer':
        One of: ['true', 'false']
        (default: 'true')

    'use_cameras':
        One of: ['true', 'false']
        (default: 'false')

    'use_robocasa':
        One of: ['true', 'false']
        (default: 'true')

    'robocasa_task':
        no description given
        (default: 'PnPCounterToCab')

    'robocasa_layout':
        One of: ['Random', 'One wall', 'One wall w/ island', 'L-shaped', 'L-shaped w/ island', 'Galley', 'U-shaped', 'U-shaped w/ island', 'G-shaped', 'G-shaped (large)', 'Wraparound']
        (default: 'Random')

    'robocasa_style':
        One of: ['Random', 'Industrial', 'Scandanavian', 'Coastal', 'Modern_1', 'Modern_2', 'Traditional_1', 'Traditional_2', 'Farmhouse', 'Rustic', 'Mediterranean', 'Transitional_1', 'Transitional_2']
        (default: 'Random')

```

You can also set the node's argument`arguments=["--ros-args", "--log-level", "debug"]` in the launch file to display Sim-to-Real time and other useful debug information.

## Getting Started

You should go through all the sections in Getting Started to run this package correctly.

### Install ROS2 Humble

The commands below are taken from this guide: https://docs.ros.org/en/humble/index.html

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install ros-humble-desktop ros-dev-tools rviz python3-pip

source /opt/ros/humble/setup.bash
```

### Setting up `ament_ws`

If you are not running this package on a robot NUC (which is _not_ [recommended](#system-requirements)), you will need to set up a ROS2 environment similar to the environment that ships with Stretch.

Please run these commands to install the environment. This will delete the existing `~/ament_ws` directory, so please proceed with caution.

```sh
cd stretch_simulation

git clone https://github.com/hello-robot/stretch_install.git --depth 1 ~/stretch_install

unzip ubuntu2204_ament_ws_files.zip

sudo cp -r ./ubuntu2204_ament_ws_files/etc/* /etc/
cp -r ./ubuntu2204_ament_ws_files/stretch_user ~/

bash ./ubuntu2204_ament_ws_files/env_install.sh

rm -rf ./ubuntu2204_ament_ws_files

cd ~/ament_ws

rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src

colcon build

source ./install/setup.bash
```


### Setting up URDF

Run the commands below or follow the instruction in the [`stretch_description #updating-the-urdf`](../stretch_description/README.md#updating-the-urdf) README file to set up the URDF meshes.

```shell
source ~/ament_ws/install/setup.bash

python3 -m pip install -U hello-robot-stretch-urdf

git clone https://github.com/hello-robot/stretch_urdf.git --depth 1 /tmp/stretch_urdf

python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py --model SE3 --tool eoa_wrist_dw3_tool_sg3

cd ~/ament_ws

colcon build # Copy new files
```


### Mujoco

This ROS 2 package includes nodes and launch files that use the [`stretch_mujoco`](https://github.com/hello-robot/stretch_mujoco) repo to interface with Mujoco.

Run the following, after having done the previous ament_ws setup steps, to start interacting with Stretch in Mujoco using ROS 2:

```shell
source ~/ament_ws/install/setup.bash
sh ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/setup.sh

cd ~/ament_ws
source ./install/setup.bash
colcon build
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation
```
