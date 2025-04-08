# Stretch Simulation in ROS2

Use this package to use ROS2 with Stretch in Mujoco.

## System Requirements

It is recommended to run this package on an Ubuntu 22.04 workstation with an Nvidia graphics card or a WSL2 environment with GPU acceleration.

This package is not supported on Metal (MacOS) at this time due to the lack of GPU acceleration and OpenGL 1.5+ support in Docker, and slow performance in UTM with a virtual machine.

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

unzip ubuntu2204_ament_ws_files.zip

cd ubuntu2204_ament_ws_files

sudo cp -r ./etc/* /etc/
cp -r ./stretch_user ~/

git clone https://github.com/hello-robot/stretch_install.git --depth 1 ~/stretch_install

bash env_install.sh

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
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py 
```
