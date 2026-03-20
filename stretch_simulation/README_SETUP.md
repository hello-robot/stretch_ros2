# Setting up Stretch Simulation

You should go through all the sections in this setup guide to run this package correctly.

> NOTE: If you are running on a Stretch robot, you can skip to [Setting up Mujoco](#setting-up-mujoco-15-minutes)

> NOTE: If you are on Linux or Windows, you can use the [Docker setup](README_DOCKER.md) to get started using Docker with hardware acceleration.

Estimated install time: `~1-2hrs`.


## Install ROS2 Humble (10 minutes)

> NOTE: Please do not run this step if you are running on a Stretch robot.

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

## Setting up `ament_ws` (1 hour)

> NOTE: Please do not run this step if you are running on a Stretch robot.

If you are not running this package on a robot NUC (which is _not_ [recommended](#system-requirements)), you will need to set up a ROS2 environment similar to the environment that ships with Stretch.

Please run these commands to install the environment. This will delete the existing `~/ament_ws` directory, so please proceed with caution.

First you should install `NodeJS>=21.x` and `npm` if you don't already have them:
```shell
curl -sL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt install -y nodejs
```

```sh
INSTALL_AMENT_WS_URL=https://raw.githubusercontent.com/hello-robot/stretch_ros2/refs/heads/humble/stretch_simulation/stretch_create_ament_workspace.sh
curl -sL $INSTALL_AMENT_WS_URL > /tmp/stretch_create_ament_workspace.sh 
bash /tmp/stretch_create_ament_workspace.sh

# Optional: add source install/setup.bash to .bashrc:
echo 'source ~/ament_ws/install/setup.bash' >> ~/.bashrc
```


> Note: If you run into a colcon build error: "fatal error: numpy/ndarrayobject.h: No such file or directory", run `sudo ln -s ~/.local/lib/python3.10/site-packages/numpy/core/ /usr/include/numpy` to resolve it.


A successful `ament_ws` setup will look like this:
```
$ ls ~/ament_ws/src
audio_common  realsense-ros  respeaker_ros2  ros2_numpy  rosbridge_suite  sllidar_ros2  stretch_ros2  stretch_tutorials  stretch_web_teleop  tf2_web_republisher_py
```

## Setting up URDF (15 minutes)

Run the commands below or follow the instruction in the [`stretch_description #updating-the-urdf`](../stretch_description/README.md#updating-the-urdf) README file to set up the URDF meshes.

```shell
source ~/ament_ws/install/setup.bash

python3 -m pip install -U hello-robot-stretch-urdf

git clone https://github.com/hello-robot/stretch_urdf.git --depth 1 /tmp/stretch_urdf

python3 -m pip install hello-robot-stretch-body

python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py
python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py --ros2_rebuild
```

A successful URDF update will look like this:
```
$ ls ~/ament_ws/src/stretch_ros2/stretch_description/urdf/
d405                             stretch_aruco.xacro     stretch_description_SE3_eoa_wrist_dw3_tool_nil.xacro          stretch_head_nav_cam.xacro        stretch_tool_sg3.xacro
d435i                            stretch_base_imu.xacro  stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro          stretch_laser_range_finder.xacro  stretch_tool_tablet_12in.xacro
export_urdf_license_template.md  stretch_d405_sg3.xacro  stretch_description_SE3_eoa_wrist_dw3_tool_tablet_12in.xacro  stretch_main.xacro                stretch_uncalibrated.urdf
export_urdf.sh                   stretch_d435i.xacro     stretch_description.xacro                                     stretch_respeaker.xacro           stretch_wrist_dw3.xacro
```

## Setting up Mujoco (15 minutes)

This ROS 2 package includes nodes and launch files that use the [`stretch_mujoco`](https://github.com/hello-robot/stretch_mujoco) repo to interface with Mujoco.

Run the following, after having done the previous ament_ws setup steps, to start interacting with Stretch in Mujoco using ROS 2:

```shell
pip3 install --upgrade pip #This is important after a fresh install of Ubuntu, for edittable installation of dependencies

source ~/ament_ws/install/setup.bash
# This script is interactive, it will ask you if you want to install robocasa model files:
bash ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/setup.sh

pip install PyOpenGL==3.1.4 # Fixes AttributeError: module 'OpenGL.EGL' has no attribute 'EGLDeviceEXT'

cd ~/ament_ws
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation
```


## Setting up Stretch Web Teleop

Make sure you've already completed everything under [Setting up `ament_ws`](#setting-up-ament_ws) above.

Run the following commands to get IK for the gripper working:
```shell
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
cp ./stretch_uncalibrated.urdf stretch.urdf

sudo apt install rpl
./export_urdf.sh # It's okay if it fails on calibrated params

mkdir -p $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
cp -r ./exported_urdf/* $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
```
