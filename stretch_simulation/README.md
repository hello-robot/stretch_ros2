# Stretch Simulation in ROS2

## Setting up URDF

Run the commands below or follow the instruction in the [`stretch_description #updating-the-urdf`](../stretch_description/README.md#updating-the-urdf) README file to set up the URDF meshes.

```shell
source ~/ament_ws/install/setup.bash

python3 -m pip install  -U hello-robot-stretch-urdf

git clone https://github.com/hello-robot/stretch_urdf.git

python3 stretch_urdf/tools/stretch_urdf_ros_update.py --model SE3 --tool eoa_wrist_dw3_tool_sg3

colcon build 
```


## Mujoco

This ROS 2 package uses the [`stretch_mujoco`](https://github.com/hello-robot/stretch_mujoco) repo to interface with Mujoco.

Run the following to start interacting with Stretch in Mujoco using ROS 2:

```shell
sh ./stretch_simulation/stretch_mujoco_driver/setup.sh

cd ~/ament_ws
source ./install/setup.bash
colcon build
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py 
```

## Docker

To run Stretch Simulation in Docker:

Create the container:
```shell
cd stretch_simulation

# This zip file contains scripts and files needed to build a Stretch ament_ws environment:
unzip docker_volume.zip

# Choose one depending on architecture:
export DOCKER_DEFAULT_PLATFORM=linux/amd64/v2 # Apple Silicon
#export DOCKER_DEFAULT_PLATFORM=linux/x86_64/v8 

docker build -t stretch_ros2 .

# Mac or CPU:
docker run -it --name stretch_ros2  --env="DISPLAY=host.docker.internal:0" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged -v ./docker_volume:/root stretch_ros2
# Nvidia GPU:
# docker run -it --gpus=all --name stretch_ros2  --env="DISPLAY=host.docker.internal:0" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged -v ./docker_volume:/root stretch_ros2
```

Open a terminal on the container:
```shell
docker start stretch_ros2
docker exec -it stretch_ros2 bash
```

Run the following on the container:
```shell
cd ~/

git clone https://github.com/hello-robot/stretch_install.git --depth 1 

cp -r ./etc /etc/

usermod -a -G video root

bash env_install.sh

cd ~/ament_ws/src/stretch_ros2
```

To use RViz and Mujoco GUI, you should enable passthrough.

For MacOS:

1. `brew install --cask xquartz`
2. `defaults write org.xquartz.X11 enable_iglx -bool true`
3. `open -a XQuartz`
4. Go to Security Settings -> "Allow connections from network clients"
5. Restart again
6. `open -a XQuartz`
7. `xhost +localhost`
8. Run `Xquartz :0 -listen tcp` and make sure it is not saying `–nolisten tcp` -> TCP is needed for X11 forwarding.
9. You may need to repeat steps 6,7 and 8 if you quit XQuartz.
