#!/bin/sh

SCRIPT_DIR=$(dirname "$0")

cd "$SCRIPT_DIR"

mkdir dependencies

cd dependencies

git clone https://github.com/hello-robot/stretch_mujoco.git --depth 1

pip install -e ./stretch_mujoco

cd ~/ament_ws

rosdep install -i --from-path src --rosdistro humble -y

colcon build

source ./install/setup.bash

echo "Done. You can now use 'ros2 launch stretch_simulation <launch_file_name.launch.py>'."
