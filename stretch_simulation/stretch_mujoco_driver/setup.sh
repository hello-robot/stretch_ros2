#!/bin/sh

SCRIPT_DIR=$(dirname "$0")

set -e

cd "$SCRIPT_DIR"

mkdir -p dependencies

cd dependencies

# Install stretch_mujoco and robocasa:
git clone https://github.com/hello-robot/stretch_mujoco.git --depth 1

cd stretch_mujoco

git submodule update --init

pip install -e ".[robocasa]"

pip install -e "third_party/robocasa"
pip install "third_party/robosuite"
python3 third_party/robosuite/robosuite/scripts/setup_macros.py
python3 third_party/robocasa/robocasa/scripts/setup_macros.py
python3 third_party/robocasa/robocasa/scripts/download_kitchen_assets.py

# Colcon Build:
cd ~/ament_ws

colcon build

source ./install/setup.bash

echo "Done. You can now use 'ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation'."
