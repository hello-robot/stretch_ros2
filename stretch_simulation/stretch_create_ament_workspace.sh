#!/bin/bash

set -e

if [[ $EUID = 0 ]]; then
   echo "Please run this script without sudo." 
   exit 1
fi 


if [ ! -d  "$HOME/stretch_install" ]; then
git clone https://github.com/hello-robot/stretch_install.git --depth 1 $HOME/stretch_install
fi

LOCALROBOT_NAME="stretch-se3-local"
sudo mkdir -p /etc/hello-robot
echo "HELLO_FLEET_ID=$LOCALROBOT_NAME" | sudo tee /etc/hello-robot/hello-robot.conf

. /etc/hello-robot/hello-robot.conf
echo "###########################################"
echo "NEW INSTALLATION OF USER SOFTWARE"
echo "###########################################"
echo "Update $HOME/.bashrc dotfile..."
echo "" >> $HOME/.bashrc
echo "######################" >> $HOME/.bashrc
echo "# STRETCH BASHRC SETUP" >> $HOME/.bashrc
echo "######################" >> $HOME/.bashrc
echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> $HOME/.bashrc
echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> $HOME/.bashrc
echo "export PATH=\${PATH}:$HOME/.local/bin" >> $HOME/.bashrc
echo "export LRS_LOG_LEVEL=None #Debug" >> $HOME/.bashrc
echo "export PYTHONWARNINGS='ignore:setup.py install is deprecated,ignore:Invalid dash-separated options,ignore:pkg_resources is deprecated as an API,ignore:Usage of dash-separated'" >> $HOME/.bashrc

echo "export _colcon_cd_root=${HOME}/ament_ws" >> $HOME/.bashrc
echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc


echo "Creating repos and stretch_user directories..."
mkdir -p $HOME/.local/bin
mkdir -p $HOME/repos
mkdir -p $HOME/stretch_user
mkdir -p $HOME/stretch_user/log
mkdir -p $HOME/stretch_user/debug
mkdir -p $HOME/stretch_user/maps
mkdir -p $HOME/stretch_user/models
mkdir -p $HOME/stretch_user/$LOCALROBOT_NAME
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_user_params.yaml

echo "robot:
  model_name: SE3" > $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml


echo "Setting up user copy of robot factory data (if not already there) at $HOME/stretch_user/$HELLO_FLEET_ID"

sudo chown -R : $HOME/stretch_user
sudo chmod -R a-x,o-w,+X $HOME/stretch_user


export PATH=${PATH}:$HOME/.local/bin
export HELLO_FLEET_ID=$HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user

echo "Ensuring correct version of params present..."
params_dir_path=$HELLO_FLEET_PATH/$HELLO_FLEET_ID
echo "Checking params directory path: $params_dir_path"

# Define file paths based on the provided directory
user_params="$params_dir_path/stretch_re1_user_params.yaml"
factory_params="$params_dir_path/stretch_re1_factory_params.yaml"
new_config_params="$params_dir_path/stretch_configuration_params.yaml"
new_user_params="$params_dir_path/stretch_user_params.yaml"


echo "###########################################"
echo "CREATING HUMBLE AMENT WORKSPACE at ~/ament_ws"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "humble" ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/humble/setup.bash

if [[ -d ~/ament_ws ]]; then
    echo "You are about to delete and replace the existing ament workspace. If you have any personal data in the workspace, please create a back up before proceeding."
    prompt_yes_no(){
    read -p "Do you want to continue? Press (y/n for yes/no): " x
    if [ $x = "n" ]; then
            echo "Exiting the script."
            exit 1
    elif [ $x = "y" ]; then
            echo "Continuing to create a new ament workspace."
    else
        echo "Press 'y' for yes or 'n' for no."
        prompt_yes_no
    fi
    }
    prompt_yes_no
fi

echo "Downgrade to numpy 1.26.4..."
pip3 install numpy==1.26.4

export PATH=${PATH}:~/.local/bin
echo "Updating rosdep indices..."
rosdep update --include-eol-distros
echo "Deleting ~/ament_ws if it already exists..."
sudo rm -rf ~/ament_ws
echo "Creating the workspace directory..."
mkdir -p ~/ament_ws/src
echo "Cloning the workspace's packages..."
cd ~/ament_ws/src
vcs import --input ~/stretch_install/factory/22.04/stretch_ros2_humble.repos
echo "Fetch ROS packages' dependencies (this might take a while)..."
cd ~/ament_ws/
# The rosdep flags below have been chosen very carefully. Please review the docs before changing them.
# https://docs.ros.org/en/independent/api/rosdep/html/commands.html
rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src
sudo apt remove -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
pip3 cache purge

echo "Install web interface dependencies..."
cd ~/ament_ws/src/stretch_web_teleop
pip3 install -r requirements.txt
npm install --force
npx playwright install
echo "Generating web interface certs..."
cd ~/ament_ws/src/stretch_web_teleop/certificates
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
CAROOT=`pwd` mkcert --install
mkdir -p ~/.local/share/mkcert
rm -rf ~/.local/share/mkcert/root*
cp root* ~/.local/share/mkcert
mkcert ${HELLO_FLEET_ID} ${HELLO_FLEET_ID}.local ${HELLO_FLEET_ID}.dev localhost 127.0.0.1 0.0.0.0 ::1
rm mkcert-v*-linux-amd64
cd ~/ament_ws/src/stretch_web_teleop
touch .env
echo certfile=${HELLO_FLEET_ID}+6.pem >> .env
echo keyfile=${HELLO_FLEET_ID}+6-key.pem >> .env
cd ~/ament_ws/


set +e

echo "###########################################"
echo "INSTALLATION OF USER LEVEL PIP3 PACKAGES"
echo "###########################################"
echo "Clear pip cache"
python3 -m pip cache purge
echo "Upgrade pip3"
python3 -m pip -q install --no-warn-script-location --user --upgrade pip
echo "Install Stretch Body"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-body
echo "Install Stretch Body Tools"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-body-tools
echo "Install Stretch Factory"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-factory
echo "Install Stretch Tool Share"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-tool-share
echo "Install Stretch Diagnostics"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-diagnostics
echo "Install Stretch URDF"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-urdf
echo "Upgrade prompt_toolkit"
python3 -m pip -q install --no-warn-script-location -U prompt_toolkit
pip3 install setuptools==59.6.0
pip3 install numpy==1.26.4
echo "Remove setuptools-scm"
python3 -m pip -q uninstall -y setuptools-scm
echo ""


set -e

sudo add-apt-repository ppa:sweptlaser/python3-pcl
sudo apt update
sudo apt install python3-pcl

cd $HOME/ament_ws
sudo rosdep init
rosdep update
rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src
colcon build
source ./install/setup.bash
