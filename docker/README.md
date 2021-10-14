# Stretch Docker Documentation for ROS World Workshop

Welcome to our ROS World Workshop! This README is a documentation to go through installation steps to install docker and setup your docker environment for Stretch.

## Installation (Linux)

### Prerequisites

#### Install Docker

Install docker, following the [official installation steps](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and verify your installation.

#### Docker post-install steps

Follow the [official post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) for Linux allow non-root users to manage dockers and verify your post installation.

#### Install nvidia-docker2 (Skip if you don't have an nvidia gpu)

Follow the [official instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian) install nvidia-docker2 and verify your installation.

### Getting Started with Stretch Docker

We provide a pre-built docker image and a set of scripts to easily get you up and running with Stretch on docker! To get your docker running with UI and networking enabled:

1\. Fetch our start_docker script

```bash
wget https://raw.githubusercontent.com/vatanaksoytezer/stretch_ros/pr-docker/docker/scripts/start_docker.sh
```

2\. Start your container from our pre-built image

with an nvidia-gpu:

```bash
chmod +x start_docker.sh && \
./start_docker.sh stretch_roscon ghcr.io/vatanaksoytezer/stretch:roscon
```

without an nvidia-gpu:

```bash
chmod +x start_docker.sh && \
./start_docker.sh stretch_roscon ghcr.io/vatanaksoytezer/stretch:roscon nogpu
```

To open multiple terminals, you can just `./start_docker.sh stretch_roscon` from now on since the docker container is already constructed from the image with the command above.

3\. At this point you should be seeing a terminal window with all the necessary source code pre-built for you. 

Now you can test your Stretch docker by bringing up Stretch in Ignition Gazebo by issuing the following set of commands inside your container:

```bash
ros2 launch stretch_ignition ignition.launch.py
```

Congratulations! You brought up Stretch in Ignition Gazebo! Now you should be seeing Stretch in Ignition Gazebo with an empty world around it:

![Stretch with Ignition Gazebo](media/stretch_empty_world.png)
### Building the docker image from scratch

## Installation (Windows)

### Prerequisites

#### Install WSL2 and Docker

## Installation (MacOS)

## Guided exploration: stretch_moveit_config demo

* Run the following command:

  ```bash
  # Terminal 1
  ros2 launch stretch_moveit_config demo.launch.py
  ```

* Change velocity and acceleration scaling to 1 (esp. on slow machines).
* Show how to plan from current state to (1) random valid goal state or (2) a specific state by moving the interactive marker. The latter may not always work, because some planning groups are kinematically very constrained.
* Repeat the previous step for the `stretch_arm` and `mobile_base_arm` groups.
* Turn on "Query Start State" by checking the checkbox in the Displays panel under "Motion Planning > Planning Request".
* Turn on "Loop animation" by checking the checkbox in the Displays panel under "Trajectory."
* Explain structure of `stretch_moveit_config`; show contents of some configuration files and maybe SRDF/URDF .
  
## Guided exploration: Ignition Demo World

* Run the following commands in three different terminal windows:

  ```bash
  # Terminal 1
  ros2 launch stretch_ignition ignition.launch.py
  # Terminal 2
  ros2 launch stretch_moveit_config demo_ignition.launch.py
  # Terminal 3
  ros2 launch stretch_roscon_demos move_group_interface_demo.launch.py
  ```

## Guided exploration: Pick and place with whole body planning

* Run the following commands in three different terminal windows:

  ```bash
  # Terminal 1
  ros2 launch stretch_ignition ignition.launch.py aws:=true
  # Terminal 2
  ros2 launch stretch_moveit_config demo_ignition.launch.py
  # Terminal 3
  ros2 launch stretch_roscon_demos move_group_interface_demo.launch.py
  ```

## Guided exploration: Pick and place with MoveIt Task Constructor

* Run the following commands in two different terminal windows:

  ```bash
  # Terminal 1
  ros2 launch pick_place_task demo.launch.py
  # Terminal 2
  ros2 launch pick_place_task pick_place_demo.launch.py 
  ```

* Explain MTC panel. Show different solutions.
* Explain structure of the code.

## Editing the source code during the Workshop

Our docker image comes with a pre-built installation of VS Code, vim and nano. You can also install your editor of choice via apt-get.
