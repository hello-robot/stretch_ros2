# Docker Setup for Stretch Simulation with Nvidia GPU Support

This guide provides instructions for setting up and running the Stretch Simulation environment in Docker with Nvidia GPU acceleration.

## Prerequisites

Note: This tutorial was written for Ubuntu 22.04 (or compatible Linux distribution). It is possible to run this Dockerfile on Windows or MacOS, however, this tutorial does not cover enabling hardware acceleration for those platforms.

- Nvidia GPU with recent drivers installed
- Docker installed on your system
- Minimum 16GB RAM (32GB recommended)

## Quick Setup

The following commands will install the Nvidia Container Toolkit, test GPU access, build the Docker image, and run the container:

1. Follow [Install Docker](#1-install-docker) and [Install Nvidia Container Toolkit](#2-install-nvidia-container-toolkit) to set up your system.
2. Run the following commands:

**Note:** The build process may take 1-2 hours depending on your internet connection and system performance.


```bash
# 1. Install Nvidia Container Toolkit (Ubuntu only)
make install-nvidia-toolkit

# 2. Test GPU access
make test-gpu

# 3. Build the Docker image (takes 1-2 hours)
make build

# 4. Run the container
make run
```

After you have the container running, you can follow the [main README](./README.md) for commands to run the simulation.

If you would like to manually go through the setup steps, please follow the instructions starting at [Build the Docker Image](#3-build-the-docker-image).

## 1. Install Docker

If you don't have Docker installed, run:

```bash
# Update package index
sudo apt-get update

# Install dependencies
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add your user to the docker group (optional, to run docker without sudo)
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Install Nvidia Container Toolkit

Note: This tutorial was written for Ubuntu 22.04 (or compatible Linux distribution). It is possible to run this Dockerfile on Windows or MacOS, however, this tutorial does not cover enabling hardware acceleration for those platforms.

The Nvidia Container Toolkit allows Docker containers to access your GPU for hardware acceleration.

```bash
# Configure the production repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package list
sudo apt-get update

# Install the Nvidia Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use Nvidia runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker daemon
sudo systemctl restart docker
```

### Verify Nvidia Container Toolkit Installation

Test that GPU access works in Docker:

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

You should see your GPU information displayed.


## 3. Build the Docker Image

Navigate to the stretch_simulation directory and build the image:

```bash
cd /path/to/stretch_ros2/stretch_simulation
docker build -t stretch-simulation:latest .
```

**Note:** The build process may take 1-2 hours depending on your internet connection and system performance.

**Technical Note:** The Dockerfile sets the `DOCKER_BUILD=1` environment variable when running the workspace setup script. This enables Docker mode which:
- Bypasses sudo checks (running as root is expected in Docker)
- Skips interactive prompts for automated builds
- Uses the local script instead of downloading from the internet

The same `stretch_create_ament_workspace.sh` script works in both Docker and manual modes based on the `DOCKER_BUILD` environment variable.

## 4. Run the Docker Container

### Basic Usage

Run the container with GPU support and X11 forwarding for GUI applications:

```bash
# Allow X11 connections from Docker (Ubuntu Only)
xhost +local:docker

# Run the container
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --network=host \
    --privileged \
    stretch-simulation:latest
```

### Run with Persistent Storage

To persist maps and other data:

```bash
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="$HOME/stretch_data:/root/stretch_user:rw" \
    --network=host \
    --privileged \
    stretch-simulation:latest
```

### Run Specific Launch Commands

Launch the Mujoco driver directly:

```bash
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="MUJOCO_GL=egl" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --network=host \
    --privileged \
    stretch-simulation:latest \
    ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation use_mujoco_viewer:=true
```

## 5. Using Docker Compose (Optional)

Create a `docker-compose.yml` file for easier management:

```yaml
version: '3.8'

services:
  stretch-simulation:
    image: stretch-simulation:latest
    container_name: stretch-sim
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - MUJOCO_GL=egl
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:rw
      - ${HOME}/stretch_data:/root/stretch_user:rw
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    stdin_open: true
    tty: true
```

Run with:

```bash
xhost +local:docker
docker compose up
```

## 6. Common Usage Examples

### Navigation with Pre-mapped Scene

```bash
# Terminal 1: Launch Mujoco driver
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    use_mujoco_viewer:=true \
    mode:=navigation \
    use_rviz:=false \
    robocasa_layout:='G-shaped' \
    robocasa_style:=Modern_1

# Terminal 2: Launch navigation
ros2 launch stretch_nav2 navigation.launch.py \
    map:=/root/ament_ws/src/stretch_ros2/stretch_simulation/maps/gshaped_modern1_robocasa.yaml \
    use_sim_time:=true \
    use_rviz:=true \
    teleop_type:=none

# Terminal 3: Configure and stow
ros2 service call /stow_the_robot std_srvs/srv/Trigger
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.20
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.20
```

### Enable Cameras and PointClouds

```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
    use_mujoco_viewer:=true \
    mode:=navigation \
    use_cameras:=true
```

## 7. Troubleshooting

### GPU Not Detected

If the container can't access the GPU:

```bash
# Check Nvidia driver installation
nvidia-smi

# Verify Docker can see the GPU
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# Check Nvidia Container Toolkit configuration
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### X11 Display Issues

If GUI applications don't appear:

```bash
# Allow X11 connections
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Try running with --net=host
docker run -it --rm --gpus all --net=host -e DISPLAY=$DISPLAY ...
```

### OpenGL/EGL Issues

If you encounter OpenGL errors:

```bash
# Make sure MUJOCO_GL is set to egl
docker run -it --rm --gpus all -e MUJOCO_GL=egl ...
```

## 8. Performance Tips

- Use `--shm-size=2g` or higher if you encounter shared memory issues
- For better performance, use `--ipc=host`
- Consider using `--cpuset-cpus` to dedicate specific CPU cores
- Monitor GPU usage with `nvidia-smi` while running simulations

## 9. Cleaning Up

Remove containers and images:

```bash
# Remove stopped containers
docker container prune

# Remove unused images
docker image prune

# Remove the stretch-simulation image
docker rmi stretch-simulation:latest
```

## Additional Resources

- [Stretch ROS2 Documentation](https://docs.hello-robot.com/0.3/ros2/)
- [Nvidia Container Toolkit Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [Docker Documentation](https://docs.docker.com/)

