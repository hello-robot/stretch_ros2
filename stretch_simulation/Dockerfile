# Stretch Simulation ROS2 Dockerfile with Nvidia GPU Support
# Based on Ubuntu 22.04 with ROS2 Humble

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set up locale
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    wget \
    git \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Add universe repository
RUN add-apt-repository universe

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    rviz \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install NodeJS 22.x and npm
RUN curl -sL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools needed for setup
RUN apt-get update && apt-get install -y \
    rpl \
    gnome-terminal \
    terminator \
    gedit \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip

# Remove distutils-installed packages to avoid conflicts
RUN rm -rf /usr/lib/python3/dist-packages/sympy* || true
RUN rm -rf /usr/lib/python3/dist-packages/blinker* || true
RUN rm -rf /usr/lib/python3/dist-packages/transforms3d* || true
RUN rm -rf /usr/local/lib/python3.10/dist-packages/transforms3d* || true

# Set up environment variables
ENV HELLO_FLEET_PATH=/root/stretch_user
ENV HELLO_FLEET_ID=stretch-se3-local
ENV MUJOCO_GL=egl

# Create necessary directories
RUN mkdir -p ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set up ament_ws workspace
WORKDIR /root

# Copy the ament workspace setup script
COPY stretch_create_ament_workspace.sh /tmp/stretch_create_ament_workspace.sh
RUN chmod +x /tmp/stretch_create_ament_workspace.sh

# Run the ament workspace setup script with DOCKER_BUILD environment variable
# This bypasses sudo checks and interactive prompts
RUN /bin/bash -c "export DOCKER_BUILD=1 && source /opt/ros/humble/setup.bash && /tmp/stretch_create_ament_workspace.sh"

# Source ament_ws setup in bashrc
RUN echo "source ~/ament_ws/install/setup.bash" >> /root/.bashrc

# Fix numpy include path issue if needed
RUN ln -sf /root/.local/lib/python3.10/site-packages/numpy/core /usr/include/numpy || true

# Remove any distutils-installed transforms3d before installing stretch-body
RUN rm -rf /usr/lib/python3/dist-packages/transforms3d* || true
RUN rm -rf /usr/local/lib/python3.10/dist-packages/transforms3d* || true
RUN pip3 uninstall -y transforms3d || true

# Set up URDF
RUN /bin/bash -c "source ~/ament_ws/install/setup.bash && \
    pip3 install -U hello-robot-stretch-urdf && \
    git clone https://github.com/hello-robot/stretch_urdf.git --depth 1 /tmp/stretch_urdf && \
    pip3 install hello-robot-stretch-body && \
    printf 'y\n' | python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py && \
    printf 'y\n' | python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py --ros2_rebuild"

# Fix PyOpenGL issue before setting up Mujoco
RUN pip3 install PyOpenGL==3.1.4

# Set up Mujoco
# setup.sh already installs dependencies and runs colcon build itself
WORKDIR /root/ament_ws
RUN rm -rf build install log && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    echo 'y' | bash ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/setup.sh"

# Set up Stretch Web Teleop URDF export
WORKDIR /root/ament_ws/src/stretch_ros2/stretch_description/urdf
RUN cp ./stretch_uncalibrated.urdf stretch.urdf && \
    ./export_urdf.sh || true && \
    mkdir -p ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}/exported_urdf && \
    cp -r ./exported_urdf/* ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}/exported_urdf || true

# Create maps directory
RUN mkdir -p ${HELLO_FLEET_PATH}/maps

# Set working directory back to ament_ws
WORKDIR /root/ament_ws

# Set up display for GUI applications
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Expose any necessary ports (for web teleop)
EXPOSE 8080 3000

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source ~/ament_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

