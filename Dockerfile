FROM ghcr.io/ros-planning/moveit2:galactic-source

MAINTAINER Vatan Aksoy Tezer vatan@picknik.ai

# Update and install some common depenccies
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && apt-get install -y wget python3-pip lsb-release gnupg curl python3-vcstool python3-colcon-common-extensions git

# Install Ignition Gazebo Fortress binaries
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'  && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -  && \
    apt-get -qq update  && \
    apt-get install -y libignition-gazebo6-dev

WORKDIR /root/ws_stretch/src

# Download stretch source so that we can get necessary dependencies
RUN git clone https://github.com/vatanaksoytezer/stretch_ros.git -b pr-docker && \
    vcs import < stretch_ros/stretch_ros.repos

# Update and install dependencies
RUN . /opt/ros/galactic/setup.sh && . /root/ws_moveit/install/setup.sh \
    export IGNITION_VERSION=fortress && \
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro galactic --as-root=apt:false --skip-keys="ignition-transport11 ignition-gazebo6 ignition-msgs8" 

# Build the workspace
RUN cd /root/ws_stretch/ && . /opt/ros/galactic/setup.sh && . /root/ws_moveit/install/setup.sh \
    export IGNITION_VERSION=fortress && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status-

# GUI Config
RUN mkdir -p /root/.ignition/gazebo/6 && \
    cp /usr/share/ignition/ignition-gazebo6/gui/gui.config /root/.ignition/gazebo/6

# Remove docker clean and apt update

# Add some bashrc shortcuts
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws_moveit/install/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws_stretch/install/setup.bash" >> ~/.bashrc && \
    echo "export IGNITION_VERSION=fortress" >> ~/.bashrc && \
    echo "export IGN_GAZEBO_RESOURCE_PATH=/root/ws_stretch/src/stretch_ros:/root/ws_stretch/src/realsense-ros:/root/ws_stretch/src/aws-robomaker-small-house-world/models" >> ~/.bashrc


# Install some choice of editors (nano, vim, emacs)
RUN apt-get -qq update && \
    apt-get install apt-transport-https nano vim emacs -y && \
    rm -rf /var/lib/apt/lists/*
