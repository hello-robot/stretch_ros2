FROM librealsense/librealsense

FROM osrf/ros:humble-desktop-full

RUN apt update
RUN apt install -y python3-pip  \
    git \
    zip \
    rviz \ 
    libgstreamer1.0-0 gstreamer1.0-pulseaudio gstreamer1.0-libav gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-alsa \ 
    mesa-utils \
    libgl1-mesa-glx

RUN echo "ALL Done"

CMD [ "/bin/bash" ]