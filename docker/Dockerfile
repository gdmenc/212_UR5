FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    dbus-x11 \
    libgl1-mesa-dri \
    mesa-utils \
    ros-noetic-gazebo-msgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-tf \
    ros-noetic-ur-description \
    ros-noetic-ur-gazebo \
    ros-noetic-xacro \
    fluxbox \
    novnc \
    python3-rospkg \
    python3-yaml \
    websockify \
    x11-utils \
    x11vnc \
    xterm \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV MESA_GLSL_VERSION_OVERRIDE=330

CMD ["/bin/bash"]
