FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

# 기본 도구
RUN apt update && apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    gazebo \
    x11-apps \
    git \
    && rm -rf /var/lib/apt/lists/*

# bash 자동 source
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

WORKDIR /root
CMD ["bash"]

