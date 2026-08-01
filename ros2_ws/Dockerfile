FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    g++ \
    libevdev-dev \
    python3-colcon-common-extensions \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "vida_gamepad", "gamepad.launch.py"]
