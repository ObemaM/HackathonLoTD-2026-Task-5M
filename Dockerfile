FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libpcl-dev \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/metro_detector
COPY ros2_ws/src ./ros2_ws/src

RUN . /opt/ros/humble/setup.sh && \
    cd ros2_ws && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "metro_obstacle_detector", "detector.launch.py"]

