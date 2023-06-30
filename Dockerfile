FROM ros:humble-ros-base
RUN mkdir -p vision_opencv_ws/src
WORKDIR vision_opencv_ws/src
COPY . .
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    && \
    rosdep install -iry --from-paths . --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*
WORKDIR ../
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"]