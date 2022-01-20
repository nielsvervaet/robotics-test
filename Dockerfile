FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

COPY catkin_ws /home/catkin_ws

RUN apt-get update \
    && cd /home/catkin_ws \
    && rosdep install --rosdistro=noetic --from-path . -y --ignore-src \
    && catkin build