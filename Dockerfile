FROM ros:foxy

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip git libboost-all-dev liboctomap-dev

# Install libccd
RUN cd /tmp && git clone https://github.com/danfis/libccd.git && cd libccd \
    && mkdir -p build && cd build \
    && cmake -G 'Unix Makefiles' -DENABLE_DOUBLE_PRECISION=ON .. \
    && make -j$(nproc) && sudo make install

# Install FCL
RUN cd /tmp && git clone https://github.com/flexible-collision-library/fcl.git \
    && cd fcl && mkdir -p build && cd build \
    && cmake .. && make -j$(nproc) && sudo make install

# Create ROS workspace
WORKDIR /ros2_ws/src

# Clone robot_collision_checking code
RUN git clone https://github.com/philip-long/robot_collision_checking.git

# Build packages with Colcon
WORKDIR /ros2_ws/
RUN sudo apt-get update && rosdep update --include-eol-distros && rosdep install \
    --rosdistro foxy --from-paths src --ignore-src -y
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install