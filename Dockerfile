FROM ros:foxy-ros-base-focal

# Use bash.
SHELL ["/bin/bash", "-c"]

# Remove debconf errors (must run before any apt-get calls).
RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# Install needed linux packages.
# apt-get   https://wiki.ubuntuusers.de/apt/apt-get/
# -y        yes to all displayed questions     
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y apt-utils software-properties-common git vim nano nodejs \
    && apt-get install -y python3-pip \
    && pip install pillow pyserial

# Set up iotbot workspace.
RUN mkdir -p /home/iotbot_ws/src/iotbot
WORKDIR /home/iotbot_ws
RUN cd src/ && git clone https://github.com/gramss/openmv-ros2

# Look for ROS2 package dependencies.
# rodsep            https://docs.ros.org/en/independent/api/rosdep/html/commands.html
# -v, --verbose     verbose display
# -y, --default-yes tell the package manager to default to y or fail when installing
# -r                continue installing despite errors
RUN source /opt/ros/foxy/setup.bash \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=foxy

# build iotbot_interface first
#RUN source /opt/ros/foxy/setup.bash \
#    && colcon build --packages-select iotbot_interface
    # && colcon test

# build other nodes after interface to find interface messages in those nodes
RUN source /opt/ros/foxy/setup.bash \
    && MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential \
    && source /home/iotbot_ws/install/setup.bash \
    # && colcon test

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
    && echo "source /home/iotbot_ws/install/setup.bash" >> ~/.bashrc

# Run the Node as soon a the container is started
CMD source /opt/ros/foxy/setup.bash && \
    source /home/iotbot_ws/install/setup.bash && \
    ros2 run openmv_driver_action_server server

