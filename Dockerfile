# create ROS humble image
FROM osrf/ros:humble-desktop-full

# label with source repo
LABEL org.opencontainers.image.source \
  https://github.com/freshrobotics/swerve-sim-container

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"
ARG CONFIG_DIR="${WORKSPACE}/config"
ARG CYCLONEDDS_URI="${CONFIG_DIR}/cyclonedds.xml"
ARG DEBIAN_FRONTEND="noninteractive"

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# setup 
# * rosdep (to install runtime dependencies in deploy container)
# * cyclone dds
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python-is-python3 \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && pip install transforms3d \
    && rm -rf /var/lib/apt/lists/*

# setup default users .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    >> ${HOME_DIR}/.bashrc \
  && echo "export CYCLONEDDS_URI=${CYCLONEDDS_URI}" \
    >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc

# create workspace and source dir
RUN mkdir -p ${WORKSPACE}
WORKDIR ${WORKSPACE}

# copy code into workspace
COPY ./src ${WORKSPACE}/src
COPY ./config ${CONFIG_DIR}

# install deps and build as non-root user
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && sudo apt-get update \
  && rosdep install -y -r -i --from-paths ${WORKSPACE}/src \
  && colcon build"

RUN echo "source install/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /usr/share/gazebo/setup.sh" >> ${HOME_DIR}/.bashrc

# by default hold container open in background
CMD ["tail", "-f", "/dev/null"]
