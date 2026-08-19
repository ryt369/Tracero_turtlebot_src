FROM osrf/ros:humble-desktop-full

ARG NAV2_COMMIT=3c3db59d6969d8ecee8e68468693d006397f4a0c
ARG TURTLEBOT3_COMMIT=ed3521af40f2f73711aace307853bbf125583418
ARG TURTLEBOT3_MSGS_COMMIT=bf28eaed979bec8c79ed19a8aa0e26e96a6529d1
ARG TURTLEBOT3_SIMULATIONS_COMMIT=a35a56c8b04877dc89772b598084d8ce648a9023
ARG UBUNTU_MIRROR=https://mirrors.ustc.edu.cn/ubuntu
ARG ROS_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=30 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    TURTLEBOT3_MODEL=burger

SHELL ["/bin/bash", "-c"]

RUN sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
        /etc/apt/sources.list \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_MIRROR}|g" \
        /etc/apt/sources.list.d/ros2.sources \
    && sed -i 's/^Types: deb deb-src/Types: deb/' \
        /etc/apt/sources.list.d/ros2.sources \
    && apt-get -o Acquire::Retries=5 update \
    && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-navigation2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

COPY src/tracero_agent/requirements-static-index.txt /tmp/requirements-static-index.txt
RUN python3 -m pip install --no-cache-dir \
    -r /tmp/requirements-static-index.txt \
    && rm /tmp/requirements-static-index.txt

RUN mkdir -p /root/nav2_ws/src /root/turtlebot3_ws/src /root/turtlebot3_ws/events \
    && git init /root/nav2_ws/src/navigation2 \
    && git -C /root/nav2_ws/src/navigation2 remote add origin \
        https://github.com/ros-navigation/navigation2.git \
    && git -C /root/nav2_ws/src/navigation2 fetch --depth 1 origin ${NAV2_COMMIT} \
    && git -C /root/nav2_ws/src/navigation2 checkout --detach FETCH_HEAD \
    && git init /root/turtlebot3_ws/src/turtlebot3 \
    && git -C /root/turtlebot3_ws/src/turtlebot3 remote add origin \
        https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git -C /root/turtlebot3_ws/src/turtlebot3 fetch --depth 1 origin ${TURTLEBOT3_COMMIT} \
    && git -C /root/turtlebot3_ws/src/turtlebot3 checkout --detach FETCH_HEAD \
    && git init /root/turtlebot3_ws/src/turtlebot3_msgs \
    && git -C /root/turtlebot3_ws/src/turtlebot3_msgs remote add origin \
        https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git -C /root/turtlebot3_ws/src/turtlebot3_msgs fetch --depth 1 origin \
        ${TURTLEBOT3_MSGS_COMMIT} \
    && git -C /root/turtlebot3_ws/src/turtlebot3_msgs checkout --detach FETCH_HEAD \
    && git init /root/turtlebot3_ws/src/turtlebot3_simulations \
    && git -C /root/turtlebot3_ws/src/turtlebot3_simulations remote add origin \
        https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && git -C /root/turtlebot3_ws/src/turtlebot3_simulations fetch --depth 1 origin \
        ${TURTLEBOT3_SIMULATIONS_COMMIT} \
    && git -C /root/turtlebot3_ws/src/turtlebot3_simulations checkout --detach FETCH_HEAD \
    && find /root/nav2_ws/src /root/turtlebot3_ws/src \
        -type d -name .git -prune -exec rm -rf {} +

COPY src/tracero_agent /root/turtlebot3_ws/src/tracero_agent

RUN source /opt/ros/humble/setup.bash \
    && cd /root/turtlebot3_ws \
    && colcon build --symlink-install --packages-select tracero_agent

COPY docker/entrypoint.sh /tracero_entrypoint.sh
RUN chmod 755 /tracero_entrypoint.sh \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /root/turtlebot3_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[[ -f /usr/share/gazebo/setup.sh ]] && source /usr/share/gazebo/setup.sh' \
        >> /root/.bashrc

WORKDIR /root/turtlebot3_ws
ENTRYPOINT ["/tracero_entrypoint.sh"]
CMD ["bash"]
