FROM osrf/ros:humble-desktop-full

ARG NAV2_COMMIT=3c3db59d6969d8ecee8e68468693d006397f4a0c
ARG TURTLEBOT3_COMMIT=ed3521af40f2f73711aace307853bbf125583418
ARG TURTLEBOT3_MSGS_COMMIT=bf28eaed979bec8c79ed19a8aa0e26e96a6529d1
ARG TURTLEBOT3_SIMULATIONS_COMMIT=a35a56c8b04877dc89772b598084d8ce648a9023
ARG TRACERO_AGENT_COMMIT=working-tree
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
    && sed -i "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
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
    && git config --global http.version HTTP/1.1 \
    && fetch_commit() { \
        directory="$1"; repository_url="$2"; commit="$3"; \
        git init "${directory}"; \
        git -C "${directory}" remote add origin "${repository_url}"; \
        fetched=false; \
        for attempt in 1 2 3 4 5; do \
            if git -C "${directory}" fetch --depth 1 origin "${commit}"; then \
                fetched=true; \
                break; \
            fi; \
            echo "Fetch attempt ${attempt} failed for ${repository_url}"; \
            sleep $((attempt * 2)); \
        done; \
        "${fetched}"; \
        git -C "${directory}" checkout --detach FETCH_HEAD; \
    }; \
    fetch_commit /root/nav2_ws/src/navigation2 \
        https://github.com/ros-navigation/navigation2.git "${NAV2_COMMIT}" \
    && fetch_commit /root/turtlebot3_ws/src/turtlebot3 \
        https://github.com/ROBOTIS-GIT/turtlebot3.git "${TURTLEBOT3_COMMIT}" \
    && fetch_commit /root/turtlebot3_ws/src/turtlebot3_msgs \
        https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
        "${TURTLEBOT3_MSGS_COMMIT}" \
    && fetch_commit /root/turtlebot3_ws/src/turtlebot3_simulations \
        https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
        "${TURTLEBOT3_SIMULATIONS_COMMIT}" \
    && printf '{\n  "repositories": {\n    "navigation2": {"commit": "%s", "source_root": "nav2_ws/src/navigation2"},\n    "turtlebot3": {"commit": "%s", "source_root": "turtlebot3_ws/src/turtlebot3"},\n    "turtlebot3_msgs": {"commit": "%s", "source_root": "turtlebot3_ws/src/turtlebot3_msgs"},\n    "turtlebot3_simulations": {"commit": "%s", "source_root": "turtlebot3_ws/src/turtlebot3_simulations"},\n    "tracero_agent": {"commit": "%s", "source_root": "turtlebot3_ws/src/tracero_agent"}\n  }\n}\n' \
        "${NAV2_COMMIT}" "${TURTLEBOT3_COMMIT}" \
        "${TURTLEBOT3_MSGS_COMMIT}" "${TURTLEBOT3_SIMULATIONS_COMMIT}" \
        "${TRACERO_AGENT_COMMIT}" \
        > /root/source-versions.json \
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
