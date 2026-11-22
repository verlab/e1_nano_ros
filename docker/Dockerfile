# E1 Nano ROS 2 workspace: ROS 2 + rmw_zenoh + MuJoCo SITL + e1_* packages
#   docker build -f docker/Dockerfile -t e1_nano_ros:jazzy .
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=kilted -t e1_nano_ros:kilted .
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    MUJOCO_GL=egl \
    PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-xacro \
      python3-pip python3-numpy python3-pytest \
      libegl1 libgl1 libosmesa6 ffmpeg git \
    && rm -rf /var/lib/apt/lists/*

COPY e1_core/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /ws
COPY e1_core       src/e1_nano_ros/e1_core
COPY e1_msgs       src/e1_nano_ros/e1_msgs
COPY e1_description src/e1_nano_ros/e1_description
COPY e1_bridge     src/e1_nano_ros/e1_bridge
COPY e1_control    src/e1_nano_ros/e1_control
COPY e1_bringup    src/e1_nano_ros/e1_bringup
COPY examples      src/e1_nano_ros/examples

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update --rosdistro ${ROS_DISTRO} >/dev/null 2>&1 || true && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "e1_core" >/dev/null 2>&1 || true && \
    colcon build --symlink-install --event-handlers console_direct- && \
    rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && echo "source /ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
