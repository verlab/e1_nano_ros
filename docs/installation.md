# Installation

There are three options, from the simplest to the most complete.

=== "Docker (recommended)"

    Requires Docker and Docker Compose. Tested on Ubuntu 24.04. On macOS/Windows the simulation runs, but
    `network_mode: host` only reaches real drones on Linux.

    ```bash
    git clone --recursive https://github.com/verlab/e1_nano_ros.git
    cd e1_nano_ros
    docker compose build                       # image e1_nano_ros:jazzy (ROS 2 + rmw_zenoh + MuJoCo)
    E1_N=4 docker compose up router sim bridge controller
    ```

    In another terminal:

    ```bash
    docker compose run --rm tools ros2 topic list            # /e1_1/state, /e1_1/pose, ...
    docker compose run --rm tools ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.5, duration: 3.0}"
    ```

    Other distributions: `ROS_DISTRO=kilted docker compose build` (or `lyrical`).

=== "Native (ROS 2 Jazzy/Kilted)"

    ```bash
    sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp ros-$ROS_DISTRO-robot-state-publisher \
                     ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-joint-state-publisher-gui
    mkdir -p ~/e1_ws/src && cd ~/e1_ws/src
    git clone --recursive https://github.com/verlab/e1_nano_ros.git
    pip install -r e1_nano_ros/e1_core/requirements.txt     # eclipse-zenoh==1.10.1, mujoco, imageio
    cd ~/e1_ws && rosdep install --from-paths src --ignore-src -y --skip-keys e1_core
    colcon build --symlink-install
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp                 # add to ~/.bashrc
    ```

    !!! note "Zenoh versions must match"
        The Jazzy/Kilted `rmw_zenoh` embeds zenoh-c **1.10.x**. For this reason the Python package is pinned to
        `eclipse-zenoh==1.10.1` and the firmware to `zenoh-pico 1.10.1`. If you upgrade one, upgrade all three.

=== "Python only (without ROS)"

    Suitable for experimenting with swarm algorithms or running in notebooks:

    ```bash
    git clone --recursive https://github.com/verlab/e1_nano_ros.git && cd e1_nano_ros
    python3 -m venv .venv && source .venv/bin/activate
    pip install -e "e1_core[sim,zenoh]"
    export E1_DESCRIPTION_DIR=$PWD/e1_description
    python3 -m e1_core.sim.cli -n 4 --mode peer --listen tcp/127.0.0.1:7447 --connect ""   # simulator
    python3 examples/swarm_formation.py --n 4                                               # another terminal
    ```

## Verifying

```bash
cd e1_core && python3 -m pytest -q      # 9 tests: protocol, closed-loop firmware, failsafe, end-to-end Zenoh
```

## Firmware

The firmware does not require ROS. See the [firmware tutorial](tutorial/03-firmware.md):
`pio run -e e1_nano -t upload` or `docker compose run --rm firmware pio run`.
