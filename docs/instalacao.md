# Instalação

Há três formas, da mais simples para a mais completa.

=== "Docker (recomendado)"

    Requer Docker e Docker Compose. Testado em Ubuntu 24.04. No macOS/Windows a simulação roda, mas o
    `network_mode: host` só alcança drones reais em Linux.

    ```bash
    git clone --recursive https://github.com/verlab/e1_nano_ros.git
    cd e1_nano_ros
    docker compose build                       # imagem e1_nano_ros:jazzy (ROS 2 + rmw_zenoh + MuJoCo)
    E1_N=4 docker compose up router sim bridge controller
    ```

    Em outro terminal:

    ```bash
    docker compose run --rm tools ros2 topic list            # /e1_1/state, /e1_1/pose, ...
    docker compose run --rm tools ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.5, duration: 3.0}"
    ```

    Outras distribuições: `ROS_DISTRO=kilted docker compose build` (ou `lyrical`).

=== "Nativo (ROS 2 Jazzy/Kilted)"

    ```bash
    sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp ros-$ROS_DISTRO-robot-state-publisher \
                     ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-joint-state-publisher-gui
    mkdir -p ~/e1_ws/src && cd ~/e1_ws/src
    git clone --recursive https://github.com/verlab/e1_nano_ros.git
    pip install -r e1_nano_ros/e1_core/requirements.txt     # eclipse-zenoh==1.10.1, mujoco, imageio
    cd ~/e1_ws && rosdep install --from-paths src --ignore-src -y --skip-keys e1_core
    colcon build --symlink-install
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp                 # coloque no ~/.bashrc
    ```

    !!! note "Versões do Zenoh precisam bater"
        O `rmw_zenoh` do Jazzy/Kilted embute o zenoh-c **1.10.x**. Por isso o pacote Python fica fixado em
        `eclipse-zenoh==1.10.1` e o firmware em `zenoh-pico 1.10.1`. Se atualizar um, atualize os três.

=== "Só Python (sem ROS)"

    Bom para experimentar algoritmos de enxame ou rodar em notebooks:

    ```bash
    git clone --recursive https://github.com/verlab/e1_nano_ros.git && cd e1_nano_ros
    python3 -m venv .venv && source .venv/bin/activate
    pip install -e "e1_core[sim,zenoh]"
    export E1_DESCRIPTION_DIR=$PWD/e1_description
    python3 -m e1_core.sim.cli -n 4 --mode peer --listen tcp/127.0.0.1:7447 --connect ""   # simulador
    python3 examples/swarm_formation.py --n 4                                               # outro terminal
    ```

## Verificando

```bash
cd e1_core && python3 -m pytest -q      # 9 testes: protocolo, firmware em malha fechada, failsafe, Zenoh ponta a ponta
```

## Firmware

O firmware não precisa de ROS. Veja o [tutorial de firmware](tutorial/03-firmware.md):
`pio run -e e1_nano -t upload` ou `docker compose run --rm firmware pio run`.
