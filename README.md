# E1 Nano ROS — enxame de mini-drones ESP32 com ROS 2 + Zenoh

[![CI](https://github.com/verlab/e1_nano_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/verlab/e1_nano_ros/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-orange)](https://verlab.github.io/e1_nano_ros/)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Kilted%20%7C%20Lyrical-blue)
![RMW](https://img.shields.io/badge/RMW-rmw__zenoh__cpp-green)

Driver ROS 2, firmware, simulação e ferramentas de enxame para o **Cherglow E1 Nano**
([projeto aberto](https://oshwhub.com/songge8/project_fnvbukbz)): quadricóptero de 50 mm e 25 g com
ESP32-S3, que roda o firmware aberto [CF-Drone](https://github.com/songge8/CF-Drone).

https://github.com/verlab/e1_nano_ros/raw/main/docs/assets/swarm_demo.mp4

> Enxame de 6 drones no simulador MuJoCo: decolagem, grade em dois níveis, círculo girando e pouso.
> Cada drone roda o controlador do CF-Drone e é comandado pelo mesmo protocolo dos drones reais.

## Destaques

- **Zenoh de ponta a ponta.** O drone roda `zenoh-pico` e o ROS 2 usa `rmw_zenoh_cpp` (Tier-1 desde o
  Kilted). Drones, simulador, nós ROS e scripts Python compartilham **um roteador**, sem agente
  micro-ROS por robô e sem descoberta DDS O(N²) no WiFi. [Por que Zenoh?](docs/middleware.md)
- **Sim = real.** O simulador (`e1-sim`, MuJoCo ou Genesis) executa um porte do controlador do CF-Drone
  e fala o mesmo protocolo. Código testado na simulação voa no drone sem alterações.
- **Um driver para o enxame todo** (`e1_bridge`): descoberta automática, `/e1_<id>/...` por drone, TF,
  IMU, bateria, serviços de arm/modo/parâmetros/console e parada de emergência global.
- **Controle de posição para N drones** (`e1_control`): `takeoff`/`land`/`goto`, streaming de setpoints,
  separação mínima entre drones e missões de exemplo.
- **Modelo 3D gerado do PCB real** (URDF, MJCF e Genesis), com dimensões exatas: entre eixos de 50,0 mm e
  placa de 50,9 × 50,9 × 1,2 mm.
- **Docker**, CI, documentação no GitHub Pages, wiki e tutorial passo a passo.

## Estrutura

```
e1_nano_ros/
├── e1_core/          # núcleo Python (sem ROS): protocolo, firmware em Python, controle, cliente, simuladores
├── e1_msgs/          # mensagens e serviços ROS 2
├── e1_description/   # URDF / MJCF / Genesis URDF / malhas (modelo do PCB real)
├── e1_bridge/        # driver ROS 2 <-> Zenoh (um processo para o enxame)
├── e1_control/       # controle de posição do enxame + missões
├── e1_bringup/       # launch: simulação, voo real, roteador Zenoh, RViz
├── firmware/         # CF-Drone (submódulo) + E1 Zenoh link (zenoh-pico), PlatformIO
├── examples/         # scripts Python puros (quadrado, formação, vídeo offline)
├── docker/ compose.yaml
├── docs/ mkdocs.yml  # site (GitHub Pages)
└── wiki/             # páginas para a wiki do GitHub
```

## Início rápido

### Docker

```bash
git clone --recursive https://github.com/verlab/e1_nano_ros.git && cd e1_nano_ros
docker compose build
E1_N=4 docker compose --profile demo up router sim bridge controller mission
```

### ROS 2 nativo (Jazzy/Kilted)

```bash
sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-rviz2
cd ~/e1_ws/src && git clone --recursive https://github.com/verlab/e1_nano_ros.git
pip install -r e1_nano_ros/e1_core/requirements.txt
cd ~/e1_ws && colcon build --symlink-install && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch e1_bringup sim.launch.py num:=4 mission:=formation
```

### Só Python, sem ROS

```bash
pip install -e "e1_core[sim,zenoh]" && export E1_DESCRIPTION_DIR=$PWD/e1_description
python3 -m e1_core.sim.cli -n 6 --mode peer --listen tcp/127.0.0.1:7447 --connect "" &
python3 examples/swarm_formation.py --n 6
```

### Drone real

```bash
cd firmware && pio run -e e1_nano -t upload        # CF-Drone + E1 Zenoh link
# no console do drone:  sta <ssid> <senha> ; p WIFI_MODE 2 ; zn id 1 ; zn connect tcp/<ip-do-pc>:7447 ; reboot
ros2 launch e1_bringup real.launch.py ids:="[1]" mocap_topic:="/vrpn_mocap/e1_{id}/pose"
ros2 service call /e1_1/takeoff e1_msgs/srv/Takeoff "{height: 0.4, duration: 3.0}"
```

## Interface (resumo)

| ROS 2 (`/e1_<id>/...`) | Zenoh (`e1/<id>/...`) | |
|---|---|---|
| `state` `FlightState` · `imu` · `battery` · `pose` · `odom` · TF | `state` (50 Hz) · `pose` | telemetria |
| `cmd/attitude` `AttitudeTarget` | `cmd/attitude` | atitude + empuxo (AUTO) |
| `setpoint` `PositionTarget` · `takeoff` · `land` · `goto` | — (controle no solo) | posição |
| `arm` · `set_mode` · `set_param` · `console` | `cmd/arm` · `cmd/mode` · `cmd/param` · `cmd/console` | gestão |
| `/e1/emergency_stop` · `/e1/swarm_status` | `e1/all/cmd/arm` · `e1/*/alive` | enxame |

Referência completa: [API ROS 2](docs/ros-api.md) · [Protocolo Zenoh](docs/protocolo.md).

## Documentação

* Site: **https://verlab.github.io/e1_nano_ros/** (gerado a partir de `docs/`)
* Tutorial: [1 Simulação](docs/tutorial/01-simulacao.md) · [2 ROS 2](docs/tutorial/02-ros2.md) ·
  [3 Firmware](docs/tutorial/03-firmware.md) · [4 Voo real](docs/tutorial/04-voo-real.md) ·
  [5 Enxame](docs/tutorial/05-enxame.md) · [6 Genesis/RL](docs/tutorial/06-genesis.md) ·
  [7 Seu controlador](docs/tutorial/07-controlador.md)
* [Arquitetura](docs/arquitetura.md) · [Middleware](docs/middleware.md) · [Hardware](docs/hardware.md) ·
  [Simulação](docs/simulacao.md) · [Segurança](docs/seguranca.md) · [FAQ](docs/faq.md)

## Testes

```bash
cd e1_core && python3 -m pytest -q
```

São 9 testes: protocolo (tamanhos iguais ao header C), hover + quadrado com erro < 10 cm, 4 drones,
failsafe de perda de comando, parada de emergência por broadcast e decolagem/pouso através do Zenoh.

## Estado

| Parte | Estado |
|---|---|
| `e1_core` (protocolo, firmware em Python, controle, cliente, MuJoCo/Genesis) | testado (pytest, ponta a ponta via Zenoh) |
| Pacotes ROS 2 (`e1_bridge`, `e1_control`, `e1_bringup`) | sintaxe e lint verificados; build/execução validados pela CI em `ros:jazzy` |
| Firmware (E1 Zenoh link) | compila no host contra a API real do zenoh-pico 1.10.1 e os headers do CF-Drone `83ebe61` (`firmware/test/host_check/run.sh`); build ESP32 completo pela CI; **teste em hardware pendente** |
| Constantes de motor/hélice | estimadas: identifique as suas |

## Créditos e licenças

* Hardware E1 Nano e firmware CF-Drone: **琛光科技 / Cherglow (songge8)**. Hardware sob CC BY-NC-SA 4.0;
  CF-Drone sob GPL-3.0 com restrição comercial. O firmware combinado (`firmware/`) segue essas condições.
* Este repositório (pacotes ROS 2, `e1_core`, docs): **Apache-2.0** (`LICENSE`). O modelo 3D
  (`e1_description`) deriva do PCB aberto: **CC BY-NC-SA 4.0** (`e1_description/LICENSE.md`).
* Desenvolvido no **VeRLab — DCC/UFMG**.
