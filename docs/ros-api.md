# API ROS 2

Namespace por drone: `/e1_<id>` (parâmetro `namespace_prefix`).

## e1_bridge (`ros2 run e1_bridge e1_bridge`)

| Tipo | Nome | Mensagem | Notas |
|---|---|---|---|
| pub | `/e1_<id>/state` | `e1_msgs/FlightState` | 50 Hz |
| pub | `/e1_<id>/imu` | `sensor_msgs/Imu` | best effort, frame `e1_<id>/base_link` |
| pub | `/e1_<id>/battery` | `sensor_msgs/BatteryState` | |
| pub | `/e1_<id>/pose` | `geometry_msgs/PoseStamped` | de `e1/<id>/pose` (sim ou mocap) |
| pub | `/e1_<id>/odom` | `nav_msgs/Odometry` | velocidade quando disponível |
| pub | `/e1_<id>/joint_states` | `sensor_msgs/JointState` | rotores (visual) |
| pub | TF | `world → e1_<id>/base_link` | |
| sub | `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` | entra em AUTO; ≥ 10 Hz |
| sub | `/e1_<id>/cmd/manual` | `e1_msgs/ManualControl` | sticks virtuais |
| sub | `/e1_<id>/cmd/motors` | `e1_msgs/MotorCommand` | bancada / identificação |
| srv | `/e1_<id>/arm` | `std_srvs/SetBool` | |
| srv | `/e1_<id>/set_mode` | `e1_msgs/SetMode` | |
| srv | `/e1_<id>/set_param` | `e1_msgs/SetParam` | parâmetros do CF-Drone |
| srv | `/e1_<id>/console` | `e1_msgs/Console` | retorna a saída do console |
| pub | `/e1/swarm_status` | `e1_msgs/SwarmStatus` | 1 Hz |
| srv | `/e1/emergency_stop` | `std_srvs/Trigger` | desarma todos |

Parâmetros: `zenoh_connect`, `ids` (`[0]` = descoberta automática), `namespace_prefix`, `world_frame`,
`publish_tf`, `publish_joint_states`, `mocap_topic` (ex.: `/vrpn_mocap/e1_{id}/pose`). Veja `e1_bridge/config/bridge.yaml`.

## e1_swarm_controller (`ros2 run e1_control swarm_controller`)

| Tipo | Nome | Mensagem |
|---|---|---|
| sub | `/e1_<id>/pose`, `/e1_<id>/odom`, `/e1_<id>/state` | entradas |
| sub | `/e1_<id>/setpoint` | `e1_msgs/PositionTarget` (streaming) |
| pub | `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` |
| srv | `/e1_<id>/takeoff` · `/e1_<id>/land` · `/e1_<id>/goto` | `Takeoff` · `Land` · `GoTo` |
| srv | `/e1/takeoff_all` · `/e1/land_all` | `Takeoff` · `Land` |
| pub | `/e1/setpoints` | `visualization_msgs/MarkerArray` |

Parâmetros: `ids`, `rate`, `mass`, `max_thrust`, `kp`, `kd`, `ki`, `tilt_max_deg`, `min_separation`
(`e1_control/config/controller.yaml`).

## Launch

| Arquivo | Argumentos |
|---|---|
| `e1_bringup sim.launch.py` | `num`, `layout`, `spacing`, `backend` (mujoco/genesis), `viewer`, `rviz`, `start_router`, `zenoh`, `mission` (none/square/formation), `record` |
| `e1_bringup real.launch.py` | `ids`, `mocap_topic`, `zenoh`, `start_router`, `router_config`, `rviz` |
| `e1_description display.launch.py` | um drone no RViz com sliders dos rotores |

## Mensagens

Veja `e1_msgs/msg/*.msg` e `e1_msgs/srv/*.srv`. Todas são comentadas.
