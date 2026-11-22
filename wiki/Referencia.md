# Referência rápida

| ROS 2 | Tipo |
|---|---|
| `/e1_<id>/state` | `e1_msgs/FlightState` |
| `/e1_<id>/pose`, `/odom`, `/imu`, `/battery` | padrão |
| `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` |
| `/e1_<id>/setpoint` | `e1_msgs/PositionTarget` |
| `/e1_<id>/takeoff`, `/land`, `/goto` | serviços `e1_msgs` |
| `/e1_<id>/arm`, `/set_mode`, `/set_param`, `/console` | serviços |
| `/e1/takeoff_all`, `/e1/land_all`, `/e1/emergency_stop`, `/e1/swarm_status` | enxame |

| Zenoh | |
|---|---|
| `e1/<id>/state` · `pose` · `log` · `alive` | telemetria / descoberta |
| `e1/<id>/cmd/attitude` · `manual` · `motors` · `arm` · `mode` · `param` · `console` | comandos |
| `e1/all/cmd/*` | broadcast |

`e1-cli list | status | arm <id> | disarm <id|all> | param <id> NOME VALOR | console <id> "cmd" | estop`
