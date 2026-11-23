# Quick reference

| ROS 2 | Type |
|---|---|
| `/e1_<id>/state` | `e1_msgs/FlightState` |
| `/e1_<id>/pose`, `/odom`, `/imu`, `/battery` | standard |
| `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` |
| `/e1_<id>/setpoint` | `e1_msgs/PositionTarget` |
| `/e1_<id>/takeoff`, `/land`, `/goto` | `e1_msgs` services |
| `/e1_<id>/arm`, `/set_mode`, `/set_param`, `/console` | services |
| `/e1/takeoff_all`, `/e1/land_all`, `/e1/emergency_stop`, `/e1/swarm_status` | swarm |

| Zenoh | |
|---|---|
| `e1/<id>/state` · `pose` · `log` · `alive` | telemetry / discovery |
| `e1/<id>/cmd/attitude` · `manual` · `motors` · `arm` · `mode` · `param` · `console` | commands |
| `e1/all/cmd/*` | broadcast |

`e1-cli list | status | arm <id> | disarm <id|all> | param <id> NAME VALUE | console <id> "cmd" | estop`
