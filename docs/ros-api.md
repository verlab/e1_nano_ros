# ROS 2 API

Per-drone namespace: `/e1_<id>` (parameter `namespace_prefix`).

## e1_bridge (`ros2 run e1_bridge e1_bridge`)

| Kind | Name | Message | Notes |
|---|---|---|---|
| pub | `/e1_<id>/state` | `e1_msgs/FlightState` | 50 Hz |
| pub | `/e1_<id>/imu` | `sensor_msgs/Imu` | best effort, frame `e1_<id>/base_link` |
| pub | `/e1_<id>/battery` | `sensor_msgs/BatteryState` | |
| pub | `/e1_<id>/pose` | `geometry_msgs/PoseStamped` | from `e1/<id>/pose` (sim or mocap) |
| pub | `/e1_<id>/odom` | `nav_msgs/Odometry` | velocity when available |
| pub | `/e1_<id>/joint_states` | `sensor_msgs/JointState` | rotors (visual) |
| pub | TF | `world → e1_<id>/base_link` | |
| sub | `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` | switches to AUTO; ≥ 10 Hz |
| sub | `/e1_<id>/cmd/manual` | `e1_msgs/ManualControl` | virtual sticks |
| sub | `/e1_<id>/cmd/motors` | `e1_msgs/MotorCommand` | bench tests / identification |
| srv | `/e1_<id>/arm` | `std_srvs/SetBool` | |
| srv | `/e1_<id>/set_mode` | `e1_msgs/SetMode` | |
| srv | `/e1_<id>/set_param` | `e1_msgs/SetParam` | CF-Drone parameters |
| srv | `/e1_<id>/console` | `e1_msgs/Console` | returns the console output |
| pub | `/e1/swarm_status` | `e1_msgs/SwarmStatus` | 1 Hz |
| srv | `/e1/emergency_stop` | `std_srvs/Trigger` | disarms all drones |

Parameters: `zenoh_connect`, `ids` (`[0]` = automatic discovery), `namespace_prefix`, `world_frame`,
`publish_tf`, `publish_joint_states`, `mocap_topic` (e.g. `/vrpn_mocap/e1_{id}/pose`). See `e1_bridge/config/bridge.yaml`.

## e1_swarm_controller (`ros2 run e1_control swarm_controller`)

| Kind | Name | Message |
|---|---|---|
| sub | `/e1_<id>/pose`, `/e1_<id>/odom`, `/e1_<id>/state` | inputs |
| sub | `/e1_<id>/setpoint` | `e1_msgs/PositionTarget` (streaming) |
| pub | `/e1_<id>/cmd/attitude` | `e1_msgs/AttitudeTarget` |
| srv | `/e1_<id>/takeoff` · `/e1_<id>/land` · `/e1_<id>/goto` | `Takeoff` · `Land` · `GoTo` |
| srv | `/e1/takeoff_all` · `/e1/land_all` | `Takeoff` · `Land` |
| pub | `/e1/setpoints` | `visualization_msgs/MarkerArray` |

Parameters: `ids`, `rate`, `mass`, `max_thrust`, `kp`, `kd`, `ki`, `tilt_max_deg`, `min_separation`, `align_yaw`, `pose_timeout`
(`e1_control/config/controller.yaml`).

## e1_mocap (OptiTrack NatNet)

| | |
|---|---|
| Publishes | `/e1_<id>/mocap/pose` (`geometry_msgs/PoseStamped`, Z-up world), `/e1/mocap/status` (`std_msgs/String`, JSON, 1 Hz) |
| Parameters | `server_ip`, `local_ip`, `multicast`, `multicast_group`, `command_port`, `data_port`, `natnet_version`, `up_axis` (z/y), `name_pattern`, `by_id`, `namespace_prefix`, `topic`, `world_frame`, `publish_tf`, `rate` (`e1_mocap/config/optitrack.yaml`) |

See [Motion capture](mocap.md).

## Launch

| File | Arguments |
|---|---|
| `e1_bringup sim.launch.py` | `num`, `layout`, `spacing`, `backend` (mujoco/genesis), `viewer`, `rviz`, `start_router`, `zenoh`, `mission` (none/square/formation), `record`, `mocap` (fake OptiTrack + `e1_mocap`), `yaw_offset`, `yaw_drift` |
| `e1_bringup real.launch.py` | `ids`, `mocap` (natnet/topic/none), `server_ip`, `local_ip`, `multicast`, `up_axis`, `mocap_topic`, `zenoh`, `start_router`, `router_config`, `rviz` |
| `e1_description display.launch.py` | a single drone in RViz with rotor sliders |

## Messages

See `e1_msgs/msg/*.msg` and `e1_msgs/srv/*.srv`. All of them are commented.
