# 5. Swarm

## Addressing

Each drone has an `id` (1 to 254) stored in the firmware (`zn id <n>`). The `id` defines the key `e1/<id>/**`, the
ROS namespace `/e1_<id>` and the frame `e1_<id>/base_link`. There is no central configuration: drones are
discovered through the liveliness tokens `e1/<id>/alive`.

## Simulated

```bash
ros2 launch e1_bringup sim.launch.py num:=8 layout:=grid mission:=formation
```

or Python only:

```bash
python3 -m e1_core.sim.cli -n 8 --mode peer --listen tcp/127.0.0.1:7447 --connect ""
python3 examples/swarm_formation.py --n 8
```

## Scaling (why Zenoh)

* **Inexpensive discovery.** In DDS, every participant announces itself to all others (O(N²) messages) over
  multicast, which typically saturates WiFi with a few dozen robots. In Zenoh, each drone opens a
  TCP session with the router and declares only its own keys.
* **No per-robot agent.** micro-ROS requires an XRCE agent and one session per microcontroller;
  zenoh-pico talks directly to the ROS 2 router.
* **Meshed routers.** With more than about 30 drones, or several rooms, use several routers
  (one per WiFi AP) connected to each other (`connect.endpoints` in `zenoh_router.json5`).
* **Broadcast.** `e1/all/cmd/*` reaches all drones with a single message (emergency stop, arm, mode).

## Swarm algorithms

* Centralized control: `e1_swarm_controller` (one node, N drones) or `e1_core.client.Swarm`.
* Distributed control: run one node per drone. Each node subscribes to its neighbors' `/e1_<j>/pose` and publishes its
  own `/e1_<i>/setpoint`. `separation_acc` in `e1_core.control` provides a simple inter-drone repulsion.
* For large N, publish setpoints directly over Zenoh (`e1_core.client`) and use ROS only for visualization and recording.

## Network best practices

* A dedicated AP for the swarm on a free 2.4 GHz channel, 20 MHz wide (the ESP32-S3 has no 5 GHz radio).
* Telemetry at 50 Hz per drone takes about 4 kB/s (76 B + headers). Commands at 50 Hz, about 2.5 kB/s.
  In practice, about 30 drones per AP with margin.
* Disable Zenoh multicast scouting (`scouting/multicast: false`, already set in `zenoh_router.json5`).
