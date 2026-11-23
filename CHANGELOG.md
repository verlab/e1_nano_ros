# Changelog

## 0.2.0 — 2026-11-23
- `e1_mocap`: OptiTrack motion capture through a pure-Python NatNet client (NatNet 3.x/4.x, multicast or unicast,
  Y/Z-up, rigid body names `e1_<id>`), `/e1/mocap/status` diagnostics; `e1-mocap` relay for use without ROS
- `real.launch.py mocap:=natnet|topic|none`; `sim.launch.py mocap:=true` rehearses the real pipeline with a fake Motive
- Heading alignment (`YawAlign`): attitude commands are rotated into the onboard yaw frame (no magnetometer), on by
  default in `e1_swarm_controller` and the Python client
- Simulator: onboard heading offset and drift (`--yaw-offset`, `--yaw-drift`), fake NatNet server (`--natnet`)
- Firmware: stock CF-Drone variants (`e1_nano_stock`, `e1_mini_stock`); merged binaries per variant in CI and releases
- Documentation in English; interactive 3D viewer (MuJoCo and Genesis replays, exploded drone model) on GitHub Pages

## 0.1.0 — 2026-11-21
- 3D model of the E1 Nano v1.5Y built from the actual PCB (URDF, MJCF, Genesis, meshes)
- E1 protocol v1 over Zenoh, with a C mirror for the firmware
- `e1_core`: port of the CF-Drone controller, position control, client/swarm, MuJoCo and Genesis simulators, `e1-sim`, `e1-cli`
- `e1_bridge`, `e1_control`, `e1_bringup`, `e1_msgs`, `e1_description`
- Firmware: E1 Zenoh link (zenoh-pico 1.10.1) on top of CF-Drone `83ebe61`
- Docker/Compose, CI, MkDocs (GitHub Pages), wiki, tutorial
