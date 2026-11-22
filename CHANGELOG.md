# Changelog

## 0.1.0 — 2026-11-21
- Modelo 3D do E1 Nano v1.5Y a partir do PCB real (URDF, MJCF, Genesis, malhas)
- Protocolo E1 v1 sobre Zenoh, com espelho em C para o firmware
- `e1_core`: porte do controlador do CF-Drone, controle de posição, cliente/enxame, simuladores MuJoCo e Genesis, `e1-sim`, `e1-cli`
- `e1_bridge`, `e1_control`, `e1_bringup`, `e1_msgs`, `e1_description`
- Firmware: E1 Zenoh link (zenoh-pico 1.10.1) sobre CF-Drone `83ebe61`
- Docker/Compose, CI, MkDocs (GitHub Pages), wiki, tutorial
