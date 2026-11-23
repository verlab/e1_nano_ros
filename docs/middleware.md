# Middleware choice: Zenoh, micro-ROS or something else?

**Decision: Zenoh end to end.** The drone runs zenoh-pico (client) and ROS 2 runs `rmw_zenoh_cpp`.
Both share the same router. The drone uses a compact, versioned binary protocol
(`e1/<id>/**`), and `e1_bridge` exposes everything as idiomatic ROS 2.

## Requirements

1. Dozens of drones on the same 2.4 GHz WiFi network (ESP32-S3, 512 kB of SRAM, 2 cores).
2. Commands at 50 Hz per drone with low latency. The control loop (≈1 kHz) must never block on the network.
3. Native integration with ROS 2 (RViz, rosbag, Nav2/planners) **and** access without ROS (scripts, RL, notebooks).
4. The same interface for simulation and hardware.
5. Minimal per-drone configuration (large swarms).

## Comparison

| Criterion | **Zenoh (zenoh-pico + rmw_zenoh)** | micro-ROS (DDS-XRCE) | DDS directly on the ESP32 | MAVLink (already in CF-Drone) |
|---|---|---|---|---|
| Discovery with N robots | via the router: each drone declares only its own keys | the agent bridges to DDS: DDS discovery still happens there | O(N²), multicast, impractical over WiFi | none: fixed IDs |
| Per-drone infrastructure | none (1 router for all) | 1 session on the agent; one agent per link | — | one MAVROS bridge per drone |
| ROS 2 integration | `rmw_zenoh` is **Tier-1 since Kilted** (same network) | native (ROS types) | native | via MAVROS (heavy for N drones) |
| Access without ROS | Python/C/Rust/JS APIs | requires ROS | requires DDS | pymavlink |
| Swarm broadcast | `e1/all/cmd/*` (1 msg) | 1 msg per drone | 1 msg per drone | per sysid |
| MCU footprint | small (zenoh-pico) | medium (rclc + XRCE) | large | small |
| Topologies | client, peer, meshed routers, multi-room | star around the agent | DDS mesh | point to point |
| ESP32 maturity | official Arduino/ESP-IDF support | official Arduino/ESP-IDF support | experimental | mature |

## Why not micro-ROS?

micro-ROS works well for one or a few robots. In a swarm, however, all traffic goes through the XRCE
agent, and the agent republishes it on DDS, so the DDS discovery explosion reappears on the PC side.
Even in typical swarm scenarios, micro-ROS communication is always mediated by a broker, with the
complexity of deploying and configuring that broker. There is also an `rmw_zenoh_pico` for micro-ROS
(esol-community), which is promising but lies outside official micro-ROS and is experimental.

## Why not speak `rmw_zenoh` byte for byte on the drone?

It would be possible to publish CDR on the internal `rmw_zenoh` keys (`<domain>/<topic>/<type>/<hash>`) and
declare the ROS graph liveliness tokens from the ESP32. This, however, couples the firmware to
RMW internals (there was a wire-compatibility break in Jazzy, for example) and uses more
bandwidth per message. We chose:

* **firmware → stable, versioned E1 protocol** (structs of 9 to 76 bytes, `VERSION=1`);
* **`e1_bridge` → idiomatic ROS 2** (standard types + `e1_msgs`). It is a single process for the entire swarm,
  on the same Zenoh network, so there is no extra network hop.

Should the drone need to become a "real ROS node" in the future, only the `zenoh_link.hpp` module changes.

## Versions

| Component | Version |
|---|---|
| ROS 2 | Jazzy (LTS) or Kilted/Lyrical |
| rmw_zenoh_cpp | distribution binary (embedded zenoh-c 1.10.x) |
| zenoh-pico (firmware) | 1.10.1 |
| eclipse-zenoh (Python) | 1.10.1 |

## References

* ROS 2 Kilted Kaiju: first distribution with Zenoh as a Tier-1 middleware — discourse.ros.org
* rmw_zenoh design — github.com/ros2/rmw_zenoh
* zenoh-pico: Arduino/ESP-IDF support, UDP/TCP/serial — github.com/eclipse-zenoh/zenoh-pico
* *Performance Comparison of ROS2 Middlewares for Multi-robot Mesh Networks* (J. Intell. Robot. Syst., 2024): lower delay and lower CPU usage with Zenoh in a dynamic mesh
* *Discovery Storm: Scalability Analysis of DDS and Zenoh in Large-Scale Wireless Robotic Networks* (DGIST, 2026)
* *Edge Robotics with ROS2 and Eclipse Zenoh* (EclipseCon): limitations of broker-mediated micro-ROS in swarms
* rmw_zenoh_pico for micro-ROS (esol-community) — discourse.openrobotics.org
