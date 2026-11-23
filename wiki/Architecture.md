# Architecture and middleware

* **Drone:** CF-Drone (≈1 kHz, core 1) + Zenoh link (core 0) → `e1/<id>/**` over TCP to the router.
* **Ground:** `rmw_zenohd` (ROS 2 router) ← `e1_bridge` (ROS ↔ Zenoh), `e1_swarm_controller` (position), RViz, scripts.
* **Simulation:** `e1-sim` replaces the drones using the same protocol (CF-Drone port + MuJoCo/Genesis).

**Why Zenoh and not micro-ROS?** Router-based discovery instead of O(N²) DDS discovery over WiFi, no per-robot
agent, `e1/all/cmd/*` broadcast, access outside ROS, and `rmw_zenoh` has been Tier-1 since Kilted (ROS
shares the same network). Full analysis: https://verlab.github.io/e1_nano_ros/middleware/
