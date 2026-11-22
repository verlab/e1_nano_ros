# Arquitetura e middleware

* **Drone:** CF-Drone (≈1 kHz, core 1) + link Zenoh (core 0) → `e1/<id>/**` por TCP até o roteador.
* **Solo:** `rmw_zenohd` (roteador do ROS 2) ← `e1_bridge` (ROS ↔ Zenoh), `e1_swarm_controller` (posição), RViz, scripts.
* **Simulação:** `e1-sim` substitui os drones com o mesmo protocolo (porte do CF-Drone + MuJoCo/Genesis).

**Por que Zenoh e não micro-ROS?** Descoberta via roteador em vez de DDS O(N²) no WiFi, nenhum agente
por robô, broadcast `e1/all/cmd/*`, acesso fora do ROS, e `rmw_zenoh` Tier-1 desde o Kilted (o ROS
compartilha a mesma malha). Análise completa: https://verlab.github.io/e1_nano_ros/middleware/
