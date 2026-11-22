# 4. Voo real com captura de movimento

O E1 Nano estima só a atitude (IMU MPU-6500). **Para voar por posição é preciso um sistema externo**:
OptiTrack/Vicon (via `vrpn_mocap` ou `mocap4r2`), câmera com marcadores ArUco, UWB etc. O `e1_bridge`
repassa as poses do ROS para a chave `e1/<id>/pose`, e o controlador de posição fecha a malha no PC.

## 4.1 Marcadores e rigid bodies

* 3 a 4 marcadores de 6 a 8 mm, colados assimetricamente na placa (cantos dos braços e bateria).
* Crie no software do mocap um rigid body por drone, com nome `e1_<id>` e **frame FLU**: x para a frente
  (seta na serigrafia), z para cima, origem no centro da placa.

## 4.2 Subir tudo

```bash
# mocap -> ROS (exemplo com vrpn_mocap)
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.10 port:=3883
# roteador + driver + controle + RViz
ros2 launch e1_bringup real.launch.py ids:="[1,2,3]" mocap_topic:="/vrpn_mocap/e1_{id}/pose"
```

Confira no RViz que cada drone aparece no lugar certo e com o yaw certo **antes** de decolar.

## 4.3 Voar

```bash
ros2 service call /e1_1/takeoff e1_msgs/srv/Takeoff "{height: 0.4, duration: 3.0}"
ros2 run e1_control square_mission --ros-args -p id:=1
ros2 service call /e1/land_all e1_msgs/srv/Land "{duration: 3.0}"
```

Emergência: `ros2 service call /e1/emergency_stop std_srvs/srv/Trigger` (ou `e1-cli estop`).
Tenha também um `e1-cli estop` pronto num terminal separado, que não depende do ROS.

## 4.4 Ajustes finos

| Sintoma | Ajuste |
|---|---|
| Deriva em hover | `e1-cli param 1 CTL_TRIM_ROLL 0.01` / `CTL_TRIM_PITCH` (ver comentários do CF-Drone) |
| Sobe/desce devagar | `max_thrust` em `e1_control/config/controller.yaml` (empuxo real dos seus motores) |
| Oscila em posição | reduza `kp`/`kd` do controlador; latência do mocap alta aumenta a oscilação |
| `cmd_timeout` em voo | WiFi saturado: veja [Segurança](../seguranca.md) e a configuração do roteador |
