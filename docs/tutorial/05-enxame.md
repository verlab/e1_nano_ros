# 5. Enxame

## Endereçamento

Cada drone tem um `id` (1 a 254) gravado no firmware (`zn id <n>`). O `id` define a chave `e1/<id>/**`, o
namespace ROS `/e1_<id>` e o frame `e1_<id>/base_link`. Não há configuração central: os drones são
descobertos pelos tokens de liveliness `e1/<id>/alive`.

## Simulado

```bash
ros2 launch e1_bringup sim.launch.py num:=8 layout:=grid mission:=formation
```

ou só Python:

```bash
python3 -m e1_core.sim.cli -n 8 --mode peer --listen tcp/127.0.0.1:7447 --connect ""
python3 examples/swarm_formation.py --n 8
```

## Escalando (por que Zenoh)

* **Descoberta barata.** No DDS, cada participante anuncia a todos os outros (O(N²) mensagens) e por
  multicast, o que costuma travar o WiFi já com algumas dezenas de robôs. No Zenoh, cada drone abre uma
  sessão TCP com o roteador e declara só as suas chaves.
* **Sem agente por robô.** O micro-ROS precisa de um agente XRCE e de uma sessão por micro-controlador;
  o Zenoh-pico fala direto com o roteador do ROS 2.
* **Roteadores em malha.** Com mais de cerca de 30 drones, ou várias salas, use vários roteadores
  (um por AP WiFi) ligados entre si (`connect.endpoints` no `zenoh_router.json5`).
* **Broadcast.** `e1/all/cmd/*` chega a todos com uma mensagem só (parada de emergência, arm, mode).

## Algoritmos de enxame

* Controle centralizado: `e1_swarm_controller` (um nó, N drones) ou `e1_core.client.Swarm`.
* Controle distribuído: rode um nó por drone. Cada nó assina `/e1_<j>/pose` dos vizinhos e publica o
  próprio `/e1_<i>/setpoint`. O `separation_acc` em `e1_core.control` dá uma repulsão simples entre drones.
* Para N grande, publique setpoints direto em Zenoh (`e1_core.client`) e use o ROS só para visualizar e gravar.

## Boas práticas de rede

* Um AP dedicado ao enxame num canal de 2,4 GHz livre, com 20 MHz de largura (o ESP32-S3 não tem 5 GHz).
* Telemetria a 50 Hz por drone ocupa cerca de 4 kB/s (76 B + cabeçalhos). Comandos a 50 Hz, cerca de 2,5 kB/s.
  Na prática, cerca de 30 drones por AP com folga.
* Desligue o multicast do Zenoh (`scouting/multicast: false`, já assim no `zenoh_router.json5`).
