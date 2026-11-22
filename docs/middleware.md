# Escolha do middleware: Zenoh, micro-ROS ou outro?

**Decisão: Zenoh de ponta a ponta.** O drone roda zenoh-pico (cliente) e o ROS 2 roda `rmw_zenoh_cpp`.
Os dois compartilham o mesmo roteador. O drone usa um protocolo binário enxuto e versionado
(`e1/<id>/**`), e o `e1_bridge` expõe tudo como ROS 2 idiomático.

## Requisitos

1. Dezenas de drones na mesma rede WiFi 2,4 GHz (ESP32-S3, 512 kB de SRAM, 2 núcleos).
2. Comandos a 50 Hz por drone com baixa latência. O laço de controle (≈1 kHz) não pode bloquear na rede.
3. Integração nativa com ROS 2 (RViz, rosbag, Nav2/planejadores) **e** acesso sem ROS (scripts, RL, notebooks).
4. Mesma interface para simulação e hardware.
5. Pouca configuração por drone (enxames grandes).

## Comparação

| Critério | **Zenoh (zenoh-pico + rmw_zenoh)** | micro-ROS (DDS-XRCE) | DDS direto no ESP32 | MAVLink (já no CF-Drone) |
|---|---|---|---|---|
| Descoberta com N robôs | por roteador: cada drone declara só suas chaves | o agente faz a ponte para DDS: a descoberta DDS continua lá | O(N²), multicast, inviável em WiFi | não tem: IDs fixos |
| Infra por drone | nenhuma (1 roteador para todos) | 1 sessão no agente; agente por link | — | ponte MAVROS por drone |
| Integração ROS 2 | `rmw_zenoh` é **Tier-1 desde o Kilted** (mesma malha) | nativa (tipos ROS) | nativa | via MAVROS (pesado para N drones) |
| Acesso sem ROS | APIs Python/C/Rust/JS | precisa de ROS | precisa de DDS | pymavlink |
| Broadcast para o enxame | `e1/all/cmd/*` (1 msg) | 1 msg por drone | 1 msg por drone | por sysid |
| Pegada no MCU | pequena (zenoh-pico) | média (rclc + XRCE) | grande | pequena |
| Topologias | cliente, peer, roteadores em malha, multi-sala | estrela no agente | malha DDS | ponto a ponto |
| Maturidade no ESP32 | Arduino/ESP-IDF oficiais | Arduino/ESP-IDF oficiais | experimental | madura |

## Por que não micro-ROS?

O micro-ROS funciona bem para um robô ou poucos. Em enxame, porém, todo tráfego passa pelo agente
XRCE, e o agente re-publica em DDS, de modo que a explosão de descoberta do DDS volta do lado do PC.
Mesmo nos cenários típicos de enxame, a comunicação micro-ROS é sempre intermediada por um broker, com a
complexidade de implantar e configurar esse broker. Existe ainda um `rmw_zenoh_pico` para micro-ROS
(esol-community), que é promissor, mas está fora do micro-ROS oficial e é experimental.

## Por que não usar `rmw_zenoh` byte a byte no drone?

Seria possível publicar CDR nas chaves internas do `rmw_zenoh` (`<domain>/<topic>/<type>/<hash>`) e
declarar os tokens de liveliness do grafo ROS a partir do ESP32. Isso, porém, acopla o firmware a
detalhes internos do RMW (houve uma quebra de compatibilidade de fio no Jazzy, por exemplo) e gasta mais
banda por mensagem. Escolhemos:

* **firmware → protocolo E1 estável e versionado** (structs de 9 a 76 bytes, `VERSION=1`);
* **`e1_bridge` → ROS 2 idiomático** (tipos padrão + `e1_msgs`). É um único processo para o enxame inteiro,
  na mesma malha Zenoh, então não há salto de rede extra.

Se no futuro quiser o drone como "nó ROS de verdade", só o módulo `zenoh_link.ino` muda.

## Versões

| Componente | Versão |
|---|---|
| ROS 2 | Jazzy (LTS) ou Kilted/Lyrical |
| rmw_zenoh_cpp | binário da distro (zenoh-c 1.10.x embutido) |
| zenoh-pico (firmware) | 1.10.1 |
| eclipse-zenoh (Python) | 1.10.1 |

## Referências

* ROS 2 Kilted Kaiju: primeira distro com Zenoh como middleware Tier-1 — discourse.ros.org
* Design do rmw_zenoh — github.com/ros2/rmw_zenoh
* zenoh-pico: suporte Arduino/ESP-IDF, UDP/TCP/serial — github.com/eclipse-zenoh/zenoh-pico
* *Performance Comparison of ROS2 Middlewares for Multi-robot Mesh Networks* (J. Intell. Robot. Syst., 2024): menor atraso e menor uso de CPU com Zenoh em malha dinâmica
* *Discovery Storm: Scalability Analysis of DDS and Zenoh in Large-Scale Wireless Robotic Networks* (DGIST, 2026)
* *Edge Robotics with ROS2 and Eclipse Zenoh* (EclipseCon): limitações do micro-ROS intermediado por broker em enxames
* rmw_zenoh_pico para micro-ROS (esol-community) — discourse.openrobotics.org
