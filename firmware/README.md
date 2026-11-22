# E1 Nano firmware — CF-Drone + E1 Zenoh link

O firmware de voo é o **CF-Drone** original (Cherglow, `external/CF-Drone`, submódulo git fixado
no commit `83ebe61`), sem modificações no código-fonte. A pasta `src_e1/` adiciona o **E1 Zenoh link**
(zenoh-pico 1.10.1), que coloca o drone diretamente na malha Zenoh usada pelo ROS 2
(`rmw_zenoh_cpp`). O script `tools/prepare_src.py` copia o CF-Drone para `.e1_src/`, adiciona
`src_e1/` e insere cinco ganchos de uma linha cada (verificados por âncora):

| arquivo        | gancho                     | função                                                    |
|----------------|----------------------------|-----------------------------------------------------------|
| `CF-Drone.ino` | `setupZenohLink()`         | inicia a tarefa Zenoh no core 0                           |
| `CF-Drone.ino` | `zenohLinkApply()`         | aplica comandos recebidos (antes de `estimate/control`)   |
| `CF-Drone.ino` | `zenohLinkSnapshot()`      | copia a telemetria (depois de `sendMotors`)               |
| `cli.ino`      | `zenohLinkCommand(...)`    | comandos de console `zn ...`                              |
| `cli.ino`      | `zenohLinkLog(buf)`        | espelha o console em `e1/<id>/log`                        |

## Compilar e gravar

```bash
git submodule update --init --recursive
cd firmware
pio run -e e1_nano                 # ESP32-S3 SuperMini (E1 Nano v1.5Y)
pio run -e e1_nano -t upload       # segure BOOT ao conectar o USB na primeira vez
pio device monitor
```

Sem PlatformIO instalado: `docker compose run --rm firmware pio run` (veja `docker/Dockerfile.firmware`).
O zenoh-pico chama `cmake` durante o build (já incluso na imagem Docker).

## Verificação sem ESP32

`test/host_check/run.sh` compila o `zenoh_link.ino` no PC contra os headers reais do zenoh-pico (plataforma Unix) e do
CF-Drone, com stubs mínimos de Arduino/FreeRTOS. Pega erros de API sem precisar do toolchain do ESP32.

## Configurar (console serial, Web RC ou `e1-cli console`)

```text
sta MinhaRede MinhaSenha        # WiFi do laboratório (modo estação)
p WIFI_MODE 2                   # 2 = STA (no código: W_AP=1, W_STA=2)
zn id 3                         # este drone vira e1/3
zn connect tcp/192.168.1.100:7447   # IP do PC com o roteador Zenoh (rmw_zenohd)
zn timeout 0.5                  # failsafe: sem comandos AUTO por 0,5 s -> descend()
reboot
zn                              # status: conectado, rx/tx/erros, IP
```

Antes do primeiro voo faça a calibração do IMU do CF-Drone (`ca`, depois `ps`), como na página do projeto.

## Comportamento

* **Núcleos:** o laço de voo do CF-Drone continua no core 1. O Zenoh roda numa tarefa FreeRTOS no core 0
  e troca dados com o laço por uma caixa de correio protegida por spinlock. A rede nunca bloqueia o controle.
* **Telemetria:** `e1/<id>/state` a 50 Hz (76 bytes). O console é espelhado em `e1/<id>/log`. A descoberta
  usa o token de liveliness `e1/<id>/alive`.
* **Comandos:** `e1/<id>/cmd/{attitude,manual,motors,arm,mode,param,console}` e os mesmos em `e1/all/cmd/*` (broadcast).
  `cmd/attitude` tem a mesma semântica do `SET_ATTITUDE_TARGET` do MAVLink do CF-Drone, já em FLU.
* **Failsafes:** valem todos os do CF-Drone (bateria, invertido, perda de RC). Além deles, em AUTO, se o fluxo
  de comandos parar por `ZN_TIMEOUT`, o drone chama `descend()`. Mexer nos sticks (RC/Web RC) tira o drone do AUTO.
* **Reconexão:** se o WiFi ou o roteador cair, a tarefa reabre a sessão sozinha.

## Licença

O CF-Drone é GPL-3.0 com cláusula adicional de restrição comercial do autor (veja `external/CF-Drone/LICENSE`).
O firmware combinado (`src_e1/` + CF-Drone) segue essas condições: uso acadêmico e não comercial. O restante
do repositório (pacotes ROS 2, simulador e documentação) é Apache-2.0.
