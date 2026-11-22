# Protocolo Zenoh E1 (v1)

Fonte da verdade: `e1_core/e1_core/protocol.py` (Python) e `firmware/src_e1/e1_protocol.h` (C).
Os testes conferem que os tamanhos batem.

Structs little-endian e empacotadas. Todas começam com um cabeçalho de 8 bytes:

| campo | tipo | descrição |
|---|---|---|
| version | u8 | `1` |
| type | u8 | 1 STATE, 2 POSE, 3 CMD_ATTITUDE, 4 CMD_MANUAL, 5 CMD_MOTORS, 6 CMD_ARM, 7 CMD_MODE, 8 CMD_PARAM |
| seq | u16 | contador do emissor |
| t_ms | u32 | relógio do emissor (ms) |

## Chaves

| chave | sentido | conteúdo | taxa |
|---|---|---|---|
| `e1/<id>/alive` | drone → | token de liveliness (descoberta / presença) | — |
| `e1/<id>/state` | drone → | STATE (76 B) | 50 Hz |
| `e1/<id>/log` | drone → | texto do console | eventual |
| `e1/<id>/pose` | → drone | POSE (48 B): mocap ou simulador | 50–200 Hz |
| `e1/<id>/cmd/attitude` | → drone | CMD_ATTITUDE (41 B), entra em AUTO | 10–100 Hz |
| `e1/<id>/cmd/manual` | → drone | CMD_MANUAL: sticks virtuais | 20–50 Hz |
| `e1/<id>/cmd/motors` | → drone | CMD_MOTORS: duty direto (AUTO) | ≥ 10 Hz |
| `e1/<id>/cmd/arm` | → drone | CMD_ARM | eventual |
| `e1/<id>/cmd/mode` | → drone | CMD_MODE (0 RAW, 1 ACRO, 2 STAB, 3 ALTHOLD, 4 AUTO) | eventual |
| `e1/<id>/cmd/param` | → drone | CMD_PARAM (nome[16] + float) | eventual |
| `e1/<id>/cmd/console` | → drone | linha de comando do CF-Drone (UTF-8) | eventual |
| `e1/all/cmd/<x>` | → todos | qualquer comando acima, em broadcast | — |

## STATE

`quat[4]` (w,x,y,z; corpo FLU → mundo) · `rates[3]` rad/s · `acc[3]` m/s² (força específica, corpo) ·
`motors[4]` duty (RL, RR, FR, FL) · `thrust` · `battery_v` · `mode` u8 · `armed` u8 · `flags` u16
(1 LANDED, 2 LOW_BATTERY, 4 CMD_TIMEOUT, 8 FAILSAFE, 16 IMU_ERROR, 32 POSE_VALID).

## CMD_ATTITUDE

`quat[4]` alvo · `rates[3]` feed-forward · `thrust` duty coletivo [0,1] · `type_mask` u8
(1 = ignorar rates, 2 = ignorar atitude). Empuxo > 0 arma; empuxo 0 desarma. Mesma semântica do
`SET_ATTITUDE_TARGET` do MAVLink no CF-Drone, mas já em FLU.

## Unidades e convenções

* Frame do corpo **FLU** (x frente, y esquerda, z cima) e mundo ENU (z para cima).
* "Empuxo" é o duty coletivo nativo do firmware: ≈0,53 paira com bateria cheia. Para converter newtons
  em duty, use `VehicleModel.duty_for(F, Vbat)` (empuxo ∝ duty² · V²).
* Ordem dos motores = índices do CF-Drone: 0 trás-esq. (CCW), 1 trás-dir. (CW), 2 frente-dir. (CCW), 3 frente-esq. (CW).

## Compatibilidade

Mudanças incompatíveis incrementam `VERSION`. Receptores descartam versões desconhecidas, e o firmware
conta essas mensagens em `zn` → `err`.
