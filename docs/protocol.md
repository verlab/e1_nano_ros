# E1 Zenoh protocol (v1)

Source of truth: `e1_core/e1_core/protocol.py` (Python) and `firmware/src_e1/e1_protocol.h` (C).
The tests check that the sizes match.

Packed little-endian structs. All of them start with an 8-byte header:

| field | type | description |
|---|---|---|
| version | u8 | `1` |
| type | u8 | 1 STATE, 2 POSE, 3 CMD_ATTITUDE, 4 CMD_MANUAL, 5 CMD_MOTORS, 6 CMD_ARM, 7 CMD_MODE, 8 CMD_PARAM |
| seq | u16 | sender counter |
| t_ms | u32 | sender clock (ms) |

## Keys

| key | direction | content | rate |
|---|---|---|---|
| `e1/<id>/alive` | drone → | liveliness token (discovery / presence) | — |
| `e1/<id>/state` | drone → | STATE (76 B) | 50 Hz |
| `e1/<id>/log` | drone → | console text | on event |
| `e1/<id>/pose` | → drone | POSE (48 B): mocap or simulator | 50–200 Hz |
| `e1/<id>/cmd/attitude` | → drone | CMD_ATTITUDE (41 B), switches to AUTO | 10–100 Hz |
| `e1/<id>/cmd/manual` | → drone | CMD_MANUAL: virtual sticks | 20–50 Hz |
| `e1/<id>/cmd/motors` | → drone | CMD_MOTORS: direct duty (AUTO) | ≥ 10 Hz |
| `e1/<id>/cmd/arm` | → drone | CMD_ARM | on event |
| `e1/<id>/cmd/mode` | → drone | CMD_MODE (0 RAW, 1 ACRO, 2 STAB, 3 ALTHOLD, 4 AUTO) | on event |
| `e1/<id>/cmd/param` | → drone | CMD_PARAM (name[16] + float) | on event |
| `e1/<id>/cmd/console` | → drone | CF-Drone command line (UTF-8) | on event |
| `e1/all/cmd/<x>` | → all | any of the commands above, as broadcast | — |

## STATE

`quat[4]` (w,x,y,z; body FLU → world) · `rates[3]` rad/s · `acc[3]` m/s² (specific force, body) ·
`motors[4]` duty (RL, RR, FR, FL) · `thrust` · `battery_v` · `mode` u8 · `armed` u8 · `flags` u16
(1 LANDED, 2 LOW_BATTERY, 4 CMD_TIMEOUT, 8 FAILSAFE, 16 IMU_ERROR, 32 POSE_VALID).

## CMD_ATTITUDE

`quat[4]` target · `rates[3]` feed-forward · `thrust` collective duty [0,1] · `type_mask` u8
(1 = ignore rates, 2 = ignore attitude). Thrust > 0 arms; thrust 0 disarms. Same semantics as the CF-Drone
MAVLink `SET_ATTITUDE_TARGET`, but already in FLU.

## Units and conventions

* **FLU** body frame (x forward, y left, z up) and ENU world frame (z up).
* "Thrust" is the firmware's native collective duty: ≈0.53 hovers with a full battery. To convert newtons
  to duty, use `VehicleModel.duty_for(F, Vbat)` (thrust ∝ duty² · V²).
* Motor order = CF-Drone indices: 0 rear-left (CCW), 1 rear-right (CW), 2 front-right (CCW), 3 front-left (CW).

## Compatibility

Incompatible changes increment `VERSION`. Receivers discard unknown versions, and the firmware
counts those messages in `zn` → `err`.
