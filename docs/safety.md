# Flight safety

!!! danger "Propellers"
    31 mm propellers at ~38 000 rpm cut skin. Wear safety glasses, fly inside a safety net and
    remove the propellers for any bench test (`mfl`, `cmd/motors`).

## Protection layers

| Layer | Trigger | Action |
|---|---|---|
| Firmware (CF-Drone) | critical battery | `descend()` and disarm |
| Firmware (CF-Drone) | inverted > 1.5 s | disarm |
| Firmware (CF-Drone) | RC/Web RC loss | `descend()` |
| **Firmware (E1 link)** | **no `cmd/attitude`/`cmd/motors` for `ZN_TIMEOUT` (0.5 s) in AUTO** | `descend()` |
| Firmware | stick moved during AUTO | back to STAB (pilot takes over) |
| Ground | `/e1/emergency_stop`, `e1-cli estop` | disarms **all** drones (`e1/all/cmd/arm`) |
| Ground | `min_separation` in the controller | repulsion between drones |

## Pre-flight checklist

1. Battery > 3.9 V (`e1-cli status`).
2. Each drone's pose is correct in RViz (position **and** yaw).
3. `ps` reports roll/pitch < 2°.
4. A terminal with `e1-cli estop` ready.
5. First flight of the day: one drone, 30 cm, 10 s hover.
6. Swarm: initial spacing ≥ 0.4 m; different altitudes reduce the *downwash* risk.
