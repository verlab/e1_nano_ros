# Segurança de voo

!!! danger "Hélices"
    Hélices de 31 mm a ~38 000 rpm cortam pele. Use óculos de proteção, voe dentro de rede de proteção e
    tire as hélices em qualquer teste de bancada (`mfl`, `cmd/motors`).

## Camadas de proteção

| Camada | Gatilho | Ação |
|---|---|---|
| Firmware (CF-Drone) | bateria crítica | `descend()` e desarme |
| Firmware (CF-Drone) | invertido > 1,5 s | desarma |
| Firmware (CF-Drone) | perda de RC/Web RC | `descend()` |
| **Firmware (E1 link)** | **sem `cmd/attitude`/`cmd/motors` por `ZN_TIMEOUT` (0,5 s) em AUTO** | `descend()` |
| Firmware | stick mexido durante AUTO | volta para STAB (piloto assume) |
| Solo | `/e1/emergency_stop`, `e1-cli estop` | desarma **todos** (`e1/all/cmd/arm`) |
| Solo | `min_separation` no controlador | repulsão entre drones |

## Checklist antes de cada voo

1. Bateria > 3,9 V (`e1-cli status`).
2. Pose de cada drone correta no RViz (posição **e** yaw).
3. `ps` com roll/pitch < 2°.
4. Terminal com `e1-cli estop` pronto.
5. Primeiro voo do dia: um drone, 30 cm, hover de 10 s.
6. Enxame: espaçamento inicial ≥ 0,4 m; altitudes diferentes reduzem o risco de *downwash*.
