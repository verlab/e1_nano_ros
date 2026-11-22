# 7. Escrevendo seu próprio controlador

Há três níveis de onde você pode comandar:

| Nível | Chave Zenoh / tópico ROS | Taxa | Quem estabiliza |
|---|---|---|---|
| Posição | `/e1_<id>/setpoint` (`PositionTarget`) | ≥ 5 Hz | `e1_swarm_controller` + firmware |
| Atitude + empuxo | `e1/<id>/cmd/attitude` · `/e1_<id>/cmd/attitude` | ≥ 10 Hz (50 Hz recomendado) | firmware (atitude, taxa e mixer do CF-Drone) |
| Motores | `e1/<id>/cmd/motors` · `/e1_<id>/cmd/motors` | ≥ 10 Hz | você (só para identificação de sistema, em bancada) |

Exemplo mínimo em Python puro, um hover com controle P de altura:

```python
import time, numpy as np
from e1_core.client import connect, Drone

s = connect('tcp/127.0.0.1:7447')
d = Drone(s, 1)
while d.pose is None:
    time.sleep(0.05)
z_ref = 0.4
for _ in range(500):                       # 10 s a 50 Hz
    ez = z_ref - d.pose.pos[2]
    d.attitude((1, 0, 0, 0), thrust=0.55 + 0.3 * ez)   # quaternion nivelado + empuxo (duty)
    time.sleep(0.02)
d.attitude((1, 0, 0, 0), thrust=0.0)       # empuxo 0 desarma
```

Use o simulador e os testes (`e1_core/test/test_sim.py`) como bancada antes de voar.
