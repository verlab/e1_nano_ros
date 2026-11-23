# 7. Writing your own controller

There are three levels at which you can command the drone:

| Level | Zenoh key / ROS topic | Rate | Who stabilizes |
|---|---|---|---|
| Position | `/e1_<id>/setpoint` (`PositionTarget`) | ≥ 5 Hz | `e1_swarm_controller` + firmware |
| Attitude + thrust | `e1/<id>/cmd/attitude` · `/e1_<id>/cmd/attitude` | ≥ 10 Hz (50 Hz recommended) | firmware (CF-Drone attitude, rate and mixer) |
| Motors | `e1/<id>/cmd/motors` · `/e1_<id>/cmd/motors` | ≥ 10 Hz | you (system identification on a bench only) |

Minimal example in plain Python, a hover with P control of altitude:

```python
import time, numpy as np
from e1_core.client import connect, Drone

s = connect('tcp/127.0.0.1:7447')
d = Drone(s, 1)
while d.pose is None:
    time.sleep(0.05)
z_ref = 0.4
for _ in range(500):                       # 10 s at 50 Hz
    ez = z_ref - d.pose.pos[2]
    d.attitude((1, 0, 0, 0), thrust=0.55 + 0.3 * ez)   # level quaternion + thrust (duty)
    time.sleep(0.02)
d.attitude((1, 0, 0, 0), thrust=0.0)       # thrust 0 disarms
```

Use the simulator and the tests (`e1_core/test/test_sim.py`) as a test bench before flying.
