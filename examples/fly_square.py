#!/usr/bin/env python3
"""Fly one E1 Nano around a square (pure Zenoh, no ROS). Works with e1-sim or a real drone + mocap.

    e1-sim -n 1 &                      # or: a real drone publishing e1/1/state + mocap on e1/1/pose
    python3 examples/fly_square.py --id 1 --side 0.6 --z 0.4
"""
import argparse

import numpy as np

from e1_core.client import Swarm, connect
from e1_core.control import Waypoints, square

ap = argparse.ArgumentParser()
ap.add_argument('--connect', default='tcp/127.0.0.1:7447')
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--side', type=float, default=0.6)
ap.add_argument('--z', type=float, default=0.4)
ap.add_argument('--leg', type=float, default=2.5, help='seconds per side')
a = ap.parse_args()

session = connect(a.connect)
swarm = Swarm(session, ids=[a.id], discover=False)
swarm.wait_for(1)
d = swarm[a.id]
home = d.position.copy()
print(f'drone {a.id} at {np.round(home, 3)}, battery {d.state.battery_v:.2f} V')
try:
    swarm.takeoff(a.z)
    c = home[:2] + a.side / 2
    pts = Waypoints(swarm[a.id].setpoint(swarm.now())[0], square(c, a.side, a.z, a.leg, 0.5))
    swarm.follow(a.id, pts)
    swarm.fly({a.id: [home[0], home[1], a.z]}, 2.0)
    swarm.land()
    print('done; max tracking error is printed by the simulator log if enabled')
except KeyboardInterrupt:
    swarm.emergency_stop()
finally:
    swarm.close()
    session.close()
