#!/usr/bin/env python3
"""Swarm demo (pure Zenoh): discover every e1/*/alive drone, take off, grid -> rotating circle -> land.

    e1-sim -n 6 --layout grid &
    python3 examples/swarm_formation.py --n 6
"""
import argparse
import math

import numpy as np

from e1_core.client import Swarm, connect
from e1_core.control import circle_formation, grid_formation

ap = argparse.ArgumentParser()
ap.add_argument('--connect', default='tcp/127.0.0.1:7447')
ap.add_argument('--n', type=int, default=4, help='number of drones to wait for')
ap.add_argument('--z', type=float, default=0.5)
ap.add_argument('--radius', type=float, default=0.6)
a = ap.parse_args()

session = connect(a.connect)
swarm = Swarm(session)
ids = swarm.wait_for(a.n)[:a.n]
print('swarm:', ids)
home = {i: np.array(swarm[i].pose.pos) for i in ids}
try:
    swarm.takeoff(a.z, ids=ids)
    # stack in a grid with alternating altitude
    grid = grid_formation(len(ids), spacing=0.45, z=a.z)
    swarm.fly({i: p + [0, 0, 0.15 * (k % 2)] for k, (i, p) in enumerate(zip(ids, grid))}, 3.0)
    # circle, then rotate it by 2 x 45 degrees
    for k in range(3):
        circ = circle_formation(len(ids), a.radius, z=a.z, phase=k * math.pi / 4)
        swarm.fly(dict(zip(ids, circ)), 3.0)
        swarm.sleep(0.5)
    # back over the start positions and land
    swarm.fly({i: home[i] * [1, 1, 0] + [0, 0, a.z] for i in ids}, 3.0)
    swarm.land(ids=ids)
except KeyboardInterrupt:
    swarm.emergency_stop()
finally:
    swarm.close()
    session.close()
