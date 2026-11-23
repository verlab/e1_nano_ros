#!/usr/bin/env python3
"""Record simulator runs as compact JSON for the 3D web viewer (docs/viewer/).

Each scenario runs lock-step (no Zenoh, no real time) with the same firmware model and ground
controller used everywhere else, and stores, at ``--fps``, the pose of every drone, its four
motor duties and its position setpoint.

    python tools/web_trajectories.py --out docs/viewer/data                 # MuJoCo scenarios
    python tools/web_trajectories.py --out docs/viewer/data --genesis       # + Genesis (needs genesis-world)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time

import numpy as np

from e1_core import protocol as P
from e1_core.control import (PositionController, YawAlign, circle_formation, grid_formation, min_jerk, separation_acc)
from e1_core.math3d import yaw_of


class Plan:
    """Piecewise minimum-jerk segments, one target dict {id: xyz} per segment."""

    def __init__(self, home, steps, pause=0.4):
        self.segs, t0, prev = [], 0.0, dict(home)
        for targets, dur in steps:
            full = {i: np.asarray(targets.get(i, prev[i]), float) for i in home}
            self.segs.append((t0, dur, prev, full))
            prev, t0 = full, t0 + dur + pause
        self.t_end = t0
        self.home = home

    def __call__(self, i, t):
        for s0, d, p0, p1 in reversed(self.segs):
            if t >= s0:
                return min_jerk(p0[i], p1[i], d, t - s0)
        return self.home[i], np.zeros(3), np.zeros(3)


def run(sim, plan, fps=30, align=None, min_sep=0.25, extra_time=1.0, ctrl_every=0.02):
    ids = list(sim.drones)
    align = align or {}
    ctl = {i: PositionController() for i in ids}
    ya = {i: YawAlign() for i in ids if align.get(i)}
    every_ctrl = max(1, int(round(ctrl_every / sim.dt)))
    every_rec = max(1, int(round(1 / (fps * sim.dt))))
    frames, sps = [], []
    sp_now = {i: np.array(sim.pose(i).pos) for i in ids}
    t_stop = plan.t_end
    for k in range(int((t_stop + extra_time) / sim.dt)):
        t = sim.t
        if k % every_ctrl == 0:
            pos = {i: np.array(sim.pose(i).pos) for i in ids}
            for i in ids:
                if t > t_stop:
                    sim.push(i, 'cmd/arm', P.CmdArm(False))
                    continue
                p = sim.pose(i)
                sp, sv, sa = plan(i, t)
                sp_now[i] = sp
                yaw = yaw_of(p.quat)
                q, duty = ctl[i].update(t, p.pos, p.vel, yaw, sp, sv, sa, 0.0, sim.drones[i].battery_v, ctrl_every,
                                        separation_acc(i, pos, min_sep) if min_sep > 0 else None)
                if i in ya:
                    ya[i].update(t, yaw_of(sim.drones[i].fw.attitude), yaw)
                    q = ya[i].apply(q)
                sim.push(i, 'cmd/attitude', P.CmdAttitude(tuple(q), (0, 0, 0), duty))
        sim.step()
        if k % every_rec == 0:
            row, sprow = [], []
            for i in ids:
                p = sim.pose(i)
                row += list(p.pos) + list(p.quat) + list(np.asarray(sim.drones[i].fw.motors, float))
                sprow += list(sp_now[i])
            frames.append([round(float(x), 4) for x in row])
            sps.append([round(float(x), 3) for x in sprow])
    return frames, sps


def write(out, name, meta, ids, frames, sps, fps):
    doc = dict(meta, name=name, fps=fps, ids=ids, stride=11, layout='x y z qw qx qy qz m_rl m_rr m_fr m_fl',
               duration=round(len(frames) / fps, 2), frames=frames, setpoints=sps)
    path = os.path.join(out, f'{name}.json')
    with open(path, 'w') as f:
        json.dump(doc, f, separators=(',', ':'))
    print(f'{path}: {len(frames)} frames, {len(ids)} drones, {os.path.getsize(path) / 1024:.0f} kB')
    return dict(name=name, title=meta['title'], backend=meta['backend'], drones=len(ids), duration=doc['duration'],
                description=meta['description'], file=f'data/{name}.json')


def square_steps(i, c, side, z, leg):
    pts = [(c[0] - side / 2, c[1] - side / 2), (c[0] + side / 2, c[1] - side / 2),
           (c[0] + side / 2, c[1] + side / 2), (c[0] - side / 2, c[1] + side / 2), (c[0] - side / 2, c[1] - side / 2)]
    return [({i: (x, y, z)}, leg) for x, y in pts]


def scenario_square(make_sim, fps):
    sim = make_sim(n=1)
    h = {1: np.array(sim.pose(1).pos)}
    steps = [({1: h[1] + [0, 0, 0.4]}, 2.5)] + square_steps(1, (0.3, 0.3), 0.6, 0.4, 2.5) + \
        [({1: h[1] + [0, 0, 0.4]}, 2.5), ({1: h[1] * [1, 1, 0] + [0, 0, -0.02]}, 2.5)]
    plan = Plan(h, steps)
    return [1], *run(sim, plan, fps, min_sep=0)


def scenario_swarm(make_sim, fps, n=6):
    sim = make_sim(n=n, spacing=0.5)
    ids = list(sim.drones)
    home = {i: np.array(sim.pose(i).pos) for i in ids}
    z = 0.5
    steps = [({i: home[i] * [1, 1, 0] + [0, 0, z] for i in ids}, 3.0)]
    grid = grid_formation(n, 0.45, z)
    steps.append(({i: grid[k] + [0, 0, 0.15 * (k % 2)] for k, i in enumerate(ids)}, 3.0))
    for k in range(4):
        steps.append((dict(zip(ids, circle_formation(n, 0.6, z=z, phase=k * math.pi / 4))), 3.0))
    steps.append(({i: home[i] * [1, 1, 0] + [0, 0, z] for i in ids}, 3.0))
    steps.append(({i: home[i] * [1, 1, 0] + [0, 0, -0.02] for i in ids}, 3.0))
    return ids, *run(sim, Plan(home, steps), fps)


def scenario_yaw(make_sim, fps):
    """Two drones with a 25 deg onboard heading error: #1 uses YawAlign, #2 does not."""
    sim = make_sim(n=2, spacing=1.2, yaw_offset=math.radians(25), yaw_drift=math.radians(1.0))
    ids = list(sim.drones)
    home = {i: np.array(sim.pose(i).pos) for i in ids}
    steps = [({i: home[i] + [0, 0, 0.4] for i in ids}, 2.5)]
    side, leg = 0.5, 2.5
    for dx, dy in ((side, 0), (side, side), (0, side), (0, 0)):
        steps.append(({i: home[i] + [dx - side / 2, dy - side / 2, 0.4] for i in ids}, leg))
    steps.append(({i: home[i] + [0, 0, 0.4] for i in ids}, 2.0))
    steps.append(({i: home[i] * [1, 1, 0] + [0, 0, -0.02] for i in ids}, 2.5))
    return ids, *run(sim, Plan(home, steps), fps, align={ids[0]: True}, min_sep=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='docs/viewer/data')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--genesis', action='store_true', help='also record the Genesis backend')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from e1_core.sim.mujoco_sim import SwarmSim
    mj = lambda **kw: SwarmSim(**kw)  # noqa: E731
    index = []
    t0 = time.time()
    ids, fr, sp = scenario_swarm(mj, a.fps)
    index.append(write(a.out, 'mujoco_swarm6', dict(
        backend='MuJoCo', title='Six-drone swarm',
        description='Takeoff, two-level grid, rotating circle and landing. Every drone runs the CF-Drone firmware '
                    'model and is closed through the ground position controller with inter-drone separation.'),
        ids, fr, sp, a.fps))
    ids, fr, sp = scenario_square(mj, a.fps)
    index.append(write(a.out, 'mujoco_square', dict(
        backend='MuJoCo', title='Square, one drone',
        description='0.6 m square at 0.4 m, 2.5 s per side, minimum-jerk segments.'), ids, fr, sp, a.fps))
    ids, fr, sp = scenario_yaw(mj, a.fps)
    index.append(write(a.out, 'mujoco_yaw_align', dict(
        backend='MuJoCo', title='Heading alignment (mocap vs. onboard yaw)',
        description='Both drones start with a 25 deg onboard heading error and 1 deg/s drift (no magnetometer). '
                    'Drone 1: commands rotated by YawAlign. Drone 2: raw mocap-frame commands.'), ids, fr, sp, a.fps))
    if a.genesis:
        from e1_core.sim.genesis_sim import GenesisSwarm
        gn = lambda **kw: GenesisSwarm(backend='cpu', **kw)  # noqa: E731
        ids, fr, sp = scenario_square(gn, a.fps)
        index.append(write(a.out, 'genesis_square', dict(
            backend='Genesis', title='Square, one drone (Genesis)',
            description='Same mission and controller as the MuJoCo square, on the Genesis rigid-body backend.'),
            ids, fr, sp, a.fps))
    with open(os.path.join(a.out, 'index.json'), 'w') as f:
        json.dump(index, f, indent=1)
    print(f'done in {time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
