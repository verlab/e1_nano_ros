#!/usr/bin/env python3
"""Render the heading-alignment comparison (offline, lock-step) to mp4.

Two drones with the same onboard heading error (no magnetometer) fly the same square:
drone 1 sends YawAlign-corrected commands, drone 2 sends raw mocap-frame commands.

    python3 examples/render_yaw_align.py --out yaw_align.mp4 --offset 25 --drift 1
"""
import argparse
import math
import os
import sys

import imageio.v2 as iio
import mujoco
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from web_trajectories import Plan  # noqa: E402

from e1_core import protocol as P  # noqa: E402
from e1_core.control import PositionController, YawAlign  # noqa: E402
from e1_core.math3d import yaw_of  # noqa: E402
from e1_core.sim.mujoco_sim import SwarmSim  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='yaw_align.mp4')
ap.add_argument('--offset', type=float, default=25.0, help='onboard heading error, deg')
ap.add_argument('--drift', type=float, default=1.0, help='onboard heading drift, deg/s')
ap.add_argument('--fps', type=int, default=30)
ap.add_argument('--width', type=int, default=1280)
ap.add_argument('--height', type=int, default=720)
a = ap.parse_args()

sim = SwarmSim(n=2, spacing=1.2, yaw_offset=math.radians(a.offset), yaw_drift=math.radians(a.drift))
ids = list(sim.drones)
home = {i: np.array(sim.pose(i).pos) for i in ids}
side = 0.5
steps = [({i: home[i] + [0, 0, 0.4] for i in ids}, 2.5)]
for dx, dy in ((side, 0), (side, side), (0, side), (0, 0)):
    steps.append(({i: home[i] + [dx - side / 2, dy - side / 2, 0.4] for i in ids}, 2.5))
steps.append(({i: home[i] + [0, 0, 0.4] for i in ids}, 2.0))
steps.append(({i: home[i] * [1, 1, 0] + [0, 0, -0.02] for i in ids}, 2.5))
plan = Plan(home, steps)

ctl = {i: PositionController() for i in ids}
ya = YawAlign()
COL = {ids[0]: [0.91, 0.35, 0.05, 1], ids[1]: [0.11, 0.49, 0.84, 1]}
trail = {i: [] for i in ids}

r = mujoco.Renderer(sim.model, a.height, a.width)
cam = mujoco.MjvCamera()
mid = (home[ids[0]] + home[ids[1]]) / 2
cam.lookat[:] = [mid[0], mid[1], 0.3]
cam.distance, cam.azimuth, cam.elevation = 2.6, -90.0, -38.0
w = iio.get_writer(a.out, fps=a.fps, codec='libx264', quality=8, macro_block_size=8)
every_render = int(round(1 / (a.fps * sim.dt)))


def add_geom(scn, kind, size, pos, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, kind, np.asarray(size, float), np.asarray(pos, float), np.eye(3).flatten(), np.asarray(rgba, np.float32))
    scn.ngeom += 1


for k in range(int((plan.t_end + 1.0) / sim.dt)):
    t = sim.t
    if k % 20 == 0:
        for i in ids:
            if t > plan.t_end:
                sim.push(i, 'cmd/arm', P.CmdArm(False))
                continue
            p = sim.pose(i)
            sp, sv, sa = plan(i, t)
            yaw = yaw_of(p.quat)
            q, duty = ctl[i].update(t, p.pos, p.vel, yaw, sp, sv, sa, 0.0, sim.drones[i].battery_v, 0.02)
            if i == ids[0]:
                ya.update(t, yaw_of(sim.drones[i].fw.attitude), yaw)
                q = ya.apply(q)
            sim.push(i, 'cmd/attitude', P.CmdAttitude(tuple(q), (0, 0, 0), duty))
    sim.step()
    if k % every_render == 0:
        for i in ids:
            trail[i].append(np.array(sim.pose(i).pos))
        r.update_scene(sim.data, camera=cam)
        scn = r.scene
        for i in ids:                                    # reference square
            c = home[i] + [0, 0, 0.4]
            for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.012, 0, 0], c + [dx * side / 2, dy * side / 2, 0], [0.4, 0.4, 0.4, 0.6])
            for pt in trail[i][-300::3]:
                add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.005, 0, 0], pt, COL[i])
        w.append_data(r.render())
w.close()
err = {i: max(np.linalg.norm(trail[i][n] - plan(i, n / a.fps)[0]) for n in range(int(3 * a.fps), int(plan.t_end * a.fps)))
       for i in ids}
print('written', a.out, 'max error (m):', {i: round(float(e), 3) for i, e in err.items()})
