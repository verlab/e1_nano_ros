"""Onboard heading error (no magnetometer) vs mocap heading: YawAlign must remove it."""
import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from e1_core import protocol as P  # noqa: E402
from e1_core.control import PositionController, Waypoints, YawAlign, square  # noqa: E402
from e1_core.math3d import yaw_of  # noqa: E402
from e1_core.sim.mujoco_sim import SwarmSim  # noqa: E402


def fly_square(align, yaw_offset=math.radians(70), yaw_drift=math.radians(2.0), every=20):
    sim = SwarmSim(n=1, yaw_offset=yaw_offset, yaw_drift=yaw_drift)
    s0 = np.array(sim.pose(1).pos)
    wp = Waypoints(s0, [((0, 0, 0.4), 2.5), ((0, 0, 0.4), 1.0)] + square((0.3, 0.3), 0.6, 0.4, 2.5, 0.5))
    ctl, ya, err = PositionController(), YawAlign(), []
    for k in range(int(wp.t_end / sim.dt)):
        if k % every == 0:
            p = sim.pose(1)
            sp, sv, sa = wp(sim.t)
            yaw = yaw_of(p.quat)
            q, duty = ctl.update(sim.t, p.pos, p.vel, yaw, sp, sv, sa, 0.0, sim.drones[1].battery_v, every * sim.dt)
            if align:
                ya.update(sim.t, yaw_of(sim.drones[1].fw.attitude), yaw)
                q = ya.apply(q)
            sim.push(1, 'cmd/attitude', P.CmdAttitude(tuple(q), (0, 0, 0), duty))
            if sim.t > 3.0:
                err.append(np.linalg.norm(np.array(p.pos) - sp))
        sim.step()
    return max(err) if err else math.inf, ya.offset


def test_heading_error_breaks_tracking_without_alignment():
    e, _ = fly_square(align=False)
    assert e > 0.25, e


def test_yaw_align_restores_tracking():
    e, off = fly_square(align=True)
    assert e < 0.10, e
    assert off is not None


def test_yaw_align_wraps():
    ya = YawAlign(tau=0.0)
    assert abs(ya.update(0.0, math.radians(179), math.radians(-179)) - math.radians(-2)) < 1e-9
