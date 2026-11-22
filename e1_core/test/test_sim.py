"""Closed-loop tests: CF-Drone firmware model + MuJoCo + ground position controller."""
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from e1_core import protocol as P  # noqa: E402
from e1_core.control import PositionController, Waypoints, separation_acc, square  # noqa: E402
from e1_core.math3d import yaw_of  # noqa: E402
from e1_core.sim.mujoco_sim import SwarmSim, layout  # noqa: E402


def fly(sim, wps, seconds, every=20):
    ctl = {i: PositionController() for i in sim.drones}
    err = {i: [] for i in sim.drones}
    for k in range(int(seconds / sim.dt)):
        if k % every == 0:
            for i in sim.drones:
                p = sim.pose(i)
                sp, sv, sa = wps[i](sim.t)
                q, duty = ctl[i].update(sim.t, p.pos, p.vel, yaw_of(p.quat), sp, sv, sa, 0.0, sim.drones[i].battery_v, every * sim.dt)
                sim.push(i, 'cmd/attitude', P.CmdAttitude(tuple(q), (0, 0, 0), duty))
                if sim.t > 3.0:
                    err[i].append(np.linalg.norm(np.array(p.pos) - sp))
        sim.step()
    return err


def test_hover_and_square_tracking():
    sim = SwarmSim(n=1)
    s0 = np.array(sim.pose(1).pos)
    wp = Waypoints(s0, [((0, 0, 0.4), 2.5), ((0, 0, 0.4), 1.0)] + square((0.3, 0.3), 0.6, 0.4, 2.5, 0.5))
    err = fly(sim, {1: wp}, wp.t_end)
    assert max(err[1]) < 0.10, max(err[1])          # < 10 cm during the whole square
    assert sim.drones[1].fw.armed


def test_swarm_grid_and_failsafe():
    sim = SwarmSim(n=4, spacing=0.5)
    wps = {i: Waypoints(np.array(sim.pose(i).pos), [(np.array(sim.pose(i).pos) * [1, 1, 0] + [0, 0, 0.5], 2.5)]) for i in sim.drones}
    fly(sim, wps, 4.0)
    for i in sim.drones:
        assert abs(sim.pose(i).pos[2] - 0.5) < 0.05
    # stop sending commands: firmware must descend and disarm by itself (ZN_CMD_TIMEOUT + descend)
    for _ in range(int(6.0 / sim.dt)):
        sim.step()
    for i, d in sim.drones.items():
        assert sim.pose(i).pos[2] < 0.05 and not d.fw.armed


def test_emergency_disarm_broadcast():
    sim = SwarmSim(n=2)
    for i in sim.drones:
        sim.push(i, 'cmd/attitude', P.CmdAttitude((1, 0, 0, 0), (0, 0, 0), 0.6))
    for _ in range(50):
        sim.step()
    assert all(d.fw.armed for d in sim.drones.values())
    sim.push(P.BROADCAST, 'cmd/arm', P.CmdArm(False))
    for _ in range(5):
        sim.step()
    assert not any(d.fw.armed for d in sim.drones.values())


def test_layout_and_separation():
    pts = layout(5, 'grid', 0.5)
    assert len(pts) == 5 and abs(np.mean([p[1] for p in pts])) < 0.3
    acc = separation_acc(1, {1: [0, 0, 0.5], 2: [0.1, 0, 0.5]}, 0.25)
    assert acc[0] < 0      # pushed away from drone 2
