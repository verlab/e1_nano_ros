"""``e1-sim``: run N simulated E1 Nano drones on Zenoh (MuJoCo SITL).

Examples
--------
    e1-sim -n 4                                  # 4 drones, connect to a router on localhost:7447
    e1-sim -n 1 --mode peer --listen tcp/0.0.0.0:7447   # standalone, no router needed
    e1-sim -n 6 --layout circle --viewer          # MuJoCo viewer (needs a display)
    e1-sim -n 3 --record swarm.mp4 --duration 30  # headless video
    e1-sim -n 2 --natnet 127.0.0.1 --yaw-offset 60   # emulate OptiTrack: poses only via NatNet (use e1-mocap)
"""
from __future__ import annotations

import argparse
import math
import signal
import sys

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(prog='e1-sim', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-n', '--num', type=int, default=1, help='number of drones')
    ap.add_argument('--first-id', type=int, default=1)
    ap.add_argument('--layout', choices=['grid', 'line', 'circle'], default='grid')
    ap.add_argument('--spacing', type=float, default=0.5, help='start spacing (m)')
    ap.add_argument('--connect', default='tcp/127.0.0.1:7447', help='Zenoh endpoint to connect ("" = none)')
    ap.add_argument('--listen', default=None, help='Zenoh endpoint to listen on (peer mode)')
    ap.add_argument('--mode', default='client', choices=['client', 'peer'])
    ap.add_argument('--speed', type=float, default=1.0, help='real-time factor (0 = as fast as possible)')
    ap.add_argument('--duration', type=float, default=math.inf)
    ap.add_argument('--viewer', action='store_true', help='open the MuJoCo viewer')
    ap.add_argument('--record', default=None, help='write an mp4 (offscreen rendering)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--backend', choices=['mujoco', 'genesis'], default='mujoco')
    ap.add_argument('--yaw-offset', type=float, default=0.0, help='onboard heading error at start, deg (MuJoCo)')
    ap.add_argument('--yaw-drift', type=float, default=0.0, help='onboard heading drift, deg/s (MuJoCo)')
    ap.add_argument('--natnet', default=None, metavar='IP',
                    help='stream ground truth as a fake Motive (NatNet) on this IP instead of publishing e1/<id>/pose')
    ap.add_argument('--natnet-up', choices=['z', 'y'], default='z', help='up axis of the fake Motive stream')
    ap.add_argument('--natnet-rate', type=float, default=120.0)
    ap.add_argument('--device', choices=['gpu', 'cpu'], default='gpu', help='Genesis backend device')
    a = ap.parse_args(argv)

    from ..client import connect
    from .mujoco_sim import SwarmSim, ZenohSimServer
    import mujoco

    ids = list(range(a.first_id, a.first_id + a.num))
    if a.backend == 'genesis':
        from .genesis_sim import GenesisSwarm
        sim = GenesisSwarm(ids=ids, kind=a.layout, spacing=a.spacing, backend=a.device, show_viewer=a.viewer)
        a.viewer = False
        if a.record:
            print('[e1-sim] --record is only implemented for the MuJoCo backend', file=sys.stderr)
            a.record = None
    else:
        sim = SwarmSim(ids=ids, kind=a.layout, spacing=a.spacing,
                       yaw_offset=math.radians(a.yaw_offset), yaw_drift=math.radians(a.yaw_drift))
    session = connect(a.connect or None, a.mode, a.listen)
    srv = ZenohSimServer(sim, session, speed=a.speed, publish_pose=a.natnet is None)
    natnet = None
    if a.natnet:
        from ..mocap.frames import from_z_up
        from ..mocap.natnet import RigidBody
        from ..mocap.server import NatNetServer

        def bodies():
            with srv.lock:
                poses = {did: sim.pose(did) for did in sim.drones}
            return [RigidBody(did, *from_z_up(p.pos, p.quat, a.natnet_up)) for did, p in poses.items()]
        natnet = NatNetServer(bodies, {did: f'e1_{did}' for did in ids}, ip=a.natnet).start(a.natnet_rate)
        print(f'[e1-sim] fake Motive on {a.natnet}:1510 (unicast, up={a.natnet_up}); run e1-mocap --server {a.natnet} --unicast',
              flush=True)
    print(f'[e1-sim] {len(ids)} drone(s) {ids} on Zenoh ({a.mode}, connect={a.connect!r}, listen={a.listen!r})', flush=True)

    signal.signal(signal.SIGINT, lambda *_: srv.stop())
    signal.signal(signal.SIGTERM, lambda *_: srv.stop())

    on_step = None
    writer = renderer = None
    if a.record:
        import imageio.v2 as iio
        renderer = mujoco.Renderer(sim.model, 720, 1280)
        writer = iio.get_writer(a.record, fps=a.fps, codec='libx264', quality=8, macro_block_size=8)
        cam = mujoco.MjvCamera()
        every = max(1, int(round(1 / (a.fps * sim.dt))))
        bodies = [d.body for d in sim.drones.values()]

        def on_step(s):  # noqa: F811
            if s.k % every:
                return
            c = s.data.xpos[bodies].mean(axis=0)
            spread = np.ptp(s.data.xpos[bodies][:, :2], axis=0).max() if len(bodies) > 1 else 0.0
            cam.lookat[:] = [c[0], c[1], max(c[2], 0.15)]
            cam.distance = 0.6 + 1.3 * spread
            cam.azimuth, cam.elevation = -125.0, -22.0
            renderer.update_scene(s.data, camera=cam)
            writer.append_data(renderer.render())

    if a.viewer:
        import mujoco.viewer
        v = mujoco.viewer.launch_passive(sim.model, sim.data)
        prev = on_step

        def on_step(s, prev=prev):  # noqa: F811
            if prev:
                prev(s)
            if s.k % 16 == 0:
                if not v.is_running():
                    srv.stop()
                v.sync()

    try:
        srv.run(a.duration, on_step)
    finally:
        if writer:
            writer.close()
            print(f'[e1-sim] video written to {a.record}')
        if natnet:
            natnet.stop()
        srv.close()
        session.close()


if __name__ == '__main__':
    sys.exit(main())
