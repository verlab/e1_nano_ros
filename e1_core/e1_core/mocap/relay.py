"""``e1-mocap``: stream OptiTrack (NatNet) rigid bodies to the drones' Zenoh key space, no ROS needed.

Each rigid body becomes ``e1/<id>/pose`` (position, orientation, filtered velocity) in a Z-up
world frame. The drone id comes from the rigid body name (``e1_3`` -> 3, pattern ``--name-pattern``)
or, with ``--by-id``, from the Motive streaming id.

    e1-mocap --server 192.168.1.10 --local 192.168.1.20            # multicast (Motive default)
    e1-mocap --server 192.168.1.10 --local 192.168.1.20 --unicast --up y
    e1-mocap --fake 3                                              # 3 fake bodies hovering (no Motive)
"""
from __future__ import annotations

import argparse
import math
import re
import signal
import threading
import time

from .. import protocol as P
from .frames import PoseFilter, to_z_up
from .natnet import NatNetClient, RigidBody, Version


def drone_id(body: RigidBody, pattern: re.Pattern | None):
    if pattern is None:
        return body.id
    m = pattern.fullmatch(body.name or '')
    return int(m.group(1)) if m else None


class MocapRelay:
    """NatNet frames -> ``e1/<id>/pose`` puts, rate-limited per drone."""

    def __init__(self, session, up_axis='z', name_pattern=r'e1_(\d+)', by_id=False, rate_hz=100.0):
        self.s, self.up = session, up_axis
        self.pattern = None if by_id else re.compile(name_pattern)
        self.min_dt = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self.filters, self.pubs, self.t_sent, self.seq = {}, {}, {}, {}
        self.lock = threading.Lock()

    def on_frame(self, frame, bodies, t):
        with self.lock:
            for b in bodies:
                did = drone_id(b, self.pattern)
                if did is None or not b.valid:
                    continue
                pos, quat = to_z_up(b.pos, b.quat, self.up)
                out = self.filters.setdefault(did, PoseFilter()).update(t, pos, quat)
                if out is None or t - self.t_sent.get(did, -math.inf) < self.min_dt:
                    continue
                if did not in self.pubs:
                    self.pubs[did] = self.s.declare_publisher(P.key(did, 'pose'))
                self.seq[did] = (self.seq.get(did, 0) + 1) & 0xFFFF
                p, q, v = out
                self.pubs[did].put(P.Pose(tuple(map(float, p)), tuple(map(float, q)), tuple(map(float, v)),
                                          self.seq[did], int(t * 1000) & 0xFFFFFFFF).encode())
                self.t_sent[did] = t


def main(argv=None):
    ap = argparse.ArgumentParser(prog='e1-mocap', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--server', default='127.0.0.1', help='Motive PC IP')
    ap.add_argument('--local', default='0.0.0.0', help='IP of this PC on the mocap network')
    ap.add_argument('--unicast', action='store_true', help='unicast instead of multicast')
    ap.add_argument('--group', default='239.255.42.99')
    ap.add_argument('--command-port', type=int, default=1510)
    ap.add_argument('--data-port', type=int, default=1511)
    ap.add_argument('--natnet', default=None, help='force NatNet version, e.g. 4.1 (default: ask Motive)')
    ap.add_argument('--up', choices=['z', 'y'], default='z', help="Motive 'Up Axis' setting")
    ap.add_argument('--name-pattern', default=r'e1_(\d+)', help='regex with the drone id as group 1')
    ap.add_argument('--by-id', action='store_true', help='use the Motive streaming id as the drone id')
    ap.add_argument('--rate', type=float, default=100.0, help='max pose rate per drone (Hz)')
    ap.add_argument('--connect', default='tcp/127.0.0.1:7447', help='Zenoh router')
    ap.add_argument('--fake', type=int, default=0, help='run a fake Motive with N hovering bodies (testing)')
    a = ap.parse_args(argv)

    from ..client import connect
    session = connect(a.connect)
    relay = MocapRelay(session, a.up, a.name_pattern, a.by_id, a.rate)
    fake = None
    if a.fake:
        from .server import NatNetServer
        t0 = time.monotonic()

        def bodies():
            t = time.monotonic() - t0
            return [RigidBody(i, (0.5 * i, 0.1 * math.sin(t + i), 0.4), (1, 0, 0, 0)) for i in range(1, a.fake + 1)]
        fake = NatNetServer(bodies, {i: f'e1_{i}' for i in range(1, a.fake + 1)}, ip=a.server,
                            command_port=a.command_port, data_port=a.data_port).start()
        a.unicast = True
    version = Version(*map(int, a.natnet.split('.')[:2])) if a.natnet else None
    client = NatNetClient(a.server, a.local, not a.unicast, a.group, a.command_port, a.data_port, version)
    client.on_frame = relay.on_frame
    ok = client.start()
    print(f'[e1-mocap] server {a.server} ({client.server_name or "no reply"}), NatNet {client.version.major}.{client.version.minor}, '
          f'{"unicast" if a.unicast else "multicast " + a.group}, bodies {client.names or "(no names)"}', flush=True)
    if not ok:
        print('[e1-mocap] warning: Motive did not answer the connect request; waiting for frames anyway', flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    last = 0
    while not stop.wait(2.0):
        n = client.stats.frames
        print(f'[e1-mocap] {(n - last) / 2.0:.0f} frames/s, drones {sorted(relay.pubs)}', flush=True)
        last = n
    client.stop()
    if fake:
        fake.stop()
    session.close()


if __name__ == '__main__':
    main()
