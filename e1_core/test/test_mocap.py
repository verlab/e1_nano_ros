"""NatNet decoding, frame conversions and the full mocap chain (fake Motive -> NatNet -> Zenoh pose)."""
import math
import time

import numpy as np
import pytest

from e1_core.math3d import from_euler, yaw_of
from e1_core.mocap.frames import PoseFilter, from_z_up, to_z_up
from e1_core.mocap.natnet import (NatNetClient, RigidBody, Version, decode_frame, decode_model_def, encode_frame,
                                  encode_model_def)
from e1_core.mocap.server import NatNetServer


@pytest.mark.parametrize('ver', [Version(3, 0), Version(3, 1), Version(4, 0), Version(4, 1)])
def test_frame_roundtrip(ver):
    q = tuple(from_euler(0.1, -0.2, 1.3))
    bodies = [RigidBody(3, (0.1, 0.2, 0.3), q, True, 1e-4), RigidBody(7, (1, 2, 3), (1, 0, 0, 0), False)]
    frame, out = decode_frame(encode_frame(42, bodies, ver), ver)
    assert frame == 42 and [b.id for b in out] == [3, 7]
    assert np.allclose(out[0].pos, bodies[0].pos) and np.allclose(out[0].quat, q, atol=1e-6)
    assert out[0].valid and not out[1].valid


@pytest.mark.parametrize('ver', [Version(3, 0), Version(4, 1)])
def test_model_def_roundtrip(ver):
    names = {1: 'e1_1', 12: 'e1_12', 5: 'wand'}
    assert decode_model_def(encode_model_def(names, ver), ver) == names


def test_y_up_conversion():
    # Motive Y-up: up is +y, so a point 1 m above the floor at Motive z=-2 is Z-up (0, 2, 1)
    p, q = to_z_up((0, 1, -2), (1, 0, 0, 0), 'y')
    assert np.allclose(p, (0, 2, 1)) and np.allclose(np.abs(q), (1, 0, 0, 0), atol=1e-9)
    qz = from_euler(0.05, 0.1, 0.7)
    p2, q2 = to_z_up(*from_z_up((0.3, -0.4, 0.5), qz, 'y'), 'y')
    assert np.allclose(p2, (0.3, -0.4, 0.5)) and abs(abs(np.dot(q2, qz)) - 1) < 1e-9
    assert abs(yaw_of(q2) - 0.7) < 1e-9


def test_pose_filter_velocity_and_outliers():
    f = PoseFilter(alpha=1.0)
    f.update(0.0, (0, 0, 0), (1, 0, 0, 0))
    _, _, v = f.update(0.01, (0.01, 0, 0), (1, 0, 0, 0))
    assert np.allclose(v, (1, 0, 0))
    assert f.update(0.02, (2.0, 0, 0), (1, 0, 0, 0)) is None          # 2 m jump in 10 ms: rejected
    _, q, _ = f.update(0.03, (0.03, 0, 0), (-1, 0, 0, 0))
    assert q[0] > 0                                                    # sign continuity


def test_client_server_unicast():
    bodies = [RigidBody(1, (0.5, 0.0, 0.4), (1, 0, 0, 0)), RigidBody(2, (-0.5, 0.0, 0.4), (1, 0, 0, 0))]
    srv = NatNetServer(lambda: bodies, {1: 'e1_1', 2: 'e1_2'}, ip='127.0.0.1', command_port=21510, data_port=21511).start(200)
    got = []
    c = NatNetClient('127.0.0.1', '127.0.0.1', multicast=False, command_port=21510, data_port=21511)
    c.on_frame = lambda f, b, t: got.append(b)
    try:
        assert c.start()
        assert c.version == Version(4, 1) and c.names == {1: 'e1_1', 2: 'e1_2'}
        end = time.monotonic() + 2
        while len(got) < 20 and time.monotonic() < end:
            time.sleep(0.02)
        assert len(got) >= 20
        assert {b.name for b in got[-1]} == {'e1_1', 'e1_2'}
    finally:
        c.stop()
        srv.stop()


def test_relay_to_zenoh():
    pytest.importorskip('zenoh')
    from e1_core import protocol as P
    from e1_core.client import connect
    from e1_core.mocap.relay import MocapRelay

    a = connect(None, 'peer', 'tcp/127.0.0.1:27447')
    b = connect('tcp/127.0.0.1:27447', 'peer')
    poses = {}

    def on_pose(s):
        poses[P.parse_id(str(s.key_expr))] = P.Pose.decode(s.payload.to_bytes())
    sub = b.declare_subscriber(P.key('*', 'pose'), on_pose)
    time.sleep(0.5)
    relay = MocapRelay(a, up_axis='y', rate_hz=0)
    q = tuple(from_euler(0, 0, 0.0))
    for k in range(30):
        t = k * 0.01
        relay.on_frame(k, [RigidBody(9, (0.1 * t, 0.4, -1.0), q, name='e1_3'),          # Y-up Motive
                           RigidBody(4, (0, 0, 0), q, name='wand')], t)                   # ignored: no match
        time.sleep(0.005)
    time.sleep(0.3)
    sub.undeclare(); a.close(); b.close()
    assert set(poses) == {3}
    p = poses[3]
    assert np.allclose(p.pos, (0.029, 1.0, 0.4), atol=1e-6)
    assert abs(p.vel[0] - 0.1) < 0.02 and not math.isnan(p.vel[1])
