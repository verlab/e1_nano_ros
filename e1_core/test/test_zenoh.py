"""End-to-end over Zenoh: simulator server + Swarm client in the same process (peer mode)."""
import threading
import time

import pytest

pytest.importorskip('mujoco')
zenoh = pytest.importorskip('zenoh')

from e1_core.client import Swarm, connect  # noqa: E402
from e1_core.sim.mujoco_sim import SwarmSim, ZenohSimServer  # noqa: E402


def test_takeoff_land_over_zenoh():
    port = 17447 + int(time.time()) % 1000
    srv_s = connect(None, 'peer', f'tcp/127.0.0.1:{port}')
    sim = SwarmSim(n=2)
    srv = ZenohSimServer(sim, srv_s)
    th = threading.Thread(target=srv.run, kwargs={'duration': 30}, daemon=True)
    th.start()
    cli = connect(f'tcp/127.0.0.1:{port}', 'client')
    sw = Swarm(cli)
    try:
        ids = sw.wait_for(2, timeout=10)
        assert ids == [1, 2]
        sw.takeoff(0.3, duration=2.0)
        sw.sleep(1.0)
        for i in ids:
            assert abs(sw[i].pose.pos[2] - 0.3) < 0.06
        sw.land(duration=2.0)
        sw.sleep(1.0)
        assert all(sw[i].pose.pos[2] < 0.05 for i in ids)
        assert not any(sw[i].state.armed for i in ids)
    finally:
        sw.close()
        srv.stop()
        th.join(5)
        srv.close()
        cli.close()
        srv_s.close()
