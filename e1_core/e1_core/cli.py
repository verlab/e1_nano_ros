"""``e1-cli``: command-line tool for E1 drones on Zenoh (no ROS needed).

    e1-cli list                       # discovered drones (liveliness)
    e1-cli status [--id 3]            # live telemetry table
    e1-cli arm 3 | disarm 3 | disarm all
    e1-cli param 3 CTL_TRIM_ROLL 0.01
    e1-cli console 3 "ps"             # any CF-Drone CLI command, prints e1/3/log
    e1-cli estop                      # disarm every drone (e1/all/cmd/arm)
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from . import protocol as P
from .client import connect
from .math3d import to_euler


def main(argv=None):
    ap = argparse.ArgumentParser(prog='e1-cli', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--connect', default='tcp/127.0.0.1:7447')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list')
    st = sub.add_parser('status'); st.add_argument('--id', default='*'); st.add_argument('--seconds', type=float, default=1e9)
    for c in ('arm', 'disarm'):
        p = sub.add_parser(c); p.add_argument('id')
    p = sub.add_parser('param'); p.add_argument('id'); p.add_argument('name'); p.add_argument('value', type=float)
    p = sub.add_parser('mode'); p.add_argument('id'); p.add_argument('mode', choices=list(P.MODE_NAMES.values()))
    p = sub.add_parser('console'); p.add_argument('id'); p.add_argument('line')
    sub.add_parser('estop')
    a = ap.parse_args(argv)
    s = connect(a.connect)

    if a.cmd == 'list':
        ids = sorted({P.parse_id(str(r.ok.key_expr)) for r in s.liveliness().get(f'{P.PREFIX}/*/alive', timeout=2.0) if r.ok})
        print('drones:', ' '.join(map(str, ids)) or '(none)')
    elif a.cmd == 'status':
        last = {}

        def on_state(smp):
            last[P.parse_id(str(smp.key_expr))] = P.State.decode(smp.payload.to_bytes())
        sub_ = s.declare_subscriber(f'{P.PREFIX}/{a.id}/state', on_state)
        t_end = time.monotonic() + a.seconds
        try:
            while time.monotonic() < t_end:
                time.sleep(0.5)
                print('\033[2J\033[H id  mode  armed  batt   roll  pitch   yaw   thrust  motors')
                for i, x in sorted(last.items()):
                    r, p_, y = np.degrees(to_euler(x.quat))
                    mot = ' '.join(f'{m:.2f}' for m in x.motors)
                    print(f'{i:3d}  {P.MODE_NAMES.get(x.mode, x.mode):5s} {int(x.armed):3d}   {x.battery_v:4.2f}V '
                          f'{r:6.1f} {p_:6.1f} {y:6.1f}  {x.thrust:5.2f}  {mot}')
        except KeyboardInterrupt:
            pass
        sub_.undeclare()
    elif a.cmd in ('arm', 'disarm'):
        s.put(P.key(a.id, 'cmd', 'arm'), P.CmdArm(a.cmd == 'arm').encode())
    elif a.cmd == 'param':
        s.put(P.key(a.id, 'cmd', 'param'), P.CmdParam(a.name, a.value).encode())
    elif a.cmd == 'mode':
        m = {v: k for k, v in P.MODE_NAMES.items()}[a.mode]
        s.put(P.key(a.id, 'cmd', 'mode'), P.CmdMode(m).encode())
    elif a.cmd == 'console':
        sub_ = s.declare_subscriber(P.key(a.id, 'log'), lambda smp: print(smp.payload.to_bytes().decode(errors='replace'), end=''))
        s.put(P.key(a.id, 'cmd', 'console'), a.line.encode())
        time.sleep(1.0)
        sub_.undeclare()
    elif a.cmd == 'estop':
        s.put(P.key(P.BROADCAST, 'cmd', 'arm'), P.CmdArm(False).encode())
        print('disarm broadcast sent')
    time.sleep(0.2)
    s.close()


if __name__ == '__main__':
    sys.exit(main())
