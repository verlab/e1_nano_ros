"""CI helper: wait (up to 15 s) until the drones reach 0.4 m, then require them to stay within 6 cm for 3 s."""
import sys

import rclpy
from e1_msgs.msg import FlightState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

ids = [int(x) for x in sys.argv[1:]] or [1]
rclpy.init()
n = Node('check_altitude')
z, st = {}, {}
for i in ids:
    n.create_subscription(PoseStamped, f'/e1_{i}/pose', lambda m, i=i: z.__setitem__(i, m.pose.position.z), 10)
    n.create_subscription(FlightState, f'/e1_{i}/state', lambda m, i=i: st.__setitem__(i, m), 10)
ok = lambda: len(z) == len(ids) and all(abs(v - 0.4) < 0.06 for v in z.values())  # noqa: E731
now = lambda: n.get_clock().now().nanoseconds * 1e-9  # noqa: E731
end = now() + 15
while now() < end and not ok():
    rclpy.spin_once(n, timeout_sec=0.1)
lo, hi = dict(z), dict(z)
end = now() + 3
while now() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    for i, v in z.items():
        lo[i], hi[i] = min(lo.get(i, v), v), max(hi.get(i, v), v)
print('altitude range over 3 s', {i: (round(lo[i], 3), round(hi[i], 3)) for i in z})
for i, m in st.items():
    print(f'e1_{i}: armed={m.armed} mode={m.mode} cmd_timeout={m.cmd_timeout} failsafe={m.failsafe} thrust={m.thrust:.2f}')
assert len(z) == len(ids) and all(abs(lo[i] - 0.4) < 0.06 and abs(hi[i] - 0.4) < 0.06 for i in ids), (lo, hi)
