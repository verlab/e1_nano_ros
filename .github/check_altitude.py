"""CI helper: wait (up to 15 s) until the given drones hover at 0.4 m; print state flags on failure."""
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
ok = lambda: len(z) == len(ids) and all(abs(v - 0.4) < 0.08 for v in z.values())  # noqa: E731
end = n.get_clock().now().nanoseconds + 15e9
while n.get_clock().now().nanoseconds < end and not ok():
    rclpy.spin_once(n, timeout_sec=0.1)
print('altitudes', {i: round(v, 3) for i, v in z.items()})
for i, m in st.items():
    print(f'e1_{i}: armed={m.armed} mode={m.mode} cmd_timeout={m.cmd_timeout} failsafe={m.failsafe} thrust={m.thrust:.2f}')
assert ok(), z
