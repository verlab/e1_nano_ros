"""CI helper: assert that the given drones hover at 0.4 m (reads /e1_<id>/pose for 3 s)."""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

ids = [int(x) for x in sys.argv[1:]] or [1]
rclpy.init()
n = Node('check_altitude')
z = {}
for i in ids:
    n.create_subscription(PoseStamped, f'/e1_{i}/pose', lambda m, i=i: z.__setitem__(i, m.pose.position.z), 10)
end = n.get_clock().now().nanoseconds + 3e9
while n.get_clock().now().nanoseconds < end:
    rclpy.spin_once(n, timeout_sec=0.1)
print('altitudes', z)
assert len(z) == len(ids) and all(abs(v - 0.4) < 0.08 for v in z.values()), z
