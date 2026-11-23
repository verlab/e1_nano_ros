# 4. Real flight with motion capture

The E1 Nano estimates only its attitude (MPU-6500 IMU). **Position flight requires an external system**:
OptiTrack through the built-in NatNet client (`e1_mocap`), or any source that publishes `PoseStamped`
(Vicon, `vrpn_mocap`, `mocap4r2`, ArUco, UWB). `e1_bridge` forwards the poses to the key `e1/<id>/pose`,
and the position controller closes the loop on the PC. Details: [Motion capture](../mocap.md).

## 4.1 Markers and rigid bodies

* 3 to 4 markers of 3 to 4 mm, attached asymmetrically to the top of the board, outside the propeller discs.
* In the mocap software, create one rigid body per drone, named `e1_<id>`, with an **FLU frame**: x forward
  (arrow on the silkscreen), z up, origin at the board center.

## 4.2 Starting everything

```bash
# router + OptiTrack (NatNet) + driver + controller + RViz
ros2 launch e1_bringup real.launch.py ids:="[1,2,3]" mocap:=natnet server_ip:=192.168.1.10 local_ip:=192.168.1.20
# other pose sources (example with vrpn_mocap)
ros2 launch e1_bringup real.launch.py ids:="[1,2,3]" mocap:=topic mocap_topic:="/vrpn_mocap/e1_{id}/pose"
```

Check in RViz that each drone appears at the correct position and with the correct yaw **before** taking off.
The onboard yaw (no magnetometer) does not need to match the mocap yaw: `e1_swarm_controller` aligns the
two frames (`align_yaw: true`, see [Motion capture](../mocap.md#heading-mocap-vs-onboard-yaw)).

## 4.3 Flying

```bash
ros2 service call /e1_1/takeoff e1_msgs/srv/Takeoff "{height: 0.4, duration: 3.0}"
ros2 run e1_control square_mission --ros-args -p id:=1
ros2 service call /e1/land_all e1_msgs/srv/Land "{duration: 3.0}"
```

Emergency: `ros2 service call /e1/emergency_stop std_srvs/srv/Trigger` (or `e1-cli estop`).
Also keep an `e1-cli estop` ready in a separate terminal; it does not depend on ROS.

## 4.4 Fine tuning

| Symptom | Adjustment |
|---|---|
| Drift in hover | `e1-cli param 1 CTL_TRIM_ROLL 0.01` / `CTL_TRIM_PITCH` (see the CF-Drone comments) |
| Climbs/descends slowly | `max_thrust` in `e1_control/config/controller.yaml` (actual thrust of your motors) |
| Position oscillation | reduce the controller `kp`/`kd`; high mocap latency increases oscillation |
| `cmd_timeout` in flight | saturated WiFi: see [Safety](../safety.md) and the router configuration |
