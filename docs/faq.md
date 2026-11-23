# FAQ

**The drone does not appear in `e1-cli list`.**
Run `zn` on the console: the WiFi mode must be STA (`p WIFI_MODE 2`), the router IP must be correct
and port 7447/TCP must be open in the PC firewall. Also check that the zenoh-pico version is 1.10.x, the same
as the router's.

**`ros2 topic list` shows nothing.**
Either `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` or the router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) is missing.

**Can I use Fast DDS/Cyclone in the rest of the system?**
Yes. `e1_bridge` talks Zenoh to the drones through the Python library, independently of the RMW. For
a large swarm, however, we recommend `rmw_zenoh` across the whole system.

**What about micro-ROS?** See [Middleware choice](middleware.md).

**I want to fly without mocap.** Manual flight works through the CF-Drone Web RC or through `cmd/manual`.
Autonomous position flight requires some form of external positioning (mocap, UWB, camera + ArUco);
OptiTrack works out of the box (`mocap:=natnet`, see [Motion capture](mocap.md)); `e1_bridge` also accepts any `PoseStamped` on `mocap_topic`.

**Why does the simulator "know" the exact attitude?** The CF-Drone complementary estimator has not been ported;
the simulator uses the ground-truth attitude with sensor noise. To study the estimator, port `estimate.ino` to
`e1_core.firmware`.

**Does it work with the E1 MINI (classic ESP32)?** The firmware does (`pio run -e e1_mini`), but the 3D model and
the physical constants are those of the Nano.
