# Real flight

1. Mocap with one rigid body per drone (`e1_<id>`, FLU frame, origin at the board center).
2. `ros2 launch e1_bringup real.launch.py ids:="[1,2]" mocap:=natnet server_ip:=<motive-ip> local_ip:=<this-pc-ip>`
   (OptiTrack; other sources: `mocap:=topic mocap_topic:="/vrpn_mocap/e1_{id}/pose"`)
3. Check the poses in RViz.
4. `ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.4, duration: 3.0}"`
5. Emergency: `ros2 service call /e1/emergency_stop std_srvs/srv/Trigger` or `e1-cli estop`.

Checklist: battery > 3.9 V, safety glasses, safety net, first flight with 1 drone at 30 cm.
