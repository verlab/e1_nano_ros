# Step-by-step tutorial

1. **Simulation without ROS.** `python3 -m e1_core.sim.cli -n 1 --mode peer --listen tcp/127.0.0.1:7447 --connect "" --viewer`, then `python3 examples/fly_square.py`.
2. **ROS 2.** `ros2 launch e1_bringup sim.launch.py num:=2 rviz:=true`, then `ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.5, duration: 3.0}"`.
3. **Firmware.** `cd firmware && pio run -e e1_nano -t upload`. Calibrate with `ca`.
4. **Network.** `sta <ssid> <password>`, `p WIFI_MODE 2`, `zn id 1`, `zn connect tcp/<pc>:7447`, `reboot`.
5. **Real flight.** `ros2 launch e1_bringup real.launch.py ids:="[1]" mocap:=natnet server_ip:=<motive-ip>`.
6. **Swarm.** More drones with different ids. `ros2 run e1_control formation_mission --ros-args -p n:=4`.

Details for each step: https://verlab.github.io/e1_nano_ros/tutorial/01-simulation/
