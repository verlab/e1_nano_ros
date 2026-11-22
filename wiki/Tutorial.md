# Tutorial passo a passo

1. **Simulação sem ROS.** `python3 -m e1_core.sim.cli -n 1 --mode peer --listen tcp/127.0.0.1:7447 --connect "" --viewer`, depois `python3 examples/fly_square.py`.
2. **ROS 2.** `ros2 launch e1_bringup sim.launch.py num:=2 rviz:=true`, depois `ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.5, duration: 3.0}"`.
3. **Firmware.** `cd firmware && pio run -e e1_nano -t upload`. Calibre com `ca`.
4. **Rede.** `sta <ssid> <senha>`, `p WIFI_MODE 2`, `zn id 1`, `zn connect tcp/<pc>:7447`, `reboot`.
5. **Voo real.** `ros2 launch e1_bringup real.launch.py ids:="[1]" mocap_topic:="/vrpn_mocap/e1_{id}/pose"`.
6. **Enxame.** Mais drones com ids diferentes. `ros2 run e1_control formation_mission --ros-args -p n:=4`.

Detalhes de cada passo: https://verlab.github.io/e1_nano_ros/tutorial/01-simulacao/
