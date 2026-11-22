# Voo real

1. Mocap com um rigid body por drone (`e1_<id>`, frame FLU, origem no centro da placa).
2. `ros2 launch e1_bringup real.launch.py ids:="[1,2]" mocap_topic:="/vrpn_mocap/e1_{id}/pose"`
3. Confira as poses no RViz.
4. `ros2 service call /e1/takeoff_all e1_msgs/srv/Takeoff "{height: 0.4, duration: 3.0}"`
5. Emergência: `ros2 service call /e1/emergency_stop std_srvs/srv/Trigger` ou `e1-cli estop`.

Checklist: bateria > 3,9 V, óculos de proteção, rede de proteção, primeiro voo com 1 drone a 30 cm.
