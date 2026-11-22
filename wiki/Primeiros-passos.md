# Primeiros passos

```bash
git clone --recursive https://github.com/verlab/e1_nano_ros.git && cd e1_nano_ros
docker compose build
E1_N=4 docker compose --profile demo up router sim bridge controller mission
```

Sem Docker (ROS 2 Jazzy):

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp ros-jazzy-robot-state-publisher ros-jazzy-rviz2
pip install -r e1_core/requirements.txt
colcon build --symlink-install && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch e1_bringup sim.launch.py num:=4 mission:=formation
```

Sem ROS:

```bash
pip install -e "e1_core[sim,zenoh]"; export E1_DESCRIPTION_DIR=$PWD/e1_description
python3 -m e1_core.sim.cli -n 4 --mode peer --listen tcp/127.0.0.1:7447 --connect "" &
python3 examples/swarm_formation.py --n 4
```
